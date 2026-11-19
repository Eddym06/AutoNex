#!/usr/bin/env python3
"""Depurador del simulador: ejecuta un plano en EXACTAMENTE el mismo simulador (firmware C en WebAssembly,
sensores, robot y juez) y dice DONDE y POR QUE falla, para que cualquier persona o modelo pueda corregirlo
sin leer 100 000 lineas de traza.

    python simulator/tools/depurar.py plano.json                  # ronda estricta: se detiene en la primera infraccion
    python simulator/tools/depurar.py plano.json --indulgente     # anota todas las infracciones y sigue (ve TODOS los fallos)
    python simulator/tools/depurar.py plano.json --json info.json # ademas guarda el informe completo en JSON
    python simulator/tools/depurar.py plano.json --ctrl "planner.speed_cm_s 15;park_margin_cm 1.2"

El plano es el JSON de «Plano <-> JSON» de la interfaz. Necesita Node.js y Python 3 (sin dependencias).

Que detecta (cada hallazgo trae el instante, la posicion, la seccion de la pista, lo que decide el firmware,
los letreros mas cercanos y un mapa del planificador en ese instante):
  - infracciones del juez (lado equivocado, letrero movido, tope tocado, sentido contrario...) y el contexto previo;
  - paradas (el robot sin moverse), bucles de reversa/avance, tramos muy lentos y fases que no avanzan;
  - contactos con paredes, letreros o topes;
  - avisos del plano (los mismos que muestra la interfaz): letreros seguidos, o la salida del cajon y un letrero, que exigen
    cambiar de lado mas rapido de lo que cabe;
  - salida del cajon y parqueo: cuanto tardan y como acaban.
Cada hallazgo indica el comando exacto para mirarlo mas de cerca (`debug_trace.js snap ...`)."""
import argparse
import json
import math
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
WORKER = HERE / "debug_trace.js"
SECCIONES = ["S", "SE", "E", "NE", "N", "NO", "O", "SO"]


def ejecutar(args):
    """Ejecuta el trabajador de Node y devuelve las lineas JSON."""
    try:
        r = subprocess.run(["node", str(WORKER)] + args, capture_output=True, text=True, encoding="utf-8", check=False)
    except FileNotFoundError:
        sys.exit("No se encontro Node.js (node). Instalalo para ejecutar el simulador.")
    if r.returncode != 0:
        sys.exit("El simulador fallo:\n" + (r.stderr or r.stdout))
    return [json.loads(l) for l in r.stdout.splitlines() if l.strip()]


def seccion(n):
    return SECCIONES[n] if isinstance(n, int) and 0 <= n < 8 else "centro/otra"


def colapsar(violaciones):
    """El juez repite 'parado' en cada paso: se agrupa lo consecutivo del mismo tipo."""
    out = []
    for v in violaciones:
        if out and out[-1]["kind"] == v["kind"] and v["t"] - out[-1]["t_ultimo"] < 1.0:
            out[-1]["t_ultimo"] = v["t"]
            continue
        w = dict(v)
        w["t_ultimo"] = v["t"]
        out.append(w)
    return out


def tramos(muestras, condicion, minimo):
    """Intervalos consecutivos de muestras que cumplen la condicion durante al menos `minimo` segundos."""
    out, ini = [], None
    for i, m in enumerate(muestras):
        if condicion(m):
            if ini is None:
                ini = i
        elif ini is not None:
            if muestras[i - 1]["t"] - muestras[ini]["t"] >= minimo:
                out.append((ini, i - 1))
            ini = None
    if ini is not None and muestras[-1]["t"] - muestras[ini]["t"] >= minimo:
        out.append((ini, len(muestras) - 1))
    return out


def resumen_muestra(m):
    cerca = ", ".join("%s #%d a %.0f cm (adelante %.0f, derecha %.0f)" % (s["color"], s["i"], s["dist"], s["forward"], s["right"])
                      for s in m["signs"][:2])
    base = "t=%.1f s pos=(%.0f, %.0f) rumbo %.0f° seccion %s fase %s orden %s v=%.0f cm/s" % (
        m["t"], m["x"], m["z"], m["yaw"], seccion(m["section"]), m["phase"], m["action"], m["v"])
    plan = " | paso previsto %s cm, libre %s cm, escape %s" % (m["pass"], m["freeCm"], ("fase %s (%s intentos)" % (m["escPhase"], m["escapes"])) if m["escaping"] else "no")
    if m["phase"] in ("PARKING", "PARK_EXIT") and m["park"]:
        plan += " | estima en el cajon u=%s w=%s rumbo=%s perdidas=%s" % (m["park"]["u"], m["park"]["w"], m["park"]["theta"], m["park"]["lost"])
    return base + plan + (" | cerca: " + cerca if cerca else "")


def main():
    ap = argparse.ArgumentParser(description="Depura un plano del simulador y senala donde y por que falla.")
    ap.add_argument("plano", help="JSON del plano (Plano <-> JSON de la interfaz)")
    ap.add_argument("--segundos", type=float, default=190.0, help="tiempo maximo simulado (por defecto 190)")
    ap.add_argument("--cada", type=float, default=0.25, help="intervalo de muestreo en segundos (por defecto 0,25)")
    ap.add_argument("--indulgente", action="store_true", help="no terminar en la primera infraccion: listarlas todas")
    ap.add_argument("--ctrl", default="", help='configuracion del controlador "clave valor;clave valor"')
    ap.add_argument("--json", help="guarda el informe completo en este archivo")
    ap.add_argument("--sin-mapas", action="store_true", help="no pedir el mapa del planificador de cada hallazgo (mas rapido)")
    ap.add_argument("--max-hallazgos", type=int, default=12)
    ap.add_argument("--permitir", default="", help="tipos de hallazgo ya documentados que no hacen fallar el comando "
                    "(p.ej. 'parqueo'); se siguen imprimiendo marcados como [permitido]")
    a = ap.parse_args()
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")

    plano = str(Path(a.plano))
    base = ["trace", plano, "--seconds", str(a.segundos), "--every", str(a.cada)]
    if a.ctrl:
        base += ["--ctrl", a.ctrl]
    if a.indulgente:
        base += ["--lenient"]
    lineas = ejecutar(base)
    cab = next(l for l in lineas if l["kind"] == "header")
    fin = next(l for l in lineas if l["kind"] == "end")
    m = [l for l in lineas if l["kind"] == "sample"]
    if not m:
        sys.exit("El simulador no produjo muestras.")

    hallazgos = []

    def anadir(tipo, t, titulo, detalle, causa=None, muestra=None, sugerencia=None):
        hallazgos.append({"tipo": tipo, "t": t, "titulo": titulo, "detalle": detalle, "causa": causa,
                          "muestra": muestra, "sugerencia": sugerencia})

    # ---- infracciones del juez
    for v in colapsar(fin["violations"]):
        previa = [x for x in m if v["t"] - 4 <= x["t"] <= v["t"]]
        ultima = previa[-1] if previa else m[-1]
        causa = None
        if v["kind"] == "wrong-side":
            causa = "El robot paso el letrero %s (%.0f, %.0f) por el lado contrario (lateral %.1f cm)." % (v["color"], v["sign"][0], v["sign"][1], v["lateral"])
            cerca = [w for w in cab["layoutWarnings"] if any(i < len(cab["signs"]) and abs(cab["signs"][i]["x"] - v["sign"][0]) < 1 and abs(cab["signs"][i]["z"] - v["sign"][1]) < 1 for i in w["signs"])]
            for w in cerca:
                causa += " Probable causa del PLANO: " + w["text"]
        elif v["kind"] == "limiter":
            causa = "El robot toco un tope del cajon (termina la ronda y se pierden los puntos de parqueo)."
        elif v["kind"] == "moved":
            causa = "Un letrero se saco de su circulo de 8,5 cm (empujado por el robot)."
        elif v["kind"] == "stalled":
            causa = "El robot estuvo parado mas de 8 s sin haber terminado."
        elif v["kind"] == "opposite":
            causa = "El robot condujo en sentido contrario mas de dos secciones."
        anadir("infraccion", v["t"], "Infraccion del juez: %s" % v["kind"], v["text"], causa, ultima,
               "debug_trace.js snap %s %.1f" % (plano, max(0.0, v["t"] - 1.0)))

    # ---- paradas: el robot sin moverse
    # Girar en el sitio (salida del cajon, pivote, desplazamiento de lado) no es estar parado: se ordena giro.
    for i, j in tramos(m, lambda x: abs(x["v"]) < 1.0 and abs(x.get("yawRate") or 0.0) < 5.0 and not x.get("pivoting")
                       and not x.get("sidestep") and x["phase"] not in ("IDLE", "DONE") and x["t"] > 2.0, 3.0):
        s = m[i]
        detalle = "Parado de t=%.1f a t=%.1f s (%.1f s) en la fase %s." % (s["t"], m[j]["t"], m[j]["t"] - s["t"], s["phase"])
        causa = None
        if s["phase"] == "RUN" and (s["pass"] is not None and s["pass"] < 0):
            causa = "El planificador no encuentra ruta hacia delante (paso previsto = -1, libre %s cm)." % s["freeCm"]
        elif s["phase"] in ("PARKING", "PARK_EXIT") and s["park"] and s["park"]["lost"] > 8:
            causa = "Se perdieron la pared y los topes del cajon (%d barridos sin localizarse)." % s["park"]["lost"]
        elif s["action"] in ("STOP",):
            causa = "El firmware ordena STOP."
        anadir("parada", s["t"], "El robot no se mueve", detalle, causa, s, "debug_trace.js snap %s %.1f" % (plano, s["t"] + 0.5))

    # ---- bucles de reversa y avance en el mismo sitio
    atras = tramos(m, lambda x: x["v"] < -2.0, 0.4)
    i = 0
    while i < len(atras):
        grupo = [atras[i]]
        k = i + 1
        while k < len(atras) and m[atras[k][0]]["t"] - m[grupo[0][0]]["t"] <= 40 and \
                math.hypot(m[atras[k][0]]["x"] - m[grupo[0][0]]["x"], m[atras[k][0]]["z"] - m[grupo[0][0]]["z"]) < 40:
            grupo.append(atras[k])
            k += 1
        if len(grupo) >= 3 and m[grupo[0][0]]["phase"] in ("RUN", "FINISH", "PARK_SEARCH"):
            s = m[grupo[0][0]]
            anadir("bucle", s["t"], "Bucle de reversa y avance en el mismo sitio",
                   "%d reversas en %.0f s sin salir de un radio de 40 cm (t=%.1f a %.1f)." % (len(grupo), m[grupo[-1][1]]["t"] - s["t"], s["t"], m[grupo[-1][1]]["t"]),
                   "El planificador alterna escape (reversa) y avance sin encontrar una salida estable.", s,
                   "debug_trace.js snap %s %.1f" % (plano, s["t"] + 0.5))
            i = k
        else:
            i += 1

    # ---- contactos
    vistos = 0
    for x in m:
        d = x["contactsDelta"]
        if any(d.values()) and vistos < 5:
            vistos += 1
            tipo = [n for n, c in d.items() if c][0]
            anadir("contacto", x["t"], "Contacto con %s" % {"wall": "una pared", "sign": "un letrero", "limiter": "un tope del cajon"}[tipo],
                   "Contacto con %s en t=%.1f s." % (tipo, x["t"]), None, x, "debug_trace.js snap %s %.1f" % (plano, max(0.0, x["t"] - 0.5)))

    # ---- tramos muy lentos (secciones que tardan demasiado)
    ult = None
    for x in m:
        if x["maxNet"] != ult:
            ult = x["maxNet"]
            x["_t_seccion"] = x["t"]
    marcas = [(x["t"], x["maxNet"]) for i, x in enumerate(m) if i == 0 or x["maxNet"] != m[i - 1]["maxNet"]]
    for (t0, n0), (t1, n1) in zip(marcas, marcas[1:]):
        if t1 - t0 > 30 and m[0]["t"] < t0:
            s = next(x for x in m if x["t"] >= t0)
            anadir("lento", t0, "Seccion muy lenta", "La seccion %d tardo %.0f s (de t=%.1f a t=%.1f)." % (n0 + 1, t1 - t0, t0, t1), None, s,
                   "debug_trace.js snap %s %.1f" % (plano, t0 + (t1 - t0) / 2))
    # Con las 24 secciones hechas lo que queda es parquear: eso lo juzga el hallazgo de parqueo, no este.
    if marcas and marcas[-1][1] < 24 and fin["t"] - marcas[-1][0] > 30 and fin["endReason"] == "time":
        s = next(x for x in m if x["t"] >= marcas[-1][0])
        anadir("lento", marcas[-1][0], "Sin avanzar al final de la ronda", "Desde t=%.1f s no se completa ninguna seccion mas (%d/24)." % (marcas[-1][0], marcas[-1][1]), None, s,
               "debug_trace.js snap %s %.1f" % (plano, marcas[-1][0] + 5))

    # ---- salida del cajon y parqueo
    salida = [x for x in m if x["phase"] == "PARK_EXIT"]
    if cab["startInLot"]:
        if not salida:
            anadir("salida", 0.0, "No intento salir del cajon", "El plano arranca dentro del cajon y la fase PARK_EXIT no aparece.",
                   "El firmware no sabe que arranca dentro (start_in_lot) o no arranco.", m[min(8, len(m) - 1)])
        else:
            dur = salida[-1]["t"] - salida[0]["t"]
            if dur > 20 or m[-1]["phase"] == "PARK_EXIT":
                anadir("salida", salida[0]["t"], "La salida del cajon tarda demasiado o no termina", "PARK_EXIT duro %.0f s (t=%.1f a %.1f)." % (dur, salida[0]["t"], salida[-1]["t"]),
                       None, salida[-1], "debug_trace.js snap %s %.1f" % (plano, salida[-1]["t"]))
    aparcando = [x for x in m if x["phase"] in ("PARKING", "PARK_SEARCH")]
    if aparcando and fin["parking"] != "full":
        s = aparcando[-1]
        anadir("parqueo", s["t"], "El parqueo no termina bien", "Resultado del juez: %s. Ultima fase: %s." % (fin["parking"], s["phase"]),
               "Revisa la pose estimada frente a la verdad con `LIDAR System/tests/parktrace` (simulator/tests/parktrace.js).", s,
               "debug_trace.js snap %s %.1f" % (plano, s["t"]))
    elif cab["lot"] and not aparcando and fin["endReason"] in ("time", "stalled", "running"):
        anadir("parqueo", fin["t"], "No llega a buscar el cajon", "La ronda acabo (%s) con %d/24 secciones, sin pasar a PARK_SEARCH." % (fin["endReason"], fin["sections"]), None, m[-1])

    # ---- avisos sobre el plano (geometria, independiente del firmware: los mismos que muestra la interfaz)
    for w in cab["layoutWarnings"]:
        anadir("plano", 0.0, "Plano: cambio de lado imposible (%s)" % w["to"], w["text"],
               "Es un problema del plano, no del firmware: ningun chasis de 13 cm traza ese cambio de lado en esa distancia.", None,
               "Mueve uno de los dos elementos a otro asiento o cambia un color; el generador aleatorio no produce estos casos.")

    orden = {"infraccion": 0, "plano": 1, "parada": 2, "bucle": 3, "contacto": 4, "salida": 5, "parqueo": 6, "lento": 7}
    hallazgos.sort(key=lambda h: (orden.get(h["tipo"], 9), h["t"]))
    hallazgos = hallazgos[:a.max_hallazgos]

    # ---- mapas del planificador en cada hallazgo
    if not a.sin_mapas:
        for h in hallazgos:
            if h["muestra"] is not None and h["tipo"] != "plano":
                try:
                    inst = max(0.0, h["t"] - (0.5 if h["tipo"] == "infraccion" else 0.0))
                    s = ejecutar(["snap", plano, "%.2f" % inst] + (["--ctrl", a.ctrl] if a.ctrl else []))[0]
                    h["instantanea"] = s
                except SystemExit:
                    pass

    # ---- informe
    print("=" * 78)
    print("DEPURACION de %s" % a.plano)
    print("=" * 78)
    print("Sentido: %s | arranca dentro del cajon: %s | cajon: %s | letreros: %d" % (
        "antihorario" if cab["direction"] != "cw" else "horario", "si" if cab["startInLot"] else "no",
        "si" if cab["lot"] else "no", len(cab["signs"])))
    print("Resultado: termino por '%s' a los %.1f s con %d puntos, %d/24 secciones, parqueo: %s, contactos: %s" % (
        fin["endReason"], fin["t"], fin["points"], fin["sections"], fin["parking"], json.dumps(fin["contacts"])))
    print("Modo: %s" % ("indulgente (sigue tras las infracciones)" if a.indulgente else "estricto (se detiene en la primera infraccion)"))
    fases = []
    for x in m:
        if not fases or fases[-1][0] != x["phase"]:
            fases.append([x["phase"], x["t"], x["t"]])
        fases[-1][2] = x["t"]
    print("\nFases: " + "  ".join("%s %.0f-%.0f s" % tuple(f) for f in fases))
    if not hallazgos:
        print("\nSin hallazgos: la ronda transcurre sin infracciones, paradas ni bucles.")
    for n, h in enumerate(hallazgos, 1):
        print("\n[%d] %s  (t=%.1f s)" % (n, h["titulo"].upper(), h["t"]))
        print("    " + h["detalle"])
        if h["causa"]:
            print("    Causa probable: " + h["causa"])
        if h["muestra"]:
            print("    Estado: " + resumen_muestra(h["muestra"]))
        s = h.get("instantanea")
        if s:
            print("    Planificador: referencia de rumbo %s°, %d celdas de cortina; ruta %s; corredor %s" % (
                s["reference"], s["curtains"], s["route"][:4], s["plan"][:5]))
            if s["colored"]:
                print("    Letreros que cree ver: " + ", ".join("%s %s (adelante %s, x %s)%s" % (o["color"], "confirmado" if o["confirmed"] else "sin LiDAR", o["forward"], o["x"], "") for o in s["colored"]))
            if s["truthSigns"]:
                print("    Letreros reales cerca (verdad): " + ", ".join("#%d %s adelante %s derecha %s" % (o["i"], o["color"], o["forward"], o["right"]) for o in s["truthSigns"]))
            print("    Mapa local (%s):" % s["legend"])
            for fila in s["map"]:
                print("      " + fila)
        if h["sugerencia"]:
            print("    Mirar mas: " + h["sugerencia"])
    print("\nPara repetir sin el mapa: python simulator/tools/depurar.py %s --sin-mapas%s" % (a.plano, " --indulgente" if a.indulgente else ""))

    if a.json:
        Path(a.json).write_text(json.dumps({"resultado": fin, "cabecera": cab, "hallazgos": hallazgos, "fases": fases}, ensure_ascii=False, indent=1), encoding="utf-8")
        print("Informe guardado en " + a.json)
    # Tipos que el llamador acepta como conocidos (no hacen fallar el comando). Se usa para
    # dejar en verde una comprobacion automatica de un caso ya documentado, sin ocultarlo: los
    # hallazgos se siguen imprimiendo, con la marca (permitido).
    permitidos = set(x.strip() for x in (a.permitir or "").split(",") if x.strip())
    for h in hallazgos:
        if h["tipo"] in permitidos:
            h["titulo"] = h["titulo"] + " [permitido]"
    grave = ("infraccion", "parada", "bucle", "salida", "parqueo")
    falla = (bool(fin["violations"]) and "infraccion" not in permitidos) or \
            any(h["tipo"] in grave and h["tipo"] not in permitidos for h in hallazgos)
    sys.exit(1 if falla else 0)


if __name__ == "__main__":
    main()
