"""Ensambla el simulador en UN archivo HTML que se abre con doble clic.

Une los modulos de simulator/web (geometria, pista, sensores, robot, juez, envoltorio
del firmware, orquestador, interfaz) y el firmware compilado (wro_core.wasm,
en base64) dentro de index.template.html. El firmware que corre en el navegador es
el mismo codigo C del S3 y del P4: no hay una copia en JavaScript.

    python simulator/build_simulator.py [salida.html]
"""
import base64
import datetime
import hashlib
import json
import sys
from pathlib import Path

WEB = Path(__file__).resolve().parent / "web"
ORDER = ["geometry.js", "rng.js", "field.js", "sensors.js", "robot.js", "judge.js", "core.js", "sim.js", "app.js"]


def build_html(sequences=None):
    wasm = (WEB / "wro_core.wasm").read_bytes()
    scripts = ["window.WASM_B64=" + json.dumps(base64.b64encode(wasm).decode("ascii")) + ";"]
    if sequences is not None:
        scripts.append("window.SEQ=" + json.dumps(sequences, ensure_ascii=False).replace("</", "<\\/") + ";")
    for name in ORDER:
        source = (WEB / name).read_text(encoding="utf-8")
        # Los modulos de Node exportan con module.exports; en el navegador no existe.
        scripts.append("/* ---- " + name + " ---- */\n" + source)
    template = (WEB / "index.template.html").read_text(encoding="utf-8")
    # Marca de compilacion: permite ver en la pagina si el navegador cargo la version nueva.
    stamp = datetime.datetime.now().strftime("%d/%m %H:%M") + " · fw " + hashlib.sha1(wasm).hexdigest()[:7]
    html = template.replace("/*__SCRIPTS__*/", "\n".join(scripts).replace("</script", "<\\/script"))
    return html.replace("__BUILD__", stamp)


def main():
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parent / "build" / "simulador.html"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(build_html(), encoding="utf-8")
    print(f"{out} ({out.stat().st_size / 1024:.0f} KB)")


if __name__ == "__main__":
    main()
