#!/usr/bin/env python3
"""Prototipo del evaluador de rutas para el planificador local del P4.

Reproduce la cadena real del firmware -- barrido de 360 grados, rejilla de
100 x 100 celdas de 5 cm y eleccion de accion -- y anade encima un evaluador de
rutas por abanico de trayectorias con campo de distancias. Sirve para ajustar
los pesos antes de portar la logica a `LIDAR System/main/local_obstacle_map.c`.

Herramienta de escritorio: no se compila en el firmware ni lo sustituye.

    python route_planner_prototype.py --mode demo
    python route_planner_prototype.py --mode sweep
    python route_planner_prototype.py --mode drive
"""
import argparse
import itertools
import math
from dataclasses import dataclass, field, replace
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

try:
    from scipy import ndimage as _ndimage
except ImportError:  # la transformada propia mantiene el prototipo sin dependencias
    _ndimage = None

# --- Geometria y sensor: espejo de local_obstacle_map.h y del simulador ---
MAP_SIZE = 100
CELL_CM = 5.0
CENTER = MAP_SIZE // 2
LIDAR_FORWARD_CM = 12.0
CAMERA_FORWARD_CM = 6.0
CHASSIS_WIDTH_CM = 8.0
CHASSIS_LENGTH_CM = 12.0
CHASSIS_CENTER_CM = 6.0
HALF_WIDTH_CM = CHASSIS_WIDTH_CM / 2.0
HALF_LENGTH_CM = CHASSIS_LENGTH_CM / 2.0
WHEELBASE_CM = 12.0
SAFETY_MARGIN_CM = 4.5

LIDAR_RAYS = 400
LIDAR_STEP_DEG = 0.9
LIDAR_MIN_CM = 5.0
LIDAR_MAX_CM = 1200.0
LOCAL_MAP_MAX_CAMERA_OBJECTS = 12
DETECTED_COLORS = ("red", "green", "magenta")

LANE_HALF_CM = 50.0
WALL_HALF_CM = 5.0

SOFT_DEG = 12.0
HARD_DEG = 30.0

FREE, OCCUPIED = 1, 2
DEG = math.pi / 180.0

ACTIONS = ["STOP", "FORWARD", "RIGHT_SOFT", "RIGHT_HARD", "LEFT_SOFT",
           "LEFT_HARD", "PARK_LEFT", "PARK_RIGHT", "PARK_APPROACH",
           "PARK_READY", "RETURN_LEFT", "RETURN_RIGHT"]
CODE = {name: i for i, name in enumerate(ACTIONS)}


def curvature(steer_deg):
    """Curvatura de una bicicleta de batalla 12 cm (espejo del simulador)."""
    return math.tan(steer_deg * DEG) / WHEELBASE_CM


K_HARD = curvature(HARD_DEG)
# Cada accion es una curvatura constante y un sentido de avance. La reversa
# tiene curvatura cero: se usa como retroceso recto, no gira.
ARC_K = {"FORWARD": 0.0, "RIGHT_SOFT": curvature(SOFT_DEG), "RIGHT_HARD": K_HARD,
         "LEFT_SOFT": -curvature(SOFT_DEG), "LEFT_HARD": -K_HARD, "REVERSE": 0.0}
ARC_DIR = {"FORWARD": 1.0, "RIGHT_SOFT": 1.0, "RIGHT_HARD": 1.0,
           "LEFT_SOFT": 1.0, "LEFT_HARD": 1.0, "REVERSE": -1.0}
# La reticula solo planea hacia adelante: la reversa es recuperacion del
# controlador, no una recomendacion del mapa (asi lo define el firmware).
CANDIDATES = ["FORWARD", "RIGHT_SOFT", "RIGHT_HARD", "LEFT_SOFT", "LEFT_HARD"]


def turn_dir(name):
    """+1 derecha, -1 izquierda, 0 recto."""
    if "RIGHT" in name:
        return 1
    if "LEFT" in name:
        return -1
    return 0


# ---------------------------------------------------------------- pesos ----
@dataclass
class Weights:
    """Pesos del evaluador. Cada magnitud esta normalizada a ~[0, 1], asi que
    los pesos se leen como prioridades relativas: seguridad > mision > suavidad."""
    clearance: float = 4.0   # w1: castiga holgura por debajo del objetivo
    lateral: float = 1.0     # w2: castiga terminar desviado del carril
    heading: float = 3.0     # w3: castiga terminar desalineado
    turn: float = 0.3        # w4: castiga la curvatura media
    progress: float = 1.0    # w5: premia avanzar
    switch: float = 15.0     # w6: castiga cambiar de accion (histeresis)
    dead_end: float = 60.0   # w7: castiga que la ruta muera antes del horizonte
    change: float = 0.4      # w8: castiga cambiar de curvatura entre arcos
    camera: float = 2.0      # w9: sesgo hacia el lado que propone la camara


@dataclass
class Planner:
    """Retícula de `depth` arcos de `arc_cm` cm sobre las cinco acciones."""
    depth: int = 3
    arc_cm: float = 20.0
    step_cm: float = CELL_CM
    discount: float = 0.9
    clearance_pref_cm: float = 12.0
    lateral_scale_cm: float = 25.0
    min_commit_cm: float = 15.0
    step_guard_cm: float = 2.0
    escape_room_cm: float = 30.0
    escape_margin_cm: float = 25.0
    escape_probe_cm: float = 32.0
    escape_turn_rad: float = math.radians(80.0)
    footprint_lateral: int = 3
    footprint_longitudinal: int = 3
    weights: Weights = field(default_factory=Weights)

    @property
    def arc_steps(self):
        return max(1, int(round(self.arc_cm / self.step_cm)))

    @property
    def horizon_cm(self):
        return self.depth * self.arc_cm

    @property
    def steps(self):
        return self.depth * self.arc_steps

    @property
    def sequences(self):
        return list(itertools.product(CANDIDATES, repeat=self.depth))


@dataclass
class Block:
    x: float
    z: float
    width: float = 5.0
    depth: float = 5.0
    color: str = "red"


def lane_walls(z0=-160.0, z1=340.0):
    """Paredes de la franja transitable de 100 cm: cara interior en +-50 cm.

    Son negras a proposito: el detector de color del S3 no las reporta, igual
    que en la pista real.
    """
    return lane_walls_at(LANE_HALF_CM, z0, z1)


# -------------------------------------------------------- proyeccion lidar --
def ray_box(origin, ray, block):
    """Distancia de entrada del rayo a la caja; infinito si no la toca."""
    entry, exit_ = -math.inf, math.inf
    for position, direction, center, half in (
            (origin[0], ray[0], block.x, block.width / 2.0),
            (origin[1], ray[1], block.z, block.depth / 2.0)):
        if abs(direction) < 1e-10:
            if position < center - half or position > center + half:
                return math.inf
            continue
        t0 = (center - half - position) / direction
        t1 = (center + half - position) / direction
        entry = max(entry, min(t0, t1))
        exit_ = min(exit_, max(t0, t1))
        if exit_ < entry:
            return math.inf
    return max(0.0, entry) if exit_ >= 0 else math.inf


def lidar_scan(blocks, pose, scan_index=0):
    """Barrido de 360 grados, 400 rayos de 0,9 grados, con el ruido del simulador.

    Angulo en grados con cero al frente y positivo a la derecha; distancia en
    cm. `x` y `forward` son relativos al lidar, que esta 12 cm delante del eje.
    """
    x, z, yaw = pose
    ox = x + LIDAR_FORWARD_CM * math.sin(yaw)
    oz = z + LIDAR_FORWARD_CM * math.cos(yaw)
    points = []
    for sample in range(LIDAR_RAYS):
        degree = -180.0 + sample * LIDAR_STEP_DEG
        a = degree * DEG
        world_a = a + yaw
        world_ray = (math.sin(world_a), math.cos(world_a))
        nearest = min((ray_box((ox, oz), world_ray, b) for b in blocks), default=math.inf)
        blind = nearest < LIDAR_MIN_CM
        hit = LIDAR_MIN_CM <= nearest <= LIDAR_MAX_CM
        error = 1.0 if nearest <= 100.0 else 4.0
        noise = math.sin((sample + 1) * 127.1 + (scan_index + 1) * 311.7) * error
        distance = (max(LIDAR_MIN_CM, min(LIDAR_MAX_CM, nearest + noise)) if hit
                    else (0.0 if blind else LIDAR_MAX_CM))
        points.append({"angle": degree, "distance": distance, "hit": hit,
                       "valid": not blind, "x": math.sin(a) * distance,
                       "forward": math.cos(a) * distance})
    return points


def camera_objects(blocks, pose):
    """Proyeccion pinhole de los bloques de color y su distancia, como el S3.

    El clasificador del S3 solo reporta blobs de color, asi que los muros
    negros no entran: si entraran, se proyectarian como obstaculos fantasma.
    """
    x, z, yaw = pose
    px = x + CAMERA_FORWARD_CM * math.sin(yaw)
    pz = z + CAMERA_FORWARD_CM * math.cos(yaw)
    focal, width = 118.6, 160
    found = []
    for block in blocks:
        if block.color not in DETECTED_COLORS:
            continue
        dx, dz = block.x - px, block.z - pz
        side = math.cos(yaw) * dx - math.sin(yaw) * dz
        depth = math.sin(yaw) * dx + math.cos(yaw) * dz
        if depth <= 1.0:
            continue
        center_x = width / 2.0 + focal * side / depth
        span = focal * block.width / depth
        found.append({"depth": depth, "min_x": center_x - span / 2.0,
                      "max_x": center_x + span / 2.0, "color": block.color,
                      "distance": 593.0 / max(1.0, span)})
    found.sort(key=lambda o: o["depth"])
    visible = []
    for obj in found:
        if not any(o["min_x"] <= obj["min_x"] and o["max_x"] <= obj["max_x"]
                   for o in visible):
            visible.append(obj)
    return visible


def detect_action(objects):
    """Politica de camara del simulador (color, centro y distancia)."""
    color_code = {"red": 1, "green": 2}
    actionable = sorted((o for o in objects
                         if o["color"] != "magenta"
                         and o["distance"] <= 45 and o["max_x"] >= 45 and o["min_x"] <= 115),
                        key=lambda o: o["distance"])
    if not actionable:
        return "FORWARD"
    near = actionable[0]
    left, right = near["min_x"] >= 56, near["max_x"] <= 104
    for obj in objects:
        if obj is near or obj["color"] == "magenta" or obj["distance"] > near["distance"] + 12:
            continue
        if obj["min_x"] < near["min_x"] and obj["max_x"] >= 56:
            left = False
        if obj["max_x"] > near["max_x"] and obj["min_x"] <= 104:
            right = False
    if not left and not right:
        return "STOP"
    if left != right:
        direction = -1 if left else 1
    else:
        direction = (1 if near["max_x"] < 80 else -1 if near["min_x"] > 80
                     else 1 if color_code.get(near["color"], 1) == 1 else -1)
    if direction > 0:
        return "RIGHT_HARD" if near["distance"] <= 15 else "RIGHT_SOFT"
    return "LEFT_HARD" if near["distance"] <= 15 else "LEFT_SOFT"


# ------------------------------------------------- rejilla (puerto del C) ---
def _lround(value):
    return int(math.floor(value + 0.5)) if value >= 0 else int(math.ceil(value - 0.5))


def to_cell(x_cm, forward_cm):
    cx = _lround(CENTER + x_cm / CELL_CM)
    cy = _lround(CENTER - forward_cm / CELL_CM)
    if cx == MAP_SIZE:
        cx = MAP_SIZE - 1
    if cy == MAP_SIZE:
        cy = MAP_SIZE - 1
    if cx < 0 or cx >= MAP_SIZE or cy < 0 or cy >= MAP_SIZE:
        return None
    return cx, cy


def index(col, row):
    return row * MAP_SIZE + col


def mark_occupied(grid, x, y, source):
    """Marca la celda del retorno. El margen de seguridad no se aplica aqui sino
    al consultar el campo de distancias: dilatar una celda mas sumaba ~5 cm al
    margen de la huella y dejaba el margen efectivo en ~10 cm en vez de los
    4,5 cm previstos, con lo que huecos de 25 cm se volvian intransitables.
    """
    if 0 <= x < MAP_SIZE and 0 <= y < MAP_SIZE:
        cell = index(x, y)
        grid[cell] = (grid[cell] & ~FREE) | OCCUPIED | source


def ray(grid, x0, y0, x1, y1, hit):
    start_x, start_y = x0, y0
    dx, sx = abs(x1 - x0), (1 if x0 < x1 else -1)
    dy, sy = -abs(y1 - y0), (1 if y0 < y1 else -1)
    error = dx + dy
    while True:
        if x0 == x1 and y0 == y1:
            if hit:
                mark_occupied(grid, x0, y0, 0)
            else:
                grid[index(x0, y0)] |= FREE
            return
        if not (x0 == start_x and y0 == start_y) and not (grid[index(x0, y0)] & OCCUPIED):
            grid[index(x0, y0)] |= FREE
        twice = 2 * error
        if twice >= dy:
            error += dy
            x0 += sx
        if twice <= dx:
            error += dx
            y0 += sy


def add_lidar_point(grid, point):
    a = point["angle"] * DEG
    x_cm = math.sin(a) * point["distance"]
    forward_cm = LIDAR_FORWARD_CM + math.cos(a) * point["distance"]
    extent = CENTER * CELL_CM
    axis = max(abs(x_cm), abs(forward_cm))
    hit = axis <= extent
    if not hit:
        scale = extent / axis
        x_cm *= scale
        forward_cm *= scale
    target = to_cell(x_cm, forward_cm)
    origin = to_cell(0.0, LIDAR_FORWARD_CM)
    if not target or not origin:
        return False
    ray(grid, origin[0], origin[1], target[0], target[1], hit)
    return hit


def distance_transform(occupied):
    """Distancia en celdas al obstaculo mas cercano, como lista plana.

    Es la pieza que convierte cada comprobacion de huella en O(1): se calcula
    una vez por escaneo y la comparten todas las rutas candidatas.
    """
    if _ndimage is not None:
        return _ndimage.distance_transform_edt(~occupied).astype(float).ravel().tolist()

    inf = 1e9
    d = [0.0 if occupied[row, col] else inf
         for row in range(MAP_SIZE) for col in range(MAP_SIZE)]
    root2 = math.sqrt(2.0)
    for row in range(MAP_SIZE):
        base, up = row * MAP_SIZE, (row - 1) * MAP_SIZE
        for col in range(MAP_SIZE):
            i = base + col
            best = d[i]
            if col:
                best = min(best, d[i - 1] + 1.0)
            if row:
                if col:
                    best = min(best, d[up + col - 1] + root2)
                best = min(best, d[up + col] + 1.0)
                if col + 1 < MAP_SIZE:
                    best = min(best, d[up + col + 1] + root2)
            d[i] = best
    for row in range(MAP_SIZE - 1, -1, -1):
        base = row * MAP_SIZE
        down = base + MAP_SIZE
        for col in range(MAP_SIZE - 1, -1, -1):
            i = base + col
            best = d[i]
            if col + 1 < MAP_SIZE:
                best = min(best, d[i + 1] + 1.0)
            if row + 1 < MAP_SIZE:
                if col + 1 < MAP_SIZE:
                    best = min(best, d[down + col + 1] + root2)
                best = min(best, d[down + col] + 1.0)
                if col:
                    best = min(best, d[down + col - 1] + root2)
            d[i] = best
    return [min(v, float(MAP_SIZE)) for v in d]


class PlanInput:
    """Una rejilla ya proyectada: lo unico que necesita cualquier planificador.

    El firmware C superpone ademas las detecciones de camara como celdas de
    vision. Aqui no: la distancia de camara (593 / ancho de caja) es mucho mas
    gruesa que la del lidar y, dilatada una celda, siembra obstaculos fantasma
    que estropean la conduccion. Este prototipo aisla la reticula del lidar,
    asi que la camara entra solo como accion propuesta.
    """

    def __init__(self, blocks, pose, scan_index=0):
        grid = [0] * (MAP_SIZE * MAP_SIZE)
        self.lidar_points = 0
        for point in lidar_scan(blocks, pose, scan_index):
            if not point["valid"]:
                continue
            self.lidar_points += 1
            add_lidar_point(grid, point)
        objects = camera_objects(blocks, pose)[:LOCAL_MAP_MAX_CAMERA_OBJECTS]
        self.grid = grid
        self.occupied = np.array([bool(v & OCCUPIED) for v in grid],
                                 dtype=bool).reshape(MAP_SIZE, MAP_SIZE)
        self.occ_flat = self.occupied.ravel().tolist()
        self.dt_flat = distance_transform(self.occupied)
        self.dt = np.array(self.dt_flat, dtype=float).reshape(MAP_SIZE, MAP_SIZE)
        self.camera_action = detect_action(objects) if objects else "FORWARD"

    def footprint_clearance(self, offsets):
        """Holgura de la huella en la pose actual; 0 cm si penetra el obstaculo."""
        best = math.inf
        for ox, of in offsets:
            cell = to_cell(ox, of)
            if not cell:
                return 0.0
            best = min(best, self.dt_flat[index(cell[0], cell[1])] * CELL_CM)
        return best


def footprint_offsets(planner):
    lateral = np.linspace(-HALF_WIDTH_CM, HALF_WIDTH_CM, planner.footprint_lateral)
    longitudinal = np.linspace(0.0, CHASSIS_LENGTH_CM, planner.footprint_longitudinal)
    return [(float(x), float(f)) for f in longitudinal for x in lateral]


# ------------------------------------------------- evaluador de rutas -------
@dataclass
class Route:
    """Una ruta candidata: secuencia de arcos rodada sobre el campo de distancias."""
    actions: tuple
    cost: float
    viable: bool
    clearance_cm: float
    survived_cm: float
    curvature: float = 0.0
    chosen: bool = False
    path: list = field(default_factory=list)

    @property
    def name(self):
        return "+".join(self.actions)

    @property
    def first(self):
        return self.actions[0]


def rotated_step_clearance(data, x, forward, theta, offsets):
    """Holgura de la huella ya girada `theta` rad, con el eje en (x, forward).

    La rejilla esta en coordenadas del chasis, pero una ruta que gira tiene el
    chasis girado: sin rotar los puntos de la huella se mide un rectangulo
    orientado como el chasis inicial y no como el real.
    """
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    worst = math.inf
    for ox, of in offsets:
        cell = to_cell(x + ox * cos_t + of * sin_t, forward + of * cos_t - ox * sin_t)
        if not cell:
            return 0.0
        worst = min(worst, data.dt_flat[index(cell[0], cell[1])] * CELL_CM)
    return worst


def step_clearance(data, x, forward, offsets):
    """Holgura de la huella en la pose actual; 0 cm si penetra o sale del mapa."""
    return rotated_step_clearance(data, x, forward, 0.0, offsets)


def corridor_center_cm(data, forward_cm):
    """Centro del hueco libre que contiene al chasis, en cm de x del chasis.

    Es la linea de referencia que el robot no puede deducir por si mismo: la
    rejilla esta centrada en el robot, asi que sin esto el planificador alinea
    el rumbo pero no corrige la posicion lateral. Si la linea del chasis esta
    tapada, devuelve el centro del hueco libre mas ancho de esa fila.
    """
    cell = to_cell(0.0, forward_cm)
    if not cell:
        return 0.0
    base = cell[1] * MAP_SIZE
    if data.occ_flat[base + CENTER]:
        best_center, best_width, start = 0.0, -1, None
        for col in range(MAP_SIZE + 1):
            occupied = col == MAP_SIZE or data.occ_flat[base + col]
            if not occupied:
                if start is None:
                    start = col
            elif start is not None:
                if col - start > best_width:
                    best_width, best_center = col - start, (start + col - 1) / 2.0
                start = None
        return (best_center - CENTER) * CELL_CM
    left = right = CENTER
    while left > 0 and not data.occ_flat[base + left - 1]:
        left -= 1
    while right < MAP_SIZE - 1 and not data.occ_flat[base + right + 1]:
        right += 1
    return ((left + right) / 2.0 - CENTER) * CELL_CM


def evaluate_sequence(data, planner, actions, previous, offsets, reference_rad, lane_px):
    """Rueda la secuencia de arcos y la puntua; se corta en el primer choque.

    Un arco suelto no puede esquivar y volver a alinearse, por eso la ruta es
    una secuencia: asi el costo incluye el estado final y no solo el trayecto.

    `reference_rad` es la direccion del carril vista desde el chasis (la da el
    IMU) y `lane_px` un punto del centro del carril en coordenadas del chasis.
    Sin ellos el costo solo premiaria avanzar y el carrito giraria en espiral:
    la rejilla esta centrada en el robot y no recuerda hacia donde iba.
    """
    ds, discount = planner.step_cm, planner.discount
    pref = planner.clearance_pref_cm
    total = planner.steps
    cos_ref, sin_ref = math.cos(reference_rad), math.sin(reference_rad)
    lane_py = planner.horizon_cm

    theta = x = forward = 0.0
    weight_sum = penalty = lateral = heading_track = 0.0
    curvature_sum = curvature_change = 0.0
    min_clearance = math.inf
    done = 0
    last_k = 0.0
    blocked = False
    path = [(0.0, 0.0)]
    for name in actions:
        k, direction = ARC_K[name], ARC_DIR[name]
        curvature_change += abs(k - last_k)
        last_k = k
        for _ in range(planner.arc_steps):
            theta += k * ds * direction
            x += math.sin(theta) * ds * direction
            forward += math.cos(theta) * ds * direction
            clearance = rotated_step_clearance(data, x, forward, theta, offsets)
            if clearance <= 0.0:
                blocked = True
                break
            path.append((x, forward))
            w = discount ** done
            deviation = ((x - lane_px) * cos_ref - (forward - lane_py) * sin_ref)
            weight_sum += w
            penalty += w * (max(0.0, pref - clearance) / pref) ** 2
            lateral += w * (deviation / planner.lateral_scale_cm) ** 2
            heading_track += w * ((theta - reference_rad) / (HARD_DEG * DEG)) ** 2
            curvature_sum += abs(k)
            min_clearance = min(min_clearance, clearance)
            done += 1
        if blocked:
            break

    survived_cm = done * ds
    mean_k = curvature_sum / done if done else 0.0
    if done == 0 or survived_cm < planner.min_commit_cm:
        return Route(actions, math.inf, False, 0.0, survived_cm, mean_k, False, path)

    w = planner.weights
    reached = done / total
    camera_biased = (data.camera_action != "STOP"
                     and turn_dir(actions[0]) != turn_dir(data.camera_action))
    cost = (w.clearance * penalty / weight_sum
            + w.lateral * lateral / weight_sum
            + w.heading * heading_track / weight_sum
            + w.turn * mean_k / K_HARD
            + w.change * curvature_change / K_HARD
            + w.dead_end * (1.0 - reached)
            - w.progress * min(forward, planner.horizon_cm) / planner.horizon_cm
            + (w.camera if camera_biased else 0.0)
            + (w.switch if previous and actions[0] != previous else 0.0))
    return Route(actions, cost, True, min_clearance, survived_cm, mean_k, False, path)


def corridor_reference_rad(lane_px, horizon_cm):
    """Direccion del carril vista desde el chasis, deducida del propio escaneo.

    Sustituye al rumbo absoluto del IMU: el firmware no tiene una referencia de
    carril externa, pero si sabe donde esta el centro del carril a la distancia
    del horizonte, y esa direccion sirve para el mismo fin (no salirse ni girar
    en espiral). El signo positivo es a la derecha.
    """
    return math.atan2(lane_px, horizon_cm)


def plan(data, planner, previous=None, offsets=None):
    """Compara toda la retícula de rutas y devuelve la accion inicial ganadora."""
    if data.lidar_points == 0:
        return "STOP", []
    offsets = offsets or footprint_offsets(planner)
    lane_px = corridor_center_cm(data, planner.horizon_cm)
    reference_rad = corridor_reference_rad(lane_px, planner.horizon_cm)
    routes = [evaluate_sequence(data, planner, seq, previous, offsets,
                                reference_rad, lane_px)
              for seq in planner.sequences]
    viable = [r for r in routes if r.viable]
    if not viable:
        return "STOP", routes
    best = min(viable, key=lambda r: (round(r.cost, 9), r.curvature, r.name))
    best.chosen = True
    return best.first, routes


# ----------------------------------------- planificadores de referencia -----
def corridor_clearance(data, angle_deg):
    """Puerto literal de corridor_clearance() del firmware C."""
    a = angle_deg * DEG
    dx, df = math.sin(a), math.cos(a)
    side_x, side_f = math.cos(a), -math.sin(a)
    distance = CELL_CM
    while distance <= 45.0:
        for offset in (-1, 0, 1):
            lateral = offset * CELL_CM
            cell = to_cell(dx * distance + side_x * lateral,
                           df * distance + side_f * lateral)
            if not cell or data.occ_flat[index(cell[0], cell[1])]:
                return distance - CELL_CM
        distance += CELL_CM
    return 45.0


def legacy_c(data, camera_action):
    """Puerto literal de local_obstacle_map_recommend() del firmware C."""
    if data.lidar_points == 0 or camera_action not in CODE:
        return "STOP"
    camera = camera_action
    if camera == "PARK_READY":
        return "STOP"
    forward = corridor_clearance(data, 0.0)
    right = corridor_clearance(data, 35.0)
    left = corridor_clearance(data, -35.0)
    if camera == "STOP":
        if forward >= 20.0:
            return "FORWARD"
        if left < 20.0 and right < 20.0:
            return "STOP"
        return "RIGHT_SOFT" if right >= left else "LEFT_SOFT"
    if camera == "PARK_APPROACH":
        return camera if forward >= 40.0 else "STOP"
    if camera == "PARK_LEFT":
        return camera if left >= 30.0 else "STOP"
    if camera == "PARK_RIGHT":
        return camera if right >= 30.0 else "STOP"
    direction = {"PARK_LEFT": "LEFT_SOFT", "RETURN_LEFT": "LEFT_SOFT",
                 "PARK_RIGHT": "RIGHT_SOFT", "RETURN_RIGHT": "RIGHT_SOFT",
                 "PARK_APPROACH": "FORWARD"}.get(camera, camera)
    if direction == "FORWARD":
        return camera if forward >= 40.0 else "STOP"
    if direction in ("RIGHT_SOFT", "RIGHT_HARD"):
        if right >= 30.0:
            return camera
        if left >= 30.0:
            return "LEFT_HARD" if camera == "RIGHT_HARD" else "LEFT_SOFT"
        return "STOP"
    if direction in ("LEFT_SOFT", "LEFT_HARD"):
        if left >= 30.0:
            return camera
        if right >= 30.0:
            return "RIGHT_HARD" if camera == "LEFT_HARD" else "RIGHT_SOFT"
        return "STOP"
    if left < 25.0 and right < 25.0:
        return "STOP"
    return "RIGHT_SOFT" if right >= left else "LEFT_SOFT"


def lidar_path_clear(scan, action, travel_cm=6.0):
    """Puerto de lidarPathClear() del simulador web: huella sobre 6 cm."""
    direction = -1 if "LEFT" in action else 1 if "RIGHT" in action else 0
    k = curvature(direction * HARD_DEG)
    margin = SAFETY_MARGIN_CM
    travel = 0.0
    while travel <= travel_cm + 1e-9:
        yaw = k * travel
        cx = (1 - math.cos(yaw)) / k if abs(k) > 1e-9 else 0.0
        cz = math.sin(yaw) / k if abs(k) > 1e-9 else travel
        for point in scan:
            if not point["hit"] and point["valid"]:
                continue
            if not point["valid"] and abs(point["angle"]) > 90:
                continue
            a = point["angle"] * DEG
            x = point["x"] if point["hit"] else LIDAR_MIN_CM * math.sin(a)
            z = (LIDAR_FORWARD_CM + point["forward"] if point["hit"]
                 else LIDAR_FORWARD_CM + LIDAR_MIN_CM * math.cos(a))
            side = math.cos(yaw) * (x - cx) - math.sin(yaw) * (z - cz)
            ahead = math.sin(yaw) * (x - cx) + math.cos(yaw) * (z - cz)
            if (abs(side) <= HALF_WIDTH_CM + margin
                    and -margin <= ahead <= CHASSIS_LENGTH_CM + margin):
                return False
        travel += 1.0
    return True


def legacy_js(scan, camera_action, previous=None):
    """Puerto de fusedAction() del simulador web."""
    if camera_action == "STOP":
        if previous and lidar_path_clear(scan, previous):
            return previous
        if lidar_path_clear(scan, "FORWARD"):
            return "FORWARD"
    elif lidar_path_clear(scan, camera_action):
        return camera_action
    if lidar_path_clear(scan, "FORWARD"):
        return "FORWARD"
    if camera_action != "STOP" and turn_dir(camera_action):
        opposite = "RIGHT_HARD" if turn_dir(camera_action) < 0 else "LEFT_HARD"
        if lidar_path_clear(scan, opposite):
            return opposite
    left = lidar_path_clear(scan, "LEFT_HARD")
    right = lidar_path_clear(scan, "RIGHT_HARD")
    if left and not right:
        return "LEFT_HARD"
    if right and not left:
        return "RIGHT_HARD"
    if left and right:
        def count(sign):
            return sum(1 for p in scan if p["hit"] and p["distance"] < 35
                       and p["angle"] * sign > 15 and p["angle"] * sign < 90)
        return "LEFT_HARD" if count(-1) <= count(1) else "RIGHT_HARD"
    return "STOP"


# ------------------------------------------------------- mundo y choques ----
def overlap_chassis_block(pose, block):
    """Eje separador entre el chasis orientado de 8 x 12 cm y el bloque AABB."""
    x, z, yaw = pose
    cx = x + CHASSIS_CENTER_CM * math.sin(yaw)
    cz = z + CHASSIS_CENTER_CM * math.cos(yaw)
    side = (math.cos(yaw), -math.sin(yaw))
    forward = (math.sin(yaw), math.cos(yaw))
    dx, dz = block.x - cx, block.z - cz
    for ax, az in (side, forward, (1.0, 0.0), (0.0, 1.0)):
        separation = abs(dx * ax + dz * az)
        radius = (HALF_WIDTH_CM * abs(side[0] * ax + side[1] * az)
                  + HALF_LENGTH_CM * abs(forward[0] * ax + forward[1] * az))
        block_radius = block.width / 2.0 * abs(ax) + block.depth / 2.0 * abs(az)
        if separation > radius + block_radius:
            return False
    return True


def collides(pose, blocks):
    x, _, yaw = pose
    cx = x + CHASSIS_CENTER_CM * math.sin(yaw)
    reach = HALF_WIDTH_CM * abs(math.cos(yaw)) + HALF_LENGTH_CM * abs(math.sin(yaw))
    if abs(cx) + reach > LANE_HALF_CM:
        return True
    return any(overlap_chassis_block(pose, b) for b in blocks)


def advance(pose, k, step_cm):
    x, z, yaw = pose
    yaw += k * step_cm
    x += math.sin(yaw) * step_cm
    z += math.cos(yaw) * step_cm
    return (x, z, yaw)


def step_pose(pose, action, distance):
    """Avanza o retrocede `distance` cm aplicando la curvatura de la accion."""
    return advance(pose, ARC_K[action], ARC_DIR[action] * distance)


def reached_goal(pose, goal_z, escape_cm=None):
    """Objetivo: llegar al fondo del pasillo o salir por un hueco lateral."""
    if escape_cm is None:
        escape_cm = LANE_HALF_CM + WALL_HALF_CM * 2
    return pose[1] >= goal_z or abs(pose[0]) > escape_cm


# ------------------------------------------------------------ escenarios ----
def scenarios():
    """Instantaneas de decision. Las distancias son las del disparo de evasion:
    mas cerca de 45 cm el giro ya no libra el bloque y ninguna ruta es viable."""
    walls = lane_walls()
    return [
        ("libre", walls),
        ("centrado 110", walls + [Block(0, 110)]),
        ("centrado 80", walls + [Block(0, 80)]),
        ("centrado 60", walls + [Block(0, 60)]),
        ("centrado 45", walls + [Block(0, 45)]),
        ("izquierda 80", walls + [Block(-18, 80)]),
        ("derecha 80", walls + [Block(18, 80)]),
        ("derecha 45", walls + [Block(18, 45)]),
        ("hueco centro 27", walls + [Block(-16, 100), Block(16, 100)]),
        ("hueco centro 19", walls + [Block(-12, 100), Block(12, 100)]),
        ("hueco desplazado", walls + [Block(-26, 100), Block(8, 100)]),
        ("escalonados", walls + [Block(-18, 60), Block(18, 110)]),
        ("dos por delante", walls + [Block(-14, 80), Block(14, 120)]),
        ("bloqueo total", walls + [Block(0, 90, 88, 5)]),
        ("contra pared der", walls + [Block(38, 80)]),
        ("bloque ancho", walls + [Block(0, 90, 12, 5)]),
    ]


def panel_background(ax, data, title, note=""):
    ax.imshow(data.dt, origin="upper", extent=[-250, 250, -250, 250],
              cmap="viridis", vmin=0, vmax=12, alpha=0.9, interpolation="nearest")
    ys, xs = np.nonzero(data.occupied)
    if len(xs):
        ax.scatter((xs - CENTER) * CELL_CM, (CENTER - ys) * CELL_CM, s=4,
                   c="#ff4d4d", marker="s")
    ax.set_title(title + ("\n" + note if note else ""), fontsize=8)
    ax.set_xlim(-95, 95)
    ax.set_ylim(-45, 215)
    ax.set_aspect("equal")
    ax.tick_params(labelsize=6)


def draw_routes(ax, routes):
    for route in routes:
        if route.viable and route.chosen:
            continue
        points = route.path
        ax.plot([p[0] for p in points], [p[1] for p in points],
                color="#7fd4ff" if route.viable else "#ff5c5c",
                lw=0.7, alpha=0.35 if route.viable else 0.5)
    chosen = next((r for r in routes if r.chosen), None)
    if chosen:
        ax.plot([p[0] for p in chosen.path], [p[1] for p in chosen.path],
                color="#38e07b", lw=2.6,
                label=f"{chosen.name}  cost={chosen.cost:.2f}  {chosen.survived_cm:.0f} cm")


# ----------------------------------------------------------- modo demo ------
def run_demo(planner, outdir):
    cases = scenarios()
    fig, axes = plt.subplots(4, 4, figsize=(20, 20))
    rows = []
    for ax, (name, blocks) in zip(axes.ravel(), cases):
        data = PlanInput(blocks, (0.0, 0.0, 0.0))
        new_action, routes = plan(data, planner)
        c_action = legacy_c(data, data.camera_action)
        j_action = legacy_js(lidar_scan(blocks, (0.0, 0.0, 0.0)), data.camera_action)
        pick = next((r for r in routes if r.chosen), None)
        note = (f"camara={data.camera_action}\nC={c_action}  JS={j_action}")
        panel_background(ax, data, name, note)
        draw_routes(ax, routes)
        rows.append((name, data.camera_action, c_action, j_action, new_action,
                     pick.clearance_cm if pick else 0.0,
                     pick.survived_cm if pick else 0.0))
    for ax in axes.ravel()[len(cases):]:
        ax.axis("off")
    axes.ravel()[0].legend(loc="upper right", fontsize=6, framealpha=0.35)
    fig.suptitle(f"Retícula de {len(planner.sequences)} rutas x {planner.depth} arcos "
                 f"de {planner.arc_cm:.0f} cm  verde=elegida  rojo=choca  "
                 f"holgura objetivo={planner.clearance_pref_cm:.0f} cm", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.98))
    out = outdir / "routes.png"
    fig.savefig(out, dpi=110)
    plt.close(fig)

    print(f"{'escenario':<22}{'camara':<14}{'C':<14}{'JS':<14}{'reticula':<14}"
          f"{'avance':>8}{'holgura':>9}")
    print("-" * 95)
    for name, cam, c_act, j_act, n_act, clear, survived in rows:
        flag = "  <-- cambia" if n_act != c_act else ""
        print(f"{name:<22}{cam:<14}{c_act:<14}{j_act:<14}{n_act:<14}"
              f"{survived:>6.0f} cm{clear:>6.0f} cm{flag}")
    print(f"\nimagen: {out}")
    return out


# ---------------------------------------------------------- modo sweep ------
def run_sweep(planner, outdir):
    """Barre los tres pesos que gobiernan la evasion midiendo conduccion real.

    Es la metrica que importa: una decision bonita en una instantanea no sirve
    de nada si el carrito choca dos centimetros despues. Se comparan w1 (holgura
    deseada), w7 (castigo por morir antes del horizonte) y w6 (histeresis).
    """
    walls = lane_walls()
    cases = [("centrado 80", [Block(0, 80)], 115.0),
             ("hueco centro 27", [Block(-16, 100), Block(16, 100)], 135.0),
             ("bloque + escalonado", [Block(0, 80), Block(-22, 140)], 170.0)]
    offsets = footprint_offsets(planner)
    results = []
    for w1 in (1.0, 10.0):
        for w_dead in (10.0, 60.0):
            for w6 in (0.0, 15.0):
                trial = replace(planner, weights=replace(
                    planner.weights, clearance=w1, dead_end=w_dead, switch=w6))
                crashes = stops = flips = 0
                lows = []
                for _, blocks, goal in cases:
                    result = drive(trial, walls + blocks, "fan", offsets, goal,
                                   recovery=True)
                    crashes += result["outcome"] == "choque"
                    stops += result["outcome"] == "STOP"
                    flips += result["flips"]
                    lows.append(min(result["clearance"]) if result["clearance"] else 0.0)
                n = len(cases)
                results.append((w1, w_dead, w6, crashes / n, stops / n,
                                flips / n, min(lows)))
    order = sorted(range(len(results)), key=lambda i: (results[i][3], results[i][4],
                                                       results[i][5], -results[i][6]))
    labels = [f"w1={results[i][0]:g}\nw7={results[i][1]:g}\nw6={results[i][2]:g}"
              for i in order]
    print(f"{'w1':>5}{'w_dead':>8}{'w6':>5}{'choques':>10}{'STOP':>8}"
          f"{'oscila':>8}{'holgura min':>13}")
    print("-" * 57)
    for i in order:
        w1, w_dead, w6, crash, stop, flip, low = results[i]
        print(f"{w1:>5.1f}{w_dead:>8.1f}{w6:>5.1f}{crash:>10.0%}{stop:>8.0%}"
              f"{flip:>8.1f}{low:>10.1f} cm")

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    index = range(len(order))
    crashes = [results[i][3] for i in order]
    stops = [results[i][4] for i in order]
    flips = [results[i][5] for i in order]
    low = [results[i][6] for i in order]
    for ax, values, title, color in (
            (axes[0], crashes, "tasa de choque", "#ff6b6b"),
            (axes[1], stops, "tasa de STOP preventivo", "#ffb454"),
            (axes[2], low, "holgura minima alcanzada (cm)", "#38e07b")):
        ax.bar(index, values, color=color, alpha=0.85)
        ax.set_xticks(list(index))
        ax.set_xticklabels(labels, fontsize=6)
        ax.set_title(title, fontsize=9)
        ax.grid(alpha=0.25, axis="y")
    axes[0].set_ylim(0, 1.05)
    axes[1].set_ylim(0, 1.05)
    fig.suptitle("Barrido de pesos sobre 3 conducciones reales; "
                 f"cambios de direccion por recorrido: {['%.1f' % f for f in flips]}",
                 fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    out = outdir / "sweep.png"
    fig.savefig(out, dpi=130)
    plt.close(fig)
    print(f"\nimagen: {out}")
    return out


# ---------------------------------------------------------- modo drive ------
DRIVE_CASES = [
    ("centrado 80", [Block(0, 80)], 110.0),
    ("hueco centro 27", [Block(-16, 100), Block(16, 100)], 130.0),
    ("escalonados", [Block(-18, 60), Block(18, 110)], 140.0),
    ("bloque + escalonado", [Block(0, 80), Block(-22, 140)], 170.0),
]
DRIVE_KINDS = (("c", "#ffb454", "C actual"),
               ("js", "#7fd4ff", "simulador JS"),
               ("fan", "#38e07b", "reticula"))


def rear_clear(data, offsets):
    """Un paso de 1 cm hacia atras sigue cabiendo."""
    return step_clearance(data, 0.0, -1.0, offsets) > 0.0


def lateral_room(data, direction):
    """Espacio libre a un lado, medido en la fila del centro del chasis."""
    cell = to_cell(0.0, CHASSIS_CENTER_CM)
    if not cell:
        return 0.0
    base = cell[1] * MAP_SIZE
    col, room = CENTER, 0.0
    while 0 <= col + direction < MAP_SIZE and not data.occ_flat[base + col]:
        col += direction
        room += CELL_CM
    return room


def arc_clear(data, offsets, k, distance, sample=2.0):
    """Comprueba que el arco de curvatura `k` tenga `distance` cm por delante."""
    theta = x = forward = 0.0
    travelled = 0.0
    while travelled < distance - 1e-9:
        theta += k * sample
        x += math.sin(theta) * sample
        forward += math.cos(theta) * sample
        if rotated_step_clearance(data, x, forward, theta, offsets) <= 0.0:
            return False
        travelled += sample
    return True


def drive(planner, blocks, kind, offsets, goal_z, escape_cm=None, recovery=False,
          max_ticks=900, step_cm=1.0):
    """Conduce el carrito por el mundo replanificando cada paso de 1 cm.

    Con `recovery`, un STOP entra en el modo de escape del controlador: primero
    retrocede recto un tramo comprometido, despues gira tambien de forma
    comprometida hacia el lado con mas espacio, y solo entonces vuelve a
    planear. Es una maniobra de estado, no una ruta de la reticula: a 60 cm de
    horizonte el planificador local no puede ver que la salida esta detras.
    El trace guarda (x, z, retroceso) por punto para poder pintar la reversa.
    """
    pose, previous, ticks = (0.0, 0.0, 0.0), None, 0
    flips, clearance = 0, []
    trace = [(pose[0], pose[1], False)]
    reverse_cm = 0.0
    escaping = False
    escape_phase = ""
    escape_left = 0.0
    escape_dir = 0
    escape_yaw = None
    last_dir = 0
    outcome = "sigue"
    while ticks < max_ticks and not reached_goal(pose, goal_z, escape_cm):
        data = PlanInput(blocks, pose, ticks)
        clearance.append(data.footprint_clearance(offsets))
        if kind == "fan" and escaping:
            if escape_phase == "back":
                room_right, room_left = lateral_room(data, 1), lateral_room(data, -1)
                if max(room_right, room_left) >= planner.escape_room_cm:
                    escape_dir = 1 if room_right >= room_left else -1
                    escape_phase, escape_left = "margin", planner.escape_margin_cm
                    action = "REVERSE" if rear_clear(data, offsets) else "STOP"
                elif rear_clear(data, offsets):
                    action = "REVERSE"
                else:
                    action = "STOP"
            elif escape_phase == "margin" and escape_left > 1e-9 and rear_clear(data, offsets):
                action = "REVERSE"
                escape_left -= step_cm
            else:
                escape_phase = "turn"
                turn = "RIGHT_HARD" if escape_dir > 0 else "LEFT_HARD"
                progress = abs(pose[2] - escape_yaw) if escape_yaw is not None else 0.0
                remaining = (max(0.0, planner.escape_turn_rad - progress)
                             / abs(K_HARD) + 5.0)
                if arc_clear(data, offsets, ARC_K[turn],
                             min(planner.escape_probe_cm, remaining)):
                    action = turn
                else:
                    escaping = False
                    action, _ = plan(data, planner, previous, offsets=offsets)
        elif kind == "fan":
            action, _ = plan(data, planner, previous, offsets=offsets)
        elif kind == "c":
            action = legacy_c(data, data.camera_action)
        else:
            action = legacy_js(lidar_scan(blocks, pose, ticks), data.camera_action, previous)
        # La ruta se muestrea cada 5 cm, pero el chasis avanza de 1 en 1: hay
        # que verificar el tramo inmediato o el carrito puede colarse dentro del
        # margen entre dos muestras y quedarse luego sin ninguna ruta valida.
        # El escape no pasa por aqui: ya valida todo el arco comprometido.
        if (not escaping and action != "STOP"
                and not arc_clear(data, offsets, ARC_K[action], planner.step_guard_cm)):
            action = "STOP"
        if action == "STOP" and recovery and not escaping and kind == "fan":
            escaping, escape_phase, escape_dir, escape_yaw = True, "back", 0, None
            action = "REVERSE" if rear_clear(data, offsets) else "STOP"
        if action == "STOP":
            outcome = "STOP"
            break
        direction = turn_dir(action)
        if direction and last_dir and direction != last_dir:
            flips += 1
        if direction:
            last_dir = direction
        backward = action == "REVERSE"
        if backward:
            reverse_cm += step_cm
        pose = step_pose(pose, action, step_cm)
        trace.append((pose[0], pose[1], backward))
        previous = action
        ticks += 1
        # El giro de salida se completa antes de volver a planear: si no, el
        # planificador local reanudaria recto al primer centimetro.
        if escaping and escape_phase == "turn":
            if escape_yaw is None:
                escape_yaw = pose[2]
            elif abs(pose[2] - escape_yaw) >= planner.escape_turn_rad:
                escaping, escape_phase, escape_dir, escape_yaw = False, "", 0, None
        if collides(pose, blocks):
            outcome = "choque"
            break
    else:
        outcome = "recorrido" if reached_goal(pose, goal_z, escape_cm) else "sin avanzar"
    return {"outcome": outcome, "ticks": ticks, "flips": flips,
            "clearance": clearance, "trace": trace, "pose": pose,
            "reverse_cm": reverse_cm}


def plot_trace(ax, result, color, label):
    """Dibuja el recorrido; los tramos de reversa van en guion naranja."""
    trace = result["trace"]
    for start, end in zip(trace, trace[1:]):
        ax.plot([start[0], end[0]], [start[1], end[1]],
                color="#ff9f43" if end[2] else color, lw=1.8, ls="--" if end[2] else "-")
    ax.plot([trace[-1][0]], [trace[-1][1]], marker="o", color=color, ms=4,
            label=f"{label}: {result['outcome']}"
                  + (f", {result['reverse_cm']:.0f} cm atras" if result["reverse_cm"] else ""))


def draw_world(ax, blocks, xlim, zlim):
    for block in blocks:
        ax.add_patch(plt.Rectangle((block.x - block.width / 2, block.z - block.depth / 2),
                                   block.width, block.depth,
                                   color="#ff4d4d" if block.color != "black" else "#7c8aa0"))
    ax.set_xlim(*xlim)
    ax.set_ylim(*zlim)
    ax.set_aspect("equal")
    ax.grid(alpha=0.2)


def run_drive(planner, outdir):
    offsets = footprint_offsets(planner)
    walls = lane_walls()
    fig, axes = plt.subplots(1, len(DRIVE_CASES), figsize=(5 * len(DRIVE_CASES), 5.2))
    print(f"{'escenario':<20}{'planificador':<14}{'resultado':<12}{'ciclos':>7}"
          f"{'cambios':>9}{'holgura min':>13}")
    print("-" * 75)
    for ax, (name, blocks, goal_z) in zip(axes.ravel(), DRIVE_CASES):
        world = walls + blocks
        for kind, color, label in DRIVE_KINDS:
            result = drive(planner, world, kind, offsets, goal_z,
                           recovery=kind == "fan")
            low = min(result["clearance"]) if result["clearance"] else float("nan")
            print(f"{name:<20}{label:<14}{result['outcome']:<12}{result['ticks']:>7}"
                  f"{result['flips']:>9}{low:>10.0f} cm")
            plot_trace(ax, result, color, label)
        draw_world(ax, blocks, (-70, 70), (-10, 125))
        ax.set_title(name, fontsize=9)
        ax.legend(fontsize=7)
    fig.suptitle("Trazado del carrito: C actual vs. simulador JS vs. reticula propuesta",
                 fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    out = outdir / "drive.png"
    fig.savefig(out, dpi=110)
    plt.close(fig)
    print(f"\nimagen: {out}")
    return out


# -------------------------------------------------------- modo complejo -----
def vertical_wall(side, half_lane_cm, z0, z1):
    """Pared vertical a la altura del carril; `side` es +1 derecha, -1 izquierda."""
    x = side * (half_lane_cm + WALL_HALF_CM)
    return Block(x, (z0 + z1) / 2.0, 2 * WALL_HALF_CM, z1 - z0, "black")


def lane_walls_at(half_lane_cm, z0=-70.0, z1=340.0, left_gaps=(), right_gaps=()):
    """Paredes del carril con huecos laterales: cada hueco es (inicio, fin) en z."""
    def segments(side, gaps):
        pieces, start = [], z0
        for gap_start, gap_end in sorted(gaps):
            if gap_start > start:
                pieces.append(vertical_wall(side, half_lane_cm, start, gap_start))
            start = max(start, gap_end)
        if start < z1:
            pieces.append(vertical_wall(side, half_lane_cm, start, z1))
        return pieces
    return segments(-1, left_gaps) + segments(1, right_gaps)


def gap_blocks(half_gap_cm, z, depth=5.0, color="black"):
    """Dos bloques que llegan a las paredes y dejan un hueco central exacto."""
    width = LANE_HALF_CM - half_gap_cm
    center = (LANE_HALF_CM + half_gap_cm) / 2.0
    return [Block(center, z, width, depth, color),
            Block(-center, z, width, depth, color)]


def pocket_case():
    """Bolsillo estrecho y sin salida delantera: 46 cm de ancho no admiten un
    giro de 180, asi que la unica via es retroceder hasta el hueco lateral."""
    half = 23.0
    return {
        "name": "reversa y salida lateral",
        "blocks": (lane_walls_at(half, -80.0, 100.0, right_gaps=((-70.0, -15.0),))
                   + [Block(0, 45, 2 * half, 5, "black"),
                      Block(0, -82.5, 2 * half + 6, 5, "black")]),
        "goal_z": 120.0,
        "escape_cm": 26.0,
        "note": "bolsillo de 46 cm sin salida al frente: hay que retroceder "
                "y salir por el hueco derecho",
    }


def complex_cases():
    """Casos dificiles: muchos bloques, huecos precisos y un bolsillo sin salida
    delantera donde la unica solucion es retroceder y salir por otro lado."""
    lane = lane_walls() + [Block(0, -162.5, 2 * LANE_HALF_CM + 5, 5, "black")]
    pocket = pocket_case()
    return [
        ("cinco bloques", lane + [
            Block(-18, 45), Block(14, 62), Block(-6, 88), Block(20, 112),
            Block(-20, 136)], 200.0, None,
         "5 bloques alternos: eslalon completo"),
        ("hueco 34 cm", lane + gap_blocks(17.0, 60), 130.0, None,
         "holgura 1,7x el ancho del chasis"),
        ("hueco 29 cm", lane + gap_blocks(14.5, 60), 130.0, None,
         "holgura 1,45x el ancho del chasis"),
        ("hueco 24 cm", lane + gap_blocks(12.0, 60), 130.0, None,
         "holgura 1,2x el ancho del chasis"),
        ("hueco escalonado", lane + [
            Block(-16, 45, 34), Block(16, 90, 34)], 170.0, None,
         "dos huecos de 16 cm desfasados en z"),
        ("pasillo 8 cm", lane + gap_blocks(4.0, 50, depth=40), 140.0, None,
         "pasillo recto de 8 cm, igual al ancho del chasis"),
        (pocket["name"], pocket["blocks"], pocket["goal_z"], pocket["escape_cm"],
         pocket["note"]),
    ]


def run_complex(planner, outdir):
    cases = complex_cases()
    offsets = footprint_offsets(planner)
    columns = 3
    rows = (len(cases) + columns - 1) // columns
    fig, axes = plt.subplots(rows, columns, figsize=(6.5 * columns, 5.6 * rows))
    print(f"{'escenario':<26}{'planificador':<14}{'resultado':<11}{'ciclos':>7}"
          f"{'reversa':>9}{'cambios':>9}{'holgura min':>13}")
    print("-" * 89)
    for ax, (name, blocks, goal_z, escape_cm, note) in zip(axes.ravel(), cases):
        for kind, color, label in DRIVE_KINDS:
            result = drive(planner, blocks, kind, offsets, goal_z, escape_cm,
                           recovery=kind == "fan")
            low = min(result["clearance"]) if result["clearance"] else float("nan")
            print(f"{name:<26}{label:<14}{result['outcome']:<11}{result['ticks']:>7}"
                  f"{result['reverse_cm']:>7.0f} cm{result['flips']:>9}{low:>10.0f} cm")
            plot_trace(ax, result, color, label)
        draw_world(ax, blocks, (-75, 75), (-115, max(goal_z, 60) + 15))
        ax.set_title(f"{name}\n{note}", fontsize=9)
        ax.legend(fontsize=6.5, loc="upper left")
    for ax in axes.ravel()[len(cases):]:
        ax.axis("off")
    fig.suptitle("Casos complejos: eslalon de 5 bloques, huecos de alta precision y "
                 "salida por reversa", fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    out = outdir / "complex.png"
    fig.savefig(out, dpi=110)
    plt.close(fig)
    print(f"\nimagen: {out}")
    return out


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mode",
                        choices=("demo", "sweep", "drive", "complex", "all"),
                        default="demo")
    parser.add_argument("--depth", type=int, default=3)
    parser.add_argument("--arc", type=float, default=20.0)
    parser.add_argument("--clearance-pref", type=float, default=12.0)
    parser.add_argument("--w-clearance", type=float, default=4.0)
    parser.add_argument("--w-lateral", type=float, default=1.0)
    parser.add_argument("--w-heading", type=float, default=3.0)
    parser.add_argument("--w-turn", type=float, default=0.3)
    parser.add_argument("--w-progress", type=float, default=1.0)
    parser.add_argument("--w-switch", type=float, default=15.0)
    parser.add_argument("--w-dead-end", type=float, default=60.0)
    parser.add_argument("--w-change", type=float, default=0.4)
    parser.add_argument("--w-camera", type=float, default=2.0)
    parser.add_argument("--min-commit", type=float, default=15.0)
    args = parser.parse_args()

    planner = Planner(
        depth=args.depth, arc_cm=args.arc, clearance_pref_cm=args.clearance_pref,
        min_commit_cm=args.min_commit,
        weights=Weights(clearance=args.w_clearance, lateral=args.w_lateral,
                        heading=args.w_heading, turn=args.w_turn,
                        progress=args.w_progress, switch=args.w_switch,
                        dead_end=args.w_dead_end, change=args.w_change,
                        camera=args.w_camera))
    outdir = Path(__file__).resolve().parent.parent / "build" / "route-planner"
    outdir.mkdir(parents=True, exist_ok=True)
    if args.mode in ("demo", "all"):
        run_demo(planner, outdir)
    if args.mode in ("sweep", "all"):
        run_sweep(planner, outdir)
    if args.mode in ("drive", "all"):
        run_drive(planner, outdir)
    if args.mode in ("complex", "all"):
        run_complex(planner, outdir)


if __name__ == "__main__":
    main()
