"""Render geometric RGB565 scenes and exercise the actual firmware C core.

Uses only the Python standard library. Units: cm, seconds, radians.
This is an assumed kinematic model, not hardware certification.
"""
import argparse
import base64
import csv
import html
import json
import math
from pathlib import Path
import struct
import subprocess
import sys
import zlib

ROOT = Path(__file__).resolve().parent.parent
VISION_ROOT = ROOT / "Vision System"
WIDTH, HEIGHT, FOCAL = 160, 120, 118.6
COLORS = {"red": (240, 20, 20), "green": (20, 220, 35),
          "magenta": (230, 20, 220), "blue": (20, 20, 230), "white": (240, 240, 240)}
STEERING = {"RIGHT_SOFT": 12, "RIGHT_HARD": 30, "LEFT_SOFT": -12,
            "LEFT_HARD": -30, "PARK_LEFT": -12, "PARK_RIGHT": 12}


def png(rgb, width=WIDTH, height=HEIGHT):
    def chunk(kind, data):
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data))
    raw = b"".join(b"\0" + rgb[y * width * 3:(y + 1) * width * 3] for y in range(height))
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw)) + chunk(b"IEND", b""))


def render(objects, pose=(0, 0, 0), brightness=1.0):
    """Pinhole camera at body reference point, height 5 cm, horizon y=30.

    Upright front-facing 5 x 10 cm blocks. Rectangle rasterization and ROI
    clipping are retained; distance is camera-to-front-face optical depth.
    """
    rgb = bytearray(bytes((55, 55, 55)) * WIDTH * HEIGHT)
    px, pz, yaw = pose
    projections = []
    for obj in objects:
        dx, dz = obj["x"] - px, obj["z"] - pz
        side = math.cos(yaw) * dx - math.sin(yaw) * dz
        depth = math.sin(yaw) * dx + math.cos(yaw) * dz
        if depth <= 1:
            continue
        cx = WIDTH / 2 + FOCAL * side / depth
        w = FOCAL * obj.get("width", 5) / depth
        bottom = 30 + FOCAL * 5 / depth
        top = bottom - FOCAL * obj.get("height", 10) / depth
        projections.append((depth, cx, w, top, bottom, obj))
    # Paint farther objects first, preserving occlusion by nearer blocks.
    for depth, cx, w, top, bottom, obj in sorted(projections, key=lambda p: -p[0]):
        left, right = max(0, round(cx - w / 2)), min(WIDTH, round(cx + w / 2))
        upper, lower = max(0, round(top)), min(HEIGHT, round(bottom))
        color = bytes(round(c * brightness) for c in COLORS[obj["color"]])
        for y in range(upper, lower):
            for x in range(left, right):
                rgb[(y * WIDTH + x) * 3:(y * WIDTH + x + 1) * 3] = color
    # Encode, then reconstruct the preview from precisely the RGB565 input.
    packed, preview = bytearray(), bytearray()
    for i in range(0, len(rgb), 3):
        r, g, b = rgb[i:i + 3]
        pixel = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)
        packed.extend(struct.pack(">H", pixel))
        preview.extend((((pixel >> 11) & 31) * 255 // 31,
                        ((pixel >> 5) & 63) * 255 // 63, (pixel & 31) * 255 // 31))
    return bytes(packed), png(bytes(preview))


class Core:
    def __init__(self, exe, folder):
        self.folder, self.index = folder, 0
        self.process = subprocess.Popen([str(exe)], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        stderr=subprocess.PIPE, text=True)

    def analyze(self, packed, image, parking=False):
        name = f"frame_{self.index:04d}"
        self.index += 1
        raw = self.folder / (name + ".rgb565")
        raw.write_bytes(packed)
        (self.folder / (name + ".png")).write_bytes(image)
        self.process.stdin.write(f"{int(parking)} {raw}\n")
        self.process.stdin.flush()
        line = self.process.stdout.readline()
        if not line:
            raise RuntimeError("C runner stopped: " + self.process.stderr.read())
        fields = line.strip().split(",")
        result = {"valid": fields[0] == "1", "action": fields[1], "file": name,
                  "image": "data:image/png;base64," + base64.b64encode(image).decode()}
        for i, color in enumerate(("red", "green", "magenta")):
            box = fields[2 + i * 7:9 + i * 7]
            result[color] = dict(zip(("detected", "min_x", "max_x", "min_y", "max_y", "count"),
                                    [bool(int(box[0]))] + [int(v) for v in box[1:6]]))
            result[color]["distance"] = float(box[6])
        return result

    def close(self):
        self.process.stdin.close()
        code = self.process.wait(timeout=10)
        if code:
            raise RuntimeError(self.process.stderr.read())


def overlap_car_block(pose, obj):
    """Separating-axis test: oriented 8 x 12 cm car and 5 x 5 cm block."""
    x, z, yaw = pose
    side, forward = (math.cos(yaw), -math.sin(yaw)), (math.sin(yaw), math.cos(yaw))
    dx, dz = obj["x"] - x, obj["z"] + 2.5 - z
    for ax, az in (side, forward, (1, 0), (0, 1)):
        separation = abs(dx * ax + dz * az)
        car_radius = 4 * abs(side[0] * ax + side[1] * az) + 6 * abs(forward[0] * ax + forward[1] * az)
        block_radius = 2.5 * (abs(ax) + abs(az))
        if separation > car_radius + block_radius:
            return False
    return True


def run(core):
    scenarios, checks, failures = [], [], []

    def check(condition, message):
        checks.append({"passed": bool(condition), "message": message})
        if not condition:
            failures.append(message)

    # Expectations defined by nominal geometry, not by the C decision logic.
    distances = [80, 60, 50, 46, 45, 44, 40, 30, 20, 16, 15, 14, 12, 10, 8, 6]
    for color, side in (("red", "RIGHT"), ("green", "LEFT")):
        frames = []
        for d in distances:
            r = core.analyze(*render([{"color": color, "x": 0, "z": d}]))
            r.update(nominal_distance=d, time=len(frames) * .1, steering=STEERING.get(r["action"], 0))
            frames.append(r)
            check(r["valid"], f"{color} {d}cm: valid frame")
            if d >= 40:
                check(r["action"] == "FORWARD", f"{color} {d}cm: far obstacle, straight")
            elif 20 <= d < 40 or d == 16:
                check(r["action"] == side + "_SOFT", f"{color} {d}cm: soft turn to {side}")
            elif 10 < d <= 15:
                check(r["action"] == side + "_HARD", f"{color} {d}cm: hard turn to {side}")
            elif d <= 10:
                check(r["action"] == "STOP", f"{color} {d}cm: stop, both escape corridors blocked")
            else:
                check(r["action"] in ("FORWARD", side + "_SOFT"), f"{color} {d}cm: quantized 45cm boundary")
            if r[color]["detected"]:
                measured_width = r[color]["max_x"] - r[color]["min_x"] + 1
                check(abs(measured_width - FOCAL * 5 / d) <= 2.01,
                      f"{color} {d}cm: projected width within 2 pixels")
        scenarios.append({"name": f"Aproximación frontal: {color}", "frames": frames, "kind": "distance"})

    # At 30cm sweep the visible target across the full camera field.
    # La regla de color manda: un rojo en el corredor se rebasa SIEMPRE por la derecha y un
    # verde por la izquierda, este donde este dentro del corredor.
    sweep_actions = {
        "red": {20: "FORWARD", 40: "FORWARD", 60: "RIGHT_SOFT", 80: "RIGHT_SOFT",
                100: "RIGHT_SOFT", 120: "RIGHT_SOFT", 140: "FORWARD"},
        "green": {20: "FORWARD", 40: "LEFT_SOFT", 60: "LEFT_SOFT", 80: "LEFT_SOFT",
                  100: "LEFT_SOFT", 120: "FORWARD", 140: "FORWARD"},
    }
    for color in ("red", "green"):
        frames = []
        for cx in (20, 40, 60, 80, 100, 120, 140):
            r = core.analyze(*render([{"color": color, "x": (cx - 80) * 30 / FOCAL, "z": 30}]))
            expected = sweep_actions[color][cx]
            check(r["action"] == expected, f"{color} x={cx}: corridor-aware lateral avoidance {expected}")
            r.update(nominal_distance=30, time=len(frames) * .1, steering=STEERING.get(r["action"], 0))
            frames.append(r)
        scenarios.append({"name": f"Barrido lateral: {color}", "frames": frames, "kind": "distance"})

    cases = [
        ("Sin objetivo", [], False, 1, "FORWARD"),
        ("Azul", [{"color": "blue", "x": 0, "z": 20}], False, 1, "FORWARD"),
        ("Blanco", [{"color": "white", "x": 0, "z": 20}], False, 1, "FORWARD"),
        ("Rojo oscuro", [{"color": "red", "x": 0, "z": 20}], False, .2, "FORWARD"),
        ("Verde oscuro", [{"color": "green", "x": 0, "z": 20}], False, .2, "FORWARD"),
        ("Verde más próximo", [{"color": "red", "x": -5, "z": 30}, {"color": "green", "x": 5, "z": 12}], False, 1, "LEFT_HARD"),
        ("Rojo más próximo", [{"color": "red", "x": -5, "z": 12}, {"color": "green", "x": 5, "z": 30}], False, 1, "RIGHT_HARD"),
        ("Parqueo sin marcador", [], True, 1, "STOP"),
    ]
    frames = []
    for name, objects, parking, brightness, expected in cases:
        r = core.analyze(*render(objects, brightness=brightness), parking=parking)
        check(r["action"] == expected, f"{name}: {expected}")
        r.update(nominal_distance=None, time=len(frames) * .1, steering=STEERING.get(r["action"], 0), label=name)
        frames.append(r)
    scenarios.append({"name": "Distractores, iluminación y prioridad", "frames": frames, "kind": "distance"})

    # Closed loop: render from evolving pose, use C decision, integrate bicycle.
    # These steering angles are simulation assumptions, not PWM calibration.
    for color in ("red", "green"):
        obj = {"color": color, "x": 0, "z": 60}
        pose, frames, collided, minimum_clearance = (0.0, 0.0, 0.0), [], False, float("inf")
        for step in range(160):
            r = core.analyze(*render([obj], pose))
            steering = STEERING.get(r["action"], 0)
            r.update(time=step * .1, pose=list(pose), steering=steering,
                     nominal_distance=math.hypot(obj["x"] - pose[0], obj["z"] - pose[1]))
            collided = collided or overlap_car_block(pose, obj)
            r["collision"] = overlap_car_block(pose, obj)
            minimum_clearance = min(minimum_clearance, math.hypot(obj["x"] - pose[0], obj["z"] + 2.5 - pose[1]))
            frames.append(r)
            if pose[1] > obj["z"] + 18:
                break
            speed, dt, wheelbase = 10, .1, 12
            # Substeps include collision checks between camera samples.
            for _ in range(10):
                x, z, yaw = pose
                yaw += speed / wheelbase * math.tan(math.radians(steering)) * dt / 10
                pose = (x + speed * math.sin(yaw) * dt / 10,
                        z + speed * math.cos(yaw) * dt / 10, yaw)
                collided = collided or overlap_car_block(pose, obj)
        passed = pose[1] > obj["z"] + 18
        check(passed, f"Closed loop {color}: passes obstacle within 16s")
        check(not collided, f"Closed loop {color}: no rectangle overlap")
        check(pose[0] > 6.5 if color == "red" else pose[0] < -6.5,
              f"Closed loop {color}: passes on required side")
        check(any(f["steering"] > 0 if color == "red" else f["steering"] < 0 for f in frames),
              f"Closed loop {color}: actual steering applied")
        scenarios.append({"name": f"Lazo cerrado: evasión {color}", "kind": "closed",
                          "objects": [obj], "frames": frames, "collision": collided,
                          "passed": passed, "minimum_center_distance": minimum_clearance})

    frames = []
    for x, d, expected in ((-8, 30, "PARK_LEFT"), (-4, 25, "PARK_LEFT"),
                            (0, 20, "PARK_APPROACH"), (0, 12, "PARK_APPROACH"),
                            (0, 8, "PARK_READY")):
        r = core.analyze(*render([{"color": "magenta", "x": x, "z": d, "width": 5, "height": 3}]), parking=True)
        check(r["action"] == expected, f"Parking x={x}, depth={d}: {expected}")
        r.update(nominal_distance=d, time=len(frames) * .1, steering=STEERING.get(r["action"], 0))
        frames.append(r)
    scenarios.append({"name": "Guía de parqueo: alineación y aproximación", "frames": frames, "kind": "distance"})
    return scenarios, checks, failures


def overview(folder, scenarios):
    """Contact sheet reconstructed directly from the saved RGB565 inputs."""
    width, height = 996, 572
    rgb = bytearray(bytes((16, 23, 37)) * width * height)
    glyphs = {"0": [14, 17, 19, 21, 25, 17, 14], "1": [4, 12, 4, 4, 4, 4, 14],
              "3": [30, 1, 1, 14, 1, 1, 30], "5": [31, 16, 16, 30, 1, 1, 30],
              "8": [14, 17, 17, 14, 17, 17, 14], "C": [14, 17, 16, 16, 16, 17, 14],
              "M": [17, 27, 21, 21, 17, 17, 17], " ": [0] * 7}
    def pixel(x, y, color):
        rgb[(y * width + x) * 3:(y * width + x + 1) * 3] = bytes(color)
    for row, scenario in enumerate(scenarios[:2]):
        for col, n in enumerate((0, 7, 10)):
            frame = scenario["frames"][n]
            ox, oy = col * 332 + 6, row * 286 + 40
            packed = (folder / "frames" / (frame["file"] + ".rgb565")).read_bytes()
            for y in range(HEIGHT):
                for x in range(WIDTH):
                    i = (y * WIDTH + x) * 2
                    p = (packed[i] << 8) | packed[i + 1]
                    color = (((p >> 11) & 31) * 255 // 31, ((p >> 5) & 63) * 255 // 63, (p & 31) * 255 // 31)
                    for dy in (0, 1):
                        for dx in (0, 1):
                            pixel(ox + x * 2 + dx, oy + y * 2 + dy, color)
            label = str(frame["nominal_distance"]) + " CM"
            for k, char in enumerate(label):
                for gy, bits in enumerate(glyphs[char]):
                    for gx in range(5):
                        if bits & (1 << (4 - gx)):
                            for dy in range(3):
                                for dx in range(3):
                                    pixel(ox + 10 + k * 18 + gx * 3 + dx, oy - 30 + gy * 3 + dy, (235, 241, 250))
    (folder / "overview.png").write_bytes(png(bytes(rgb), width, height))


def report(folder, scenarios, checks, failures):
    payload = {"scenarios": scenarios, "checks": checks, "failures": failures}
    (folder / "results.json").write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
    with (folder / "results.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["scenario", "frame", "time_s", "nominal_distance_cm", "action", "steering_deg", "red_cm", "green_cm", "x_cm", "z_cm", "yaw_deg"])
        for scenario in scenarios:
            for f in scenario["frames"]:
                p = f.get("pose", [None, None, None])
                writer.writerow([scenario["name"], f["file"], f["time"], f["nominal_distance"], f["action"], f["steering"],
                                 f["red"]["distance"], f["green"]["distance"], p[0], p[1], math.degrees(p[2]) if p[2] is not None else ""])
    summary = (f"Visión: {len(checks) - len(failures)}/{len(checks)} comprobaciones aprobadas, "
               f"{len(failures)} fallos, sobre imágenes RGB565 de 160 × 120.")
    lines = ["# Pruebas de visión con imágenes sintéticas", "", summary, ""]
    lines += [f"- {s['name']}: {len(s['frames'])} cuadros" + (f", colisión={s['collision']}, rebasado={s['passed']}" if s["kind"] == "closed" else "") for s in scenarios]
    lines += [""] + ["- FAIL: " + f for f in failures]
    (folder / "summary.md").write_text(chr(10).join(lines) + chr(10), encoding="utf-8")
    rows = [{"name": s["name"], "frames": len(s["frames"]), "ok": not (s["kind"] == "closed" and s.get("collision")),
             "result": (("COLISIÓN" if s["collision"] else "rebasado sin contacto" if s["passed"] else "no rebasado")
                        if s["kind"] == "closed" else "acciones comprobadas")} for s in scenarios]
    sequences = {"summary": summary, "scenarios": rows, "failures": failures}
    sys.path.insert(0, str(ROOT / "simulator"))
    from build_simulator import build_html
    (folder / "report.html").write_text(build_html(sequences), encoding="utf-8")
    overview(folder, scenarios)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runner", default=str(VISION_ROOT / "build" / "simulation" / "sequence_runner.exe"))
    parser.add_argument("--out", default=str(ROOT / "simulator" / "build" / "sequence-simulation"))
    args = parser.parse_args()
    folder = Path(args.out)
    (folder / "frames").mkdir(parents=True, exist_ok=True)
    runner = Path(args.runner)
    shutil_gcc(runner)   # siempre con el vision.c actual
    core = Core(runner, folder / "frames")
    try:
        scenarios, checks, failures = run(core)
    finally:
        core.close()
    print(report(folder, scenarios, checks, failures))
    for failure in failures:
        print("FAIL:", failure)
    return 1 if failures else 0


def shutil_gcc(output):
    """Compila el ejecutor de cuadros con el MISMO vision.c del firmware."""
    output.parent.mkdir(parents=True, exist_ok=True)
    shared = ROOT / "shared"
    cmd = ["gcc", "-std=c11", "-O2", "-I", str(VISION_ROOT / "main"), "-I", str(shared),
           str(VISION_ROOT / "main" / "vision.c"), str(shared / "config_fields.c"), str(shared / "sensor_table_protocol.c"),
           str(VISION_ROOT / "tests" / "sequence_runner.c"), "-lm", "-o", str(output)]
    subprocess.run(cmd, check=True)


if __name__ == "__main__":
    sys.exit(main())
