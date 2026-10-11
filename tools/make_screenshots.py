"""Create proof images from REAL runs:  python tools/make_screenshots.py

 * runs the project's commands, saves their output to terminal_logs/*.txt
 * renders each as a terminal-style PNG in screenshots/terminal/
 * renders key source excerpts as PNGs in screenshots/code/
Needs: pygments (pip install pygments).  Fonts: DejaVu Sans Mono (falls back to PIL default).
"""
import ast
import os
import subprocess
import sys
import textwrap

from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.chdir(ROOT)
PY = sys.executable
FONT_PATHS = ["/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", "C:/Windows/Fonts/consola.ttf",
              "/System/Library/Fonts/Menlo.ttc", "/Library/Fonts/Courier New.ttf"]


def font(size):
    for p in FONT_PATHS:
        if os.path.exists(p):
            return ImageFont.truetype(p, size)
    return ImageFont.load_default()


def window(body: Image.Image, title: str, bg=(30, 31, 38)) -> Image.Image:
    bar_h, pad = 34, 10
    w, h = body.width + 2 * pad, body.height + bar_h + 2 * pad
    img = Image.new("RGB", (w, h), bg)
    d = ImageDraw.Draw(img)
    d.rectangle([0, 0, w, bar_h], fill=(52, 54, 66))
    for i, c in enumerate([(255, 95, 86), (255, 189, 46), (39, 201, 63)]):
        d.ellipse([12 + i * 22, 11, 24 + i * 22, 23], fill=c)
    d.text((w // 2, 17), title, font=font(14), fill=(200, 200, 210), anchor="mm")
    img.paste(body, (pad, bar_h + pad))
    return img


def line_color(s):
    if s.startswith("$"):
        return (130, 220, 130)
    if "FAIL" in s or "Traceback" in s:
        return (255, 110, 110)
    if "PASS" in s or "SUCCESS" in s or " passed" in s or s.strip() == "OK" or s.startswith("A* expands"):
        return (120, 230, 140)
    if s.startswith("[t="):
        return (240, 200, 110)
    if set(s.strip()) <= set("=-") and s.strip():
        return (110, 115, 130)
    return (225, 227, 235)


def terminal_png(cmd: str, output: str, path: str):
    f = font(15)
    lines = [f"$ {cmd}"] + output.rstrip().splitlines()
    cw = f.getlength("M")
    width = int(min(max(len(l) for l in lines), 130) * cw) + 10
    body = Image.new("RGB", (width, 20 * len(lines) + 6), (30, 31, 38))
    d = ImageDraw.Draw(body)
    for i, l in enumerate(lines):
        d.text((4, 3 + 20 * i), l[:130], font=f, fill=line_color(l))
    window(body, "Terminal - AI-Autonomous-Navigation-System").save(path)


def run(cmd: str):
    r = subprocess.run(cmd.replace("python ", f'"{PY}" ', 1), shell=True, capture_output=True, text=True)
    return (r.stdout + r.stderr).replace("\r", "")


TERMINALS = [
    ("01_install_check", "python -c \"import cv2, numpy, matplotlib; print('OK', cv2.__version__, numpy.__version__, matplotlib.__version__)\""),
    ("02_tests", "python run_tests.py"),
    ("03_run_warehouse", "python main.py"),
    ("04_run_city", "python main.py --scenario city --out outputs/city"),
    ("05_run_random_seed3", "python main.py --scenario random --seed 3 --no-gif --out outputs/random"),
    ("06_run_no_prior_map", "python main.py --no-prior --no-gif --out outputs/noprior"),
    ("07_run_dijkstra", "python main.py --algorithm dijkstra --no-gif --out outputs/dijkstra"),
    ("08_benchmark", "python benchmark.py"),
]

CODE = [  # (file, qualified function names or None for whole top part, output name)
    ("src/planner.py", ["plan"], "01_astar_planner"),
    ("src/agent.py", ["NavigationAgent.act"], "02_decision_and_safety_layer"),
    ("src/perception.py", ["OccupancyMap.update"], "03_occupancy_mapping"),
    ("src/perception.py", ["ObstacleDetector.detect"], "04_opencv_obstacle_detection"),
    ("src/controller.py", ["PurePursuit.command"], "05_pure_pursuit_controller"),
    ("src/sensors.py", ["Lidar.scan"], "06_lidar_simulation"),
    ("src/simulation.py", ["Simulation.step"], "07_simulation_loop"),
    ("tests/test_planner.py", None, "08_planner_tests"),
]


def extract(path, names):
    src = open(path, encoding="utf-8").read()
    if names is None:
        return src.strip("\n"), 1
    tree, wanted, chunks, first = ast.parse(src), names[0].split("."), [], None
    nodes = tree.body
    for part in wanted:
        node = next(n for n in nodes if getattr(n, "name", None) == part)
        nodes = getattr(node, "body", [])
    lines = src.splitlines()
    return textwrap.dedent("\n".join(lines[node.lineno - 1:node.end_lineno])), node.lineno


def code_png(path, names, out):
    from pygments import highlight
    from pygments.formatters import ImageFormatter
    from pygments.lexers import PythonLexer
    code, start = extract(path, names)
    fmt = ImageFormatter(font_name="DejaVu Sans Mono", font_size=15, style="monokai", line_numbers=True,
                         line_number_start=start if names is None else 1, image_pad=10, line_pad=3)
    tmp = out + ".tmp"
    with open(tmp, "wb") as fh:
        fh.write(highlight(code, PythonLexer(), fmt))
    body = Image.open(tmp).convert("RGB")
    window(body, path + (f"  -  {names[0]}()" if names else ""), bg=(39, 40, 34)).save(out)
    os.remove(tmp)


def main():
    os.makedirs("terminal_logs", exist_ok=True)
    os.makedirs("screenshots/terminal", exist_ok=True)
    os.makedirs("screenshots/code", exist_ok=True)
    for name, cmd in TERMINALS:
        out = run(cmd)
        with open(f"terminal_logs/{name}.txt", "w", encoding="utf-8") as f:
            f.write(f"$ {cmd}\n{out}")
        terminal_png(cmd, out, f"screenshots/terminal/{name}.png")
        print("saved terminal", name)
    for path, names, name in CODE:
        try:
            code_png(path, names, f"screenshots/code/{name}.png")
            print("saved code", name)
        except Exception as e:                                    # pygments missing etc.
            print("skipped code", name, "->", e)


if __name__ == "__main__":
    main()
