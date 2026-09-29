"""Build the 2-D replica of the 2026 Obstacle Course from the official layout PDF.

    python build_obstacle_course_2026.py --pdf "2026 Course Layouts.pdf"

Walls come from the PDF's vector barrier blocks (tan fill, each drawn as its
exact rotated outline).  The helical ramp's rails, the straight ramp's side
rail and the bank's outer edges are added from the drawing.  The route follows
the drawing's arrows; 3-D features (ramps, gravel, bank, potholes) are flat in
2-D, and the bridge/tunnel overpass becomes a figure-8 crossing.

Writes ``obstacle_course_2026/``: ``map.png`` + ``map.yaml`` (0.05 m, origin at
the bottom-left), ``centerline.csv`` (x_m, y_m in the driving direction) and
``features.json`` (bucket box, hoop lines, no-start zone at the crossing).
Buckets (2-9) and hoops are placed randomly per episode by ``tracks.py``.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import cv2
import numpy as np

PT_PER_FT = 17.9              # 48 ft dimension = 857.4 pt (the 65 ft one agrees within ~1.5%)
M_PER_PT = 0.3048 / PT_PER_FT
RES = 0.05
X0_PT, Y0_PT = 540.0, 300.0   # map region in PDF points
W_PT, H_PT = 1230.0, 900.0
TAN = (1.0, 0.9, 0.6)
# The route and features were read off a 110 dpi render of the page region
# starting at (550, 300) pt; these convert those pixel coordinates.
VIEW_PX_PER_PT = 1.5281
VIEW_ORIGIN_PT = (550.0, 300.0)


def to_cell(x_pt: float, y_pt: float) -> tuple[float, float]:
    return (x_pt - X0_PT) * M_PER_PT / RES, (y_pt - Y0_PT) * M_PER_PT / RES


def view_to_cell(x: float, y: float) -> tuple[float, float]:
    return to_cell(x / VIEW_PX_PER_PT + VIEW_ORIGIN_PT[0], y / VIEW_PX_PER_PT + VIEW_ORIGIN_PT[1])


def view_len(v: float) -> float:
    return v / VIEW_PX_PER_PT * M_PER_PT / RES


def arc(cx, cy, r, a0, a1, steps):
    """Points on a circle in view px; angles in degrees with y up."""
    return [(cx + r * math.cos(math.radians(a)), cy - r * math.sin(math.radians(a)))
            for a in np.linspace(a0, a1, steps)]


# Driving direction per the drawing's arrows (view px).
ROUTE = (
    [(975, 1065), (800, 1065), (600, 1065), (450, 1065), (330, 1065), (250, 1062)]   # start, 20% ramp up
    + arc(185, 1170, 110, 55, 90 + 270, 14)[1:]                  # helical ramp: left turn, 4 ft radius
    + [(290, 1100), (290, 1000), (292, 800), (292, 620), (293, 470), (293, 340)]     # tunnel, 20" narrow
    + arc(455, 330, 164, 180, 90, 7)[1:]                         # narrow curve: 6 ft radius, right turn
    + [(520, 167), (600, 168), (720, 170), (870, 170), (960, 168), (1080, 150), (1150, 128)]   # gravel
    + arc(1195, 215, 105, 115, -115, 12)                         # bank U-turn, right turn, ~1.17 m radius
    + [(1150, 300), (1080, 295), (970, 300), (820, 305), (660, 313), (590, 318)]     # potholes
    + [(535, 350), (510, 420), (520, 520), (520, 650), (520, 780), (545, 845), (600, 895)]   # wide section
    + [(660, 930), (760, 920), (880, 900), (990, 886), (1070, 885)]                 # bucket box, 36" gap
    + [(1180, 885), (1300, 885), (1420, 885), (1520, 887)]                          # hoop section
    + arc(1615, 1030, 130, 100, -135, 14)                        # right loop: 1.45 m radius, right turn
    + [(1440, 1090), (1370, 1066), (1250, 1065), (1100, 1065)]                      # car wash
)
BUCKET_BOX = [(690, 670), (970, 670), (970, 960), (690, 960)]   # inside the box walls, view px
HOOP_LINES = [((1288, 805), (1288, 960)), ((1507, 805), (1507, 960)), ((1620, 1077), (1775, 1077))]
CROSSING = (290, 1065)                                           # bridge over the tunnel


SUPER = 5                     # draw at 1 cm, then a 5 cm cell is wall if more than half covered


def barrier_walls(pdf: Path) -> np.ndarray:
    import fitz
    page = fitz.open(str(pdf))[0]
    shape = (int(H_PT * M_PER_PT / RES), int(W_PT * M_PER_PT / RES))
    walls = np.zeros((shape[0] * SUPER, shape[1] * SUPER), dtype=np.uint8)
    scaled = lambda c: (c[0] * SUPER, c[1] * SUPER)
    for item in page.get_drawings():
        fill = item.get("fill")
        if not fill or tuple(round(v, 2) for v in fill) != TAN:
            continue
        if max(item["rect"].width, item["rect"].height) > 60:     # the yellow bank is drivable
            continue
        pts = []
        for seg in item["items"]:
            if seg[0] == "l":
                pts += [seg[1], seg[2]]
            elif seg[0] == "c":
                pts += [seg[1], seg[2], seg[3], seg[4]]
            elif seg[0] == "re":
                pts += [seg[1].tl, seg[1].tr, seg[1].br, seg[1].bl]
        if len(pts) >= 3:
            hull = cv2.convexHull(np.array([scaled(to_cell(p.x, p.y)) for p in pts], np.float32)).astype(np.int32)
            cv2.fillPoly(walls, [hull], 255)
    t = max(1, int(round(0.08 / RES * SUPER)))
    line = lambda a, b: cv2.line(walls, tuple(int(round(v)) for v in scaled(view_to_cell(*a))),
                                 tuple(int(round(v)) for v in scaled(view_to_cell(*b))), 255, t)
    cx, cy = scaled(view_to_cell(185, 1170))
    centre = (int(round(cx)), int(round(cy)))
    # Helical ramp rails; the arc where the bridge/tunnel path crosses stays open.
    cv2.ellipse(walls, centre, (int(round(view_len(157) * SUPER)),) * 2, 0, -26.6, 360 - 63, 255, t)
    cv2.circle(walls, centre, int(round(view_len(63) * SUPER)), 255, t)
    line((245, 1100), (245, 1172))
    line((330, 1100), (545, 1100))          # 20% ramp: south side rail
    line((1240, 70), (1350, 70))            # bank edges
    line((1350, 70), (1350, 365))
    line((1240, 365), (1350, 365))
    # Seal the seams between adjacent blocks (closing leaves wall faces in place).
    walls = cv2.morphologyEx(walls, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    coverage = cv2.resize(walls.astype(np.float32) / 255.0, (shape[1], shape[0]), interpolation=cv2.INTER_AREA)
    return np.where(coverage > 0.5, 255, 0).astype(np.uint8)


def centerline(spacing: float = 0.05) -> np.ndarray:
    from scipy.interpolate import splev, splprep
    pts = np.array([view_to_cell(x, y) for x, y in ROUTE]) * RES
    tck, _ = splprep([pts[:, 0], pts[:, 1]], s=len(pts) * 0.0025, per=True)
    dense = np.column_stack(splev(np.linspace(0, 1, 6000, endpoint=False), tck))
    loop = np.vstack([dense, dense[:1]])
    s = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(loop, axis=0), axis=1))])
    target = np.arange(0, s[-1], spacing)
    return np.column_stack([np.interp(target, s, loop[:, 0]), np.interp(target, s, loop[:, 1])])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--pdf", type=Path, required=True)
    parser.add_argument("--out", type=Path, default=Path(__file__).resolve().parent / "obstacle_course_2026")
    args = parser.parse_args()
    walls = barrier_walls(args.pdf)
    free = (walls == 0).astype(np.uint8)
    mask = np.zeros((free.shape[0] + 2, free.shape[1] + 2), np.uint8)
    seed = tuple(int(round(v)) for v in view_to_cell(780, 1065))
    cv2.floodFill(free, mask, seed, 2)
    region = free == 2
    if region[0].any() or region[-1].any() or region[:, 0].any() or region[:, -1].any():
        raise SystemExit("drivable area leaks to the map edge")
    rows_m = walls.shape[0] * RES
    to_world = lambda c: (float(c[0] * RES), float(rows_m - c[1] * RES))
    center = centerline()
    world = np.column_stack([center[:, 0], rows_m - center[:, 1]])
    args.out.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(args.out / "map.png"), np.where(region, 255, 0).astype(np.uint8))
    (args.out / "map.yaml").write_text(
        f"image: map.png\nresolution: {RES}\norigin: [0.0, 0.0, 0.0]\nnegate: 0\n"
        "occupied_thresh: 0.65\nfree_thresh: 0.196\n", encoding="utf-8", newline="\n")
    with open(args.out / "centerline.csv", "w", newline="\n", encoding="utf-8") as handle:
        writer = csv.writer(handle, lineterminator="\n")
        writer.writerow(["x_m", "y_m"])
        writer.writerows([(f"{x:.3f}", f"{y:.3f}") for x, y in world])
    features = {
        "source": "2026 Course Layouts.pdf, page 1 (Obstacle Course)",
        "scale_pt_per_ft": PT_PER_FT,
        "bucket_box": [to_world(view_to_cell(*p)) for p in BUCKET_BOX],
        "bucket_count": [2, 9],
        "bucket_diameter_m": 0.30,
        "hoop_lines": [[to_world(view_to_cell(*a)), to_world(view_to_cell(*b))] for a, b in HOOP_LINES],
        "hoop_inner_width_m": 0.55,
        "no_start_center": to_world(view_to_cell(*CROSSING)),
        "no_start_radius_m": 1.0,
        "route_length_m": round(len(world) * 0.05, 2),
    }
    (args.out / "features.json").write_text(json.dumps(features, indent=2) + "\n", encoding="utf-8", newline="\n")
    print(f"wrote {args.out}: map {walls.shape[1]}x{walls.shape[0]} cells, drivable {region.sum() * RES * RES:.1f} m^2, "
          f"route {features['route_length_m']} m")


if __name__ == "__main__":
    main()
