#!/usr/bin/env python3
"""Export the 2026 Obstacle Course replica as a 3-D scene description for AutoDRIVE (Unity).

    python export_course3d.py [--seed N] [--out course3d.json]

Writes one JSON file that ``Editor/LaksaCourseBuilder.cs`` turns into a Unity
scene.  Units are metres in the course map frame (x right, y up, z = height);
the Unity script maps (x, y, z) -> Unity (x, z, y).

  walls         straw-bale barrier outlines (polygons from the 2-D replica map),
                extruded to bale height
  buckets       5-gallon buckets (cylinders) placed as in training
  hoops         pairs of posts with a top bar
  start         start line pose (rear axle on the centreline, heading)
  start_signal  the North American start signal: Oasis Blue board 559 x 1219 mm,
                standing on the ground at the right track edge (in line with the
                bales), 8 ft (2.438 m) down-track from the start line, facing the
                car; pinwheel pivot 813 mm up, arms 610 mm tip to tip with
                204 mm discs; red arm showing at the start, green swings out.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import tempfile
from pathlib import Path

import cv2
import numpy as np

TRAINING = Path(__file__).resolve().parents[2] / "laksa_learned_driver" / "training"
sys.path.insert(0, str(TRAINING))
sys.path.insert(0, str(TRAINING.parent))
import tracks  # noqa: E402

BALE_HEIGHT_M = 0.36            # a small square bale on its side: ~14 in
BUCKET_HEIGHT_M = 0.37          # 5-gallon bucket
SIGNAL_DOWNTRACK_M = 8 * 0.3048
SIGNAL = {"board_w": 0.559, "board_h": 1.219, "board_t": 0.019, "pivot_h": 0.813,
          "arm_len": 0.610, "arm_w": 0.102, "disc_r": 0.102,
          "colors": {"board": [0.30, 0.47, 0.70], "red": [0.78, 0.14, 0.16], "green": [0.30, 0.47, 0.24]}}


def wall_polygons(track, min_area_m2: float = 0.01, epsilon_m: float = 0.03):
    """Occupied regions of the map as simplified polygons (metres, map frame)."""
    occupied = (~track.free).astype(np.uint8) * 255
    contours, hierarchy = cv2.findContours(occupied, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    res, (ox, oy) = track.resolution, track.origin
    h = occupied.shape[0]
    polys = []
    for i, c in enumerate(contours):
        if hierarchy is not None and hierarchy[0][i][3] != -1:
            continue                                   # holes: skip (bale blocks have none)
        if cv2.contourArea(c) * res * res < min_area_m2:
            continue
        c = cv2.approxPolyDP(c, epsilon_m / res, True).reshape(-1, 2)
        pts = [[round(ox + (px + 0.5) * res, 3), round(oy + (h - py - 0.5) * res, 3)] for px, py in c]
        if len(pts) >= 3:
            polys.append(pts)
    return polys


def outer_frame(polys):
    """The map's outer border is one huge polygon; drop it (it is the floor's edge)."""
    areas = [abs(cv2.contourArea(np.array(p, np.float32))) for p in polys]
    if not areas:
        return polys, None
    i = int(np.argmax(areas))
    xs = [x for x, _ in polys[i]]
    ys = [y for _, y in polys[i]]
    return polys[:i] + polys[i + 1:], [min(xs), min(ys), max(xs), max(ys)]


def start_signal(track, heading: float, start_xy: np.ndarray):
    """Board centre and yaw: 8 ft down-track, its left edge on the right-hand bale line."""
    center = track.center
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(center, axis=0), axis=1))])
    i = int(np.searchsorted(s, SIGNAL_DOWNTRACK_M))
    i = min(i, len(center) - 1)
    p, yaw = center[i], float(track.headings()[i])
    right = np.array([math.sin(yaw), -math.cos(yaw)])          # unit vector to the right
    half = float(track.half_width[i])
    edge = p + right * half                                    # the right bale line
    board_center = edge + right * (SIGNAL["board_w"] / 2.0)    # board stands just outside it
    return {"x": round(float(board_center[0]), 3), "y": round(float(board_center[1]), 3),
            "facing_yaw": round(yaw + math.pi, 4),             # faces back toward the car
            "edge_x": round(float(edge[0]), 3), "edge_y": round(float(edge[1]), 3),
            **{k: v for k, v in SIGNAL.items() if k != "colors"},
            "board_rgb": SIGNAL["colors"]["board"], "red_rgb": SIGNAL["colors"]["red"],
            "green_rgb": SIGNAL["colors"]["green"]}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--out", type=Path, default=Path(__file__).with_name("course3d.json"))
    args = ap.parse_args()
    rng = np.random.default_rng(args.seed)
    track = tracks.load_obstacle_course(rng, Path(tempfile.mkdtemp()), "course3d")
    # Training picks a random driving direction; keep the drawing's (centerline.csv order).
    drawn = np.loadtxt(tracks.OBSTACLE_COURSE_DIR / "centerline.csv", delimiter=",", skiprows=1)
    if np.linalg.norm(track.center[0] - drawn[0]) > 0.2:
        track.center = track.center[::-1].copy()
        track.half_width = track.half_width[::-1].copy()
    walls, extent = outer_frame(wall_polygons(track))
    start_xy, yaw = track.center[0], float(track.headings()[0])
    buckets, hoops = [], []
    # tracks.py stores buckets as 16-point rings and hoop posts as 4-corner squares.
    for center, corners in track.obstacles:
        corners = np.asarray(corners)
        if len(corners) > 4:
            r = float(np.mean(np.linalg.norm(corners - center, axis=1)))
            buckets.append({"x": round(float(center[0]), 3), "y": round(float(center[1]), 3),
                            "r": round(r, 3), "h": BUCKET_HEIGHT_M})
        else:
            edge = corners[1] - corners[0]
            hoops.append({"x": round(float(center[0]), 3), "y": round(float(center[1]), 3),
                          "size": round(float(np.linalg.norm(edge)), 3),
                          "yaw": round(math.atan2(edge[1], edge[0]), 4), "h": 0.60})
    h, w = track.free.shape
    scene = {
        "name": "LAKSA Obstacle Course 2026", "units": "m", "seed": args.seed,
        "floor": {"x0": track.origin[0], "y0": track.origin[1],
                  "x1": track.origin[0] + w * track.resolution, "y1": track.origin[1] + h * track.resolution},
        # Flat x/y lists per polygon: Unity's JsonUtility cannot read nested arrays.
        "bale_height": BALE_HEIGHT_M,
        "walls": [{"xs": [p[0] for p in w], "ys": [p[1] for p in w]} for w in walls],
        "buckets": buckets, "hoops": hoops,
        "start": {"x": round(float(start_xy[0]), 3), "y": round(float(start_xy[1]), 3), "yaw": round(yaw, 4)},
        "start_signal": start_signal(track, yaw, start_xy),
        "centerline_xs": np.round(track.center[::10, 0], 3).tolist(),
        "centerline_ys": np.round(track.center[::10, 1], 3).tolist(),
    }
    args.out.write_text(json.dumps(scene, indent=1), encoding="utf-8")
    print(f"{args.out}: {len(walls)} wall polygons, {len(buckets)} buckets, {len(hoops)} hoop parts, "
          f"start ({scene['start']['x']}, {scene['start']['y']}), signal at "
          f"({scene['start_signal']['x']}, {scene['start_signal']['y']})")


if __name__ == "__main__":
    main()
