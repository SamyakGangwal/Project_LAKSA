"""Held-out evaluation of a trained policy against the privileged expert.

The competition Speed Course is used for evaluation only (its mission file
says ``training_allowed: false``).  Extract it from the speed-race branch:

    git show origin/competition/speed-race-track:firmware/esp32-s3/jetson/laksa_speed_race/course/canonical/speed_course/<file>

for ``speed_course_hires.png``, ``speed_course_hires.yaml`` and
``speed_course_centerline.csv`` into ``--course-dir`` (default
``scratch/speed_course``).  Episodes run in parallel worker processes.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import numpy as np  # noqa: E402

from episodes import REPO, default_workers, domain_dict, make_pool, run_parallel  # noqa: E402
from sim_env import Domain  # noqa: E402


def log(message: str) -> None:
    print(f"[{time.strftime('%H:%M:%S')}] {message}", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--model", type=Path, default=HERE.parent / "models" / "laksa_tinylidarnet_v2.npz")
    parser.add_argument("--course-dir", type=Path, default=REPO / "scratch" / "speed_course")
    parser.add_argument("--caps", type=float, nargs="+", default=[0.24, 0.5, 1.0, 1.5, 2.0, 3.0])
    parser.add_argument("--random-tracks", type=int, default=10)
    parser.add_argument("--obstacle-tracks", type=int, default=10, help="held-out tracks with box obstacles")
    parser.add_argument("--course-tracks", type=int, default=10, help="held-out 2026-course-style tracks")
    parser.add_argument("--randomized-trials", type=int, default=3)
    parser.add_argument("--laps", type=float, default=2.0)
    parser.add_argument("--workers", type=int, default=default_workers())
    parser.add_argument("--seed", type=int, default=777)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()

    suites = []
    if (args.course_dir / "speed_course_centerline.csv").is_file():
        suites.append(("speed_course", [("course", str(args.course_dir))]))
    else:
        log(f"competition course not found in {args.course_dir}; skipping it")
    suites.append(("random_tracks", [("random", args.seed + 50_000 + i, "heldout") for i in range(args.random_tracks)]))
    parser_course = args.course_tracks
    if parser_course:
        suites.append(("course_style_tracks", [("course_style", args.seed + 70_000 + i, "heldout")
                                               for i in range(parser_course)]))
    if args.obstacle_tracks:
        suites.append(("obstacle_tracks", [("obstacles", args.seed + 60_000 + i, "heldout")
                                           for i in range(args.obstacle_tracks)]))

    jobs = []
    for suite, specs in suites:
        for cap in args.caps:
            for driver in ("expert", "student"):
                for trial in range(1 + args.randomized_trials):
                    job_seed = args.seed + 7919 * trial + int(cap * 1000)
                    domain = {} if trial == 0 else domain_dict(Domain.sample(np.random.default_rng(job_seed)))
                    for spec in specs:
                        jobs.append({"suite": suite, "driver": driver, "track": spec, "cap": cap,
                                     "seed": job_seed, "domain": domain, "randomized": trial > 0,
                                     "student": str(args.model) if driver == "student" else None,
                                     "beta": 0.0 if driver == "student" else 1.0,
                                     "randomize_start": False, "record": False, "laps": args.laps})
    started = time.time()
    log(f"{len(jobs)} episodes on {args.workers} workers")
    with make_pool(args.workers, args.seed) as pool:
        rows = [stats for _, stats in run_parallel(pool, jobs, "evaluation", log)]
    log(f"finished in {time.time() - started:.0f}s")

    for suite, _ in suites:
        for cap in args.caps:
            for driver in ("expert", "student"):
                for label, randomized in (("clean", False), ("randomized", True)):
                    subset = [r for r in rows if r["suite"] == suite and r["cap"] == cap
                              and r["driver"] == driver and r["randomized"] == randomized]
                    if not subset:
                        continue
                    done = [r for r in subset if r["result"] == "COMPLETE"]
                    lap = np.mean([r["lap_s"] for r in done]) if done else float("nan")
                    log(f"{suite:13s} cap {cap:4.2f} {driver:7s} {label:10s}: {len(done):2d}/{len(subset):2d} complete, "
                        f"lap {lap:6.1f} s, progress {np.mean([min(r['progress_laps'], args.laps) for r in subset]) / args.laps:.2f}, "
                        f"collisions {sum(r['result'] == 'COLLISION' for r in subset)}")
    output = args.output or args.model.with_suffix(".evaluation.json")
    output.write_text(json.dumps({"model": str(args.model), "laps": args.laps, "episodes": rows}, indent=2),
                      encoding="utf-8")
    log(f"wrote {output}")


if __name__ == "__main__":
    main()
