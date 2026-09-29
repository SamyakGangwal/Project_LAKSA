"""Smoke test: generate tracks and let the privileged expert drive them.

Prints timing for every stage so slow or stuck steps are visible.
"""

import sys
import time
import traceback
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import numpy as np  # noqa: E402

REPO = HERE.parents[4]
MAPS = REPO / "scratch" / "learned_driver_maps"


def log(message: str) -> None:
    print(f"[{time.strftime('%H:%M:%S')}] {message}", flush=True)


def main() -> None:
    log(f"python {sys.version.split()[0]}; maps -> {MAPS}")
    t0 = time.time()
    from tracks import generate_track
    from expert import Expert, smooth_raceline
    from sim_env import Domain, LaksaSim, Progress
    from laksa_learned_driver.scan_features import ScanContract
    log(f"imports ok ({time.time() - t0:.1f}s)")

    rng = np.random.default_rng(1)
    sim = LaksaSim(ScanContract(), seed=0)
    for index in range(3):
        t0 = time.time()
        track = generate_track(rng, MAPS, f"smoke{index}")
        log(f"track {index}: length {track.center.shape[0] * track.spacing:.1f} m, "
            f"width {2 * track.half_width[0]:.2f} m, generated in {time.time() - t0:.1f}s")
        t0 = time.time()
        expert = Expert(track, smooth_raceline(track))
        log(f"  raceline ready ({time.time() - t0:.1f}s)")
        for cap in (0.5, 1.5, 3.0):
            t0 = time.time()
            sim.load(track, Domain())
            expert.reset()
            heading = track.headings()
            obs = sim.reset(track.center[0, 0], track.center[0, 1], heading[0])
            log(f"  cap {cap}: sim loaded/reset ({time.time() - t0:.1f}s)")
            progress = Progress(track, obs["rear_xy"])
            t0 = time.time()
            steps = 0
            distance = 0.0
            while steps < 3000:
                steering, speed = expert.command(obs["rear_xy"], obs["yaw"], obs["speed"], cap)
                obs = sim.step(steering, speed)
                steps += 1
                distance = progress.update(obs["rear_xy"])
                if steps % 250 == 0:
                    log(f"    step {steps}: progress {distance:.1f}/{progress.length:.1f} m, "
                        f"speed {obs['speed']:.2f} m/s, {steps / (time.time() - t0):.0f} steps/s")
                if obs["collided"] or distance >= progress.length:
                    break
            result = "COLLISION" if obs["collided"] else ("LAP" if distance >= progress.length else "TIMEOUT")
            log(f"  cap {cap}: {result} after {steps} steps ({steps * 0.08:.1f}s sim), "
                f"progress {distance:.1f}/{progress.length:.1f} m, wall {time.time() - t0:.1f}s")
    log("SMOKE TEST DONE")


if __name__ == "__main__":
    try:
        main()
    except Exception:
        traceback.print_exc()
        sys.exit(1)
