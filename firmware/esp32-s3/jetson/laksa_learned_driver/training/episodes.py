"""Episode rollout and parallel workers (no PyTorch import, so workers start fast).

Jobs are plain dicts so they can be sent to ``ProcessPoolExecutor`` workers on
Windows (spawn).  Each worker keeps its own simulator, track cache and loaded
student policies across jobs.
"""

from __future__ import annotations

import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import numpy as np  # noqa: E402

from expert import Expert, smooth_raceline  # noqa: E402
from sim_env import Domain, LaksaSim, Progress  # noqa: E402
from tracks import generate_track, load_map_track  # noqa: E402
import vehicle as V  # noqa: E402
from laksa_learned_driver.policy import LearnedDriverPolicy, OutputContract  # noqa: E402
from laksa_learned_driver.scan_features import ScanContract, normalize  # noqa: E402

REPO = HERE.parents[4]
MAP_ROOT = REPO / "scratch" / "learned_driver_maps"
SCAN = ScanContract()
OUTPUT = OutputContract(V.STEER_LEFT_MAX_RAD, V.STEER_RIGHT_MAX_RAD, 3.0, V.WHEELBASE_M)


def start_state(track, rng: np.random.Generator, randomize: bool):
    index = int(rng.integers(track.center.shape[0])) if randomize else 0
    heading = track.headings()[index]
    lateral = 0.0
    yaw_offset = 0.0
    if randomize:
        room = max(0.0, float(track.half_width[index]) - V.FOOTPRINT_HALF_WIDTH_M - 0.12)
        lateral = rng.uniform(-0.6, 0.6) * room
        yaw_offset = rng.uniform(-0.15, 0.15)
    x = track.center[index, 0] - math.sin(heading) * lateral
    y = track.center[index, 1] + math.cos(heading) * lateral
    return x, y, heading + yaw_offset


def run_episode(sim: LaksaSim, track, expert: Expert, v_cap: float, steps: int, rng: np.random.Generator,
                student: LearnedDriverPolicy | None = None, beta: float = 1.0, dart_sigma: float = 0.0,
                randomize_start: bool = True, record: bool = True, laps: float = 1.0):
    expert.reset()
    obs = sim.reset(*start_state(track, rng, randomize_start))
    progress = Progress(track, obs["rear_xy"])
    features, caps, steer_labels, speed_labels = [], [], [], []
    result = "TIMEOUT"
    step = 0
    peak = 0.0
    for step in range(1, steps + 1):
        expert_steer, expert_speed = expert.command(obs["rear_xy"], obs["yaw"], obs["speed"], v_cap)
        if record:
            features.append(normalize(obs["binned"], SCAN))
            caps.append(v_cap / OUTPUT.speed_cap_norm_mps)
            steer_labels.append(float(OUTPUT.encode_steering(expert_steer)))
            speed_labels.append(min(1.0, expert_speed / v_cap))
        if student is None or rng.random() < beta:
            steer = expert_steer + (rng.normal(0.0, dart_sigma) if dart_sigma > 0.0 else 0.0)
            speed = expert_speed
        else:
            steer, speed = student.act_on_binned(obs["binned"], v_cap)
        obs = sim.step(steer, speed)
        peak = max(peak, obs["speed"])
        distance = progress.update(obs["rear_xy"])
        if obs["collided"]:
            result = "COLLISION"
            break
        if distance >= laps * progress.length:
            result = "COMPLETE"
            break
    data = (np.asarray(features, np.float32).reshape(-1, SCAN.bins), np.asarray(caps, np.float32),
            np.asarray(steer_labels, np.float32), np.asarray(speed_labels, np.float32))
    time_s = step * V.CONTROL_PERIOD_S
    stats = {"result": result, "steps": step, "time_s": time_s, "progress_m": progress.distance,
             "track_m": progress.length, "progress_laps": progress.distance / progress.length,
             "peak_mps": peak}
    if result == "COMPLETE":
        stats["lap_s"] = time_s / laps
    return data, stats


# ---------------------------------------------------------------- workers ---

_STATE: dict = {}


def _init_worker(seed: int) -> None:
    import warnings
    warnings.filterwarnings("ignore")
    _STATE["sim"] = LaksaSim(SCAN, seed=seed + os.getpid())
    _STATE["tracks"] = {}
    _STATE["policies"] = {}


def _track(spec: tuple):
    cache = _STATE["tracks"]
    if spec not in cache:
        out = MAP_ROOT / f"worker_{os.getpid()}"
        if spec[0] == "random":
            _, seed, prefix = spec
            track = generate_track(np.random.default_rng(seed), out, f"{prefix}_{seed}")
        elif spec[0] == "course":
            track = load_course(Path(spec[1]), out)[0]
        else:
            raise ValueError(spec)
        if len(cache) > 24:
            cache.pop(next(iter(cache)))
        cache[spec] = (track, Expert(track, smooth_raceline(track)))
    return cache[spec]


def _policy(path: str | None):
    if path is None:
        return None
    policies = _STATE["policies"]
    if path not in policies:
        policies.clear()
        policies[path] = LearnedDriverPolicy(path)
    return policies[path]


def run_job(job: dict):
    """Execute one episode job inside a worker."""
    track, expert = _track(tuple(job["track"]))
    rng = np.random.default_rng(job["seed"])
    sim = _STATE["sim"]
    domain = Domain(**job["domain"]) if job.get("domain") else Domain()
    sim.load(track, domain)
    steps = job.get("steps") or int(1.3 * job.get("laps", 1.0) * track.center.shape[0] * track.spacing
                                     / (0.3 * job["cap"]) / V.CONTROL_PERIOD_S) + 200
    data, stats = run_episode(sim, track, expert, job["cap"], steps, rng,
                              student=_policy(job.get("student")), beta=job.get("beta", 1.0),
                              dart_sigma=job.get("dart_sigma", 0.0),
                              randomize_start=job.get("randomize_start", True),
                              record=job.get("record", True), laps=job.get("laps", 1.0))
    return (data if job.get("record", True) else None), {**{k: v for k, v in job.items() if k != "student"},
                                                         "track_name": track.name, **stats}


def make_pool(workers: int, seed: int) -> ProcessPoolExecutor:
    return ProcessPoolExecutor(max_workers=workers, initializer=_init_worker, initargs=(seed,))


def default_workers() -> int:
    # Hybrid laptop CPUs (e.g. i9-13900H: 6P+8E, 20 threads) saturate well
    # before one worker per logical thread.
    return max(1, min(12, (os.cpu_count() or 2) - 2))


def _job_cost(job: dict) -> float:
    if job.get("steps"):
        return float(job["steps"])
    return job.get("laps", 1.0) / max(job["cap"], 1e-3) * (2.0 if job["track"][0] == "course" else 1.0)


def run_parallel(pool: ProcessPoolExecutor, jobs: list[dict], label: str, log, every_s: float = 15.0) -> list:
    """Run jobs longest-first, logging progress and an ETA; return results in job order."""
    import time
    from concurrent.futures import as_completed

    order = sorted(range(len(jobs)), key=lambda i: -_job_cost(jobs[i]))
    futures = {pool.submit(run_job, jobs[i]): i for i in order}
    results = [None] * len(jobs)
    total_cost = sum(_job_cost(j) for j in jobs)
    done_cost = 0.0
    started = last = time.time()
    for count, future in enumerate(as_completed(futures), 1):
        index = futures[future]
        results[index] = future.result()
        done_cost += _job_cost(jobs[index])
        now = time.time()
        if now - last >= every_s or count == len(jobs):
            last = now
            fraction = done_cost / total_cost
            eta = (now - started) * (1.0 - fraction) / max(fraction, 1e-6)
            collisions = sum(1 for r in results if r is not None and r[1]["result"] == "COLLISION")
            log(f"{label}: {count}/{len(jobs)} episodes, ~{100 * fraction:.0f}% of work, "
                f"elapsed {now - started:.0f}s, ETA {eta:.0f}s, collisions so far {collisions}")
    return results


def domain_dict(domain: Domain) -> dict:
    return asdict(domain)


def load_course(course_dir: Path, out_dir: Path):
    import csv
    with open(course_dir / "speed_course_centerline.csv", newline="", encoding="utf-8") as handle:
        rows = [(float(r["x_m"]), float(r["y_m"])) for r in csv.DictReader(handle)]
    center = np.array(rows)
    if np.linalg.norm(center[0] - center[-1]) < 0.05:
        center = center[:-1]
    track = load_map_track("speed_course", course_dir / "speed_course_hires.yaml", center, out_dir)
    return track, (track.center[0, 0], track.center[0, 1], track.headings()[0])
