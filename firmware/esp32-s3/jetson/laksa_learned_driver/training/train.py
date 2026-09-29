"""Train the LAKSA learned driver by imitation learning (DART + DAgger).

Round 0 records the privileged expert with injected steering noise (DART), so
the data already contains recoveries.  Later rounds let the current student
drive while the expert labels every visited state (DAgger).  Each round
retrains the TinyLidarNet-style CNN, exports the exact NumPy model used on the
Jetson, and scores it on held-out procedural tracks under clean and
randomized conditions.  Episodes run in parallel worker processes.

    python train.py --output ../models/laksa_tinylidarnet_v2.npz
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import numpy as np  # noqa: E402
import torch  # noqa: E402
from torch import nn  # noqa: E402

from episodes import OUTPUT, REPO, SCAN, default_workers, domain_dict, make_pool, run_parallel  # noqa: E402
from model import TinyLidarNet, export_npz  # noqa: E402
from sim_env import Domain  # noqa: E402
import vehicle as V  # noqa: E402


def log(message: str) -> None:
    print(f"[{time.strftime('%H:%M:%S')}] {message}", flush=True)


def sample_cap(rng: np.random.Generator, low: float, high: float) -> float:
    """Half log-uniform over the full range, half uniform over the fast upper half."""
    if rng.random() < 0.5:
        return float(rng.uniform(0.5 * high, high))
    return float(math.exp(rng.uniform(math.log(low), math.log(high))))


def train_model(model: TinyLidarNet, data, epochs: int, rng: np.random.Generator) -> dict:
    x, cap, steer, speed = (torch.from_numpy(a) for a in data)
    n = x.shape[0]
    order = rng.permutation(n)
    holdout = order[: max(1, n // 10)]
    train = order[max(1, n // 10):]
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-3)
    loss_fn = nn.SmoothL1Loss(beta=0.05)

    def loss_on(idx):
        out = model(x[idx], cap[idx])
        return loss_fn(torch.tanh(out[:, 0]), steer[idx]) + 0.5 * loss_fn(torch.sigmoid(out[:, 1]), speed[idx])

    for _ in range(epochs):
        model.train()
        perm = torch.from_numpy(rng.permutation(train))
        for start in range(0, perm.numel(), 512):
            batch = perm[start:start + 512]
            optimizer.zero_grad()
            loss = loss_on(batch)
            loss.backward()
            optimizer.step()
    model.eval()
    with torch.no_grad():
        return {"train_loss": float(loss_on(torch.from_numpy(train[:20000]))),
                "holdout_loss": float(loss_on(torch.from_numpy(holdout)))}


def eval_jobs(track_seeds, caps, seed: int, student: str | None, randomized_trials: int,
              kinds=("random",)) -> list[dict]:
    jobs = []
    for kind in kinds:
        for t_index, track_seed in enumerate(track_seeds):
            for cap in caps:
                for trial in range(1 + randomized_trials):
                    job_seed = seed + 97 * t_index + int(cap * 100) + 7919 * trial
                    domain = {} if trial == 0 else domain_dict(Domain.sample(np.random.default_rng(job_seed)))
                    jobs.append({"track": (kind, track_seed, "eval"), "cap": cap, "seed": job_seed,
                                 "domain": domain, "student": student, "beta": 0.0 if student else 1.0,
                                 "randomize_start": trial > 0, "record": False, "laps": 1.0,
                                 "randomized": trial > 0, "kind": kind})
    return jobs


def summarize(rows, caps) -> dict:
    summary = {}
    kinds = sorted({r.get("kind", "random") for r in rows})
    for kind in kinds:
        for cap in caps:
            for label, randomized in (("clean", False), ("randomized", True)):
                subset = [r for r in rows if r["cap"] == cap and r["randomized"] == randomized
                          and r.get("kind", "random") == kind]
                if not subset:
                    continue
                done = [r for r in subset if r["result"] == "COMPLETE"]
                prefix = "" if kind == "random" else f"{kind}_"
                summary[f"{prefix}{cap}_{label}"] = {
                    "completion_rate": len(done) / len(subset),
                    "mean_lap_s": float(np.mean([r["lap_s"] for r in done])) if done else None,
                    "mean_progress_fraction": float(np.mean([min(1.0, r["progress_laps"]) for r in subset])),
                }
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--output", type=Path, default=HERE.parent / "models" / "laksa_tinylidarnet_v5.npz")
    parser.add_argument("--obstacle-fraction", type=float, default=0.3,
                        help="share of training tracks with box obstacles (0 reproduces v2's data)")
    parser.add_argument("--course-fraction", type=float, default=0.35,
                        help="share of training tracks in the 2026 course style (widths, open areas, buckets)")
    parser.add_argument("--replica-fraction", type=float, default=0.25,
                        help="share of training episodes on the 2026 Obstacle Course replica")
    parser.add_argument("--rounds", type=int, default=5, help="DART round + DAgger rounds")
    parser.add_argument("--episodes", type=int, default=40, help="episodes per round")
    parser.add_argument("--steps", type=int, default=900, help="control steps per training episode")
    parser.add_argument("--epochs", type=int, default=12)
    parser.add_argument("--eval-tracks", type=int, default=12)
    parser.add_argument("--eval-randomized-trials", type=int, default=2)
    parser.add_argument("--cap-min", type=float, default=0.25)
    parser.add_argument("--cap-max", type=float, default=3.0)
    parser.add_argument("--workers", type=int, default=default_workers())
    parser.add_argument("--seed", type=int, default=2026)
    args = parser.parse_args()

    torch.manual_seed(args.seed)
    torch.set_num_threads(max(1, args.workers))
    rng = np.random.default_rng(args.seed)
    run_dir = REPO / "scratch" / "learned_driver_runs" / time.strftime("%Y%m%dT%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=True)
    log(f"run directory {run_dir}; {args.workers} workers")
    eval_seeds = [args.seed + 10_000 + i for i in range(args.eval_tracks)]
    eval_caps = (0.5, 1.5, 3.0)
    eval_kinds = ("random",) + (("obstacles",) if args.obstacle_fraction > 0 else ())         + (("course_style",) if args.course_fraction > 0 else ())         + (("obstacle_course",) if args.replica_fraction > 0 else ())

    with make_pool(args.workers, args.seed) as pool:
        started = time.time()
        expert_rows = [stats for _, stats in run_parallel(
            pool, eval_jobs(eval_seeds, eval_caps, args.seed, None, args.eval_randomized_trials, eval_kinds),
            "expert evaluation", log)]
        expert_eval = summarize(expert_rows, eval_caps)
        log(f"expert ({time.time() - started:.0f}s): {json.dumps(expert_eval)}")

        model = TinyLidarNet(SCAN.bins)
        chunks = []
        student = None
        history = []
        best = None
        betas = [1.0] + [0.5 * 0.5 ** r for r in range(args.rounds - 1)]
        for round_index, beta in enumerate(betas):
            started = time.time()
            jobs = []
            for _ in range(args.episodes):
                job_seed = int(rng.integers(2**31))
                draw = rng.random()
                kind = ("obstacle_course" if draw < args.replica_fraction else
                        "course_style" if draw < args.replica_fraction + args.course_fraction else
                        "obstacles" if draw < args.replica_fraction + args.course_fraction + args.obstacle_fraction
                        else "random")
                jobs.append({"track": (kind, job_seed, "train"),
                             "cap": sample_cap(rng, args.cap_min, args.cap_max), "seed": job_seed + 1,
                             "domain": domain_dict(Domain.sample(rng)), "steps": args.steps,
                             "student": student if round_index else None, "beta": beta,
                             "dart_sigma": 0.06 if round_index == 0 else 0.0, "laps": 3.0})
            outcomes = {"COMPLETE": 0, "COLLISION": 0, "TIMEOUT": 0}
            for data, stats in run_parallel(pool, jobs, f"round {round_index} collection", log):
                chunks.append(data)
                outcomes[stats["result"]] += 1
            dataset = tuple(np.concatenate([c[i] for c in chunks]) for i in range(4))
            log(f"round {round_index} (beta={beta:.3f}): {dataset[0].shape[0]} samples, "
                f"outcomes {outcomes}, collection {time.time() - started:.0f}s")

            started = time.time()
            losses = train_model(model, dataset, args.epochs, rng)
            candidate = run_dir / f"round{round_index}.npz"
            summary = {"round": round_index, "samples": int(dataset[0].shape[0]), **losses}
            export_npz(model, candidate, SCAN, OUTPUT, summary)
            log(f"round {round_index}: losses {losses}, training {time.time() - started:.0f}s")

            started = time.time()
            student = str(candidate)
            rows = [stats for _, stats in run_parallel(
                pool, eval_jobs(eval_seeds, eval_caps, args.seed, student, args.eval_randomized_trials, eval_kinds),
                f"round {round_index} evaluation", log)]
            evaluation = summarize(rows, eval_caps)
            log(f"round {round_index} student ({time.time() - started:.0f}s): {json.dumps(evaluation)}")
            score = (np.mean([v["completion_rate"] for v in evaluation.values()]),
                     np.mean([v["mean_progress_fraction"] for v in evaluation.values()]))
            history.append({**summary, "evaluation": evaluation, "path": str(candidate)})
            # Ties go to the later round: it has seen more student-induced states.
            if best is None or score >= best[0]:
                best = (score, candidate, round_index)

    log(f"best round {best[2]} -> {args.output}")
    report = {
        "method": "DART + DAgger imitation of privileged raceline/pure-pursuit expert",
        "architecture": "TinyLidarNet-1D",
        "scan_contract": SCAN.to_dict(),
        "output_contract": OUTPUT.to_dict(),
        "vehicle": {k: getattr(V, k) for k in dir(V) if k.isupper() and not isinstance(getattr(V, k), dict)},
        "args": {k: (str(v) if isinstance(v, Path) else v) for k, v in vars(args).items()},
        "expert_evaluation": expert_eval,
        "rounds": history,
        "best_round": best[2],
        "competition_course_used_for_training": False,
        "obstacle_fraction": args.obstacle_fraction,
        "course_fraction": args.course_fraction,
        "replica_fraction": args.replica_fraction,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes(best[1].read_bytes())
    report_text = json.dumps(report, indent=2)
    args.output.with_suffix(".report.json").write_text(report_text, encoding="utf-8", newline="\n")
    (run_dir / "report.json").write_text(report_text, encoding="utf-8", newline="\n")
    log(f"all round checkpoints kept in {run_dir}")
    log("TRAINING DONE")


if __name__ == "__main__":
    main()
