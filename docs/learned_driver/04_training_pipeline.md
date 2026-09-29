# Training pipeline

[Index](README.md) · Previous: [Models](03_models.md) · Next: [Runtime safety](05_runtime_safety.md)

All training code is in `firmware/esp32-s3/jetson/laksa_learned_driver/training/`. It runs on a development PC, not on the Jetson.

## Simulator

- **F1TENTH Gym `v1.0.0`**, the maintainers' Gymnasium-based rewrite. The older `main` branch still imports the unmaintained `gym` package.
- `sim_env.py` wraps it with LAKSA's geometry. Gym always ray-casts the LiDAR from the centre of gravity, but the real LiDAR sits 0.153 m further forward. So the wrapper ray-casts its own 1080-beam scan from the true sensor pose after each control step, and checks collisions against the full car footprint.
- Vehicle constants (`vehicle.py`): wheelbase 0.324 m, mass 3.75 kg, steering 0.523 rad left and 0.288 rad right, control period 0.08 s (12.5 Hz, the A2M12's scan rate), physics step 0.01 s.

## Tracks

`tracks.py` generates random closed loops:

- smooth centerlines whose curvature respects the car's turning limit;
- corridor widths drawn from 0.9–1.25 m (45% of tracks, around the 0.91 m competition lane) or 1.25–2.2 m;
- a self-clearance check so the loop never passes too close to itself;
- rasterised at 0.05 m per pixel into the map format F1TENTH Gym expects.

The **competition Speed Course is never used for training**: its `mission.yaml` says `training_allowed: false`. It is used only as a held-out test.

## Expert

`expert.py` is the teacher. It sees the true pose and the track, which the student never does:

1. an **elastic-band raceline** that cuts curvature inside the corridor, keeping clear of the walls by the footprint plus a margin;
2. a **curvature-based speed profile**, capped by the operator speed cap;
3. **pure pursuit** steering along that raceline.

## DART and DAgger

`train.py` runs 5 rounds:

| Round | Who drives | Labels | Why |
|---|---|---|---|
| 0 (DART) | expert, with steering noise (σ = 0.06 rad) | expert | the data already contains recoveries from off-line states |
| 1–4 (DAgger) | student mixed with expert; expert share β = 0.5, 0.25, 0.125, 0.0625 | expert labels every visited state | the student learns to recover from **its own** mistakes |

Each round:
- collects 40 episodes of up to 900 steps (3 laps) across worker processes;
- retrains the network on **all** data so far (12 epochs, Adam, learning rate 1e-3, Smooth-L1 loss on steering plus 0.5 × Smooth-L1 on speed fraction);
- exports the exact NumPy model the Jetson runs;
- scores it on held-out tracks.

Speed caps are sampled from 0.25–3.0 m/s: half log-uniform over the whole range, half uniform over the fast upper half.

## Domain randomisation

Each "randomised" episode (`sim_env.Domain.sample`) draws:

| Parameter | Range | Stands in for |
|---|---|---|
| Tyre friction μ | 0.75–1.1 | different floors |
| LiDAR range noise | 0–3 cm | sensor noise |
| LiDAR dropout | 0–4% of beams | missing returns |
| Steering bias | ±0.02 rad | servo trim error |
| Speed gain | 0.85–1.1 | motor and wheel calibration error |
| Extra latency | 0–1 control step | processing and actuation delay |

## Parallel workers

Episodes are plain dicts sent to a `ProcessPoolExecutor` (`episodes.py`). Each worker keeps its own simulator, track cache and loaded student, and imports no PyTorch, so workers start quickly. Progress and an ETA print every 15 s, and the longest episodes run first so one straggler can't dominate the end. The shipped model used 12 workers.

## Results

Shipped model: `models/laksa_tinylidarnet_v2.npz`, round 4 (the best), 168,602 samples, hold-out loss 0.0185, about 24 min on a laptop CPU.

**Completion rate on held-out procedural tracks, by round (randomised, 3 m/s cap).** Every round completed 100% of clean runs at all caps and of randomised runs at 0.5 and 1.5 m/s.

| Round | Samples | 3.0 m/s randomised |
|---|---|---|
| 0 | 32,469 | 71% |
| 1 | 66,176 | 83% |
| 2 | 101,148 | 96% |
| 3 | 134,293 | 96% |
| 4 | 168,602 | **100%** |

**Final evaluation** (`evaluate.py`, two laps per run):

| Speed cap | Competition course (clean + 3 randomised) | 10 unseen random tracks (40 runs) |
|---|---|---|
| 0.24–2.0 m/s | 4/4 at every cap; lap times within about 1% of the expert | 40/40 at every cap |
| 3.0 m/s | clean lap 37.2 s (expert 39.2 s); **randomised 0/3**, crashes about 10% into the lap | 40/40 |

The 3 m/s randomised failure on the competition course is the one known gap. It is far above what the car does today (0.15–0.24 m/s caps).

## How to retrain

Requirements: Python 3.10+, `numpy scipy opencv-python torch pyyaml`, and F1TENTH Gym `v1.0.0`:

```bash
git clone -b v1.0.0 https://github.com/f1tenth/f1tenth_gym
pip install -e f1tenth_gym --no-deps
pip install gymnasium pygame requests shapely "yamldataclassconfig<2"
```

Then, from `firmware/esp32-s3/jetson/laksa_learned_driver/training`:

```bash
python smoke_test.py
python train.py
python evaluate.py
```

- `smoke_test.py`: the expert drives three random tracks, as a sanity check.
- `train.py`: writes `../models/laksa_tinylidarnet_v2.npz` and its `.report.json`.
- `evaluate.py`: runs the competition course and fresh tracks, expert against student.

Notes:
- The first import takes about 80 s while numba compiles the simulator physics; later runs reuse the cache.
- Generated maps go to `scratch/learned_driver_maps/`. It is local and not committed.
- Before shipping a new model, run `python -m pytest test` in the package. The tests load the model and check its contracts.
