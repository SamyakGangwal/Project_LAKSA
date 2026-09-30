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

First shipped model, **v2** (plain corridors; the default is now v5, see [below](#obstacles-and-the-2026-courses-in-progress)): `models/laksa_tinylidarnet_v2.npz`, round 4 (the best), 168,602 samples, hold-out loss 0.0185, about 24 min on a laptop CPU.

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

The 3 m/s randomised failure on the competition course was v2's one known gap. At the time the car ran at 0.15–0.24 m/s caps; the trial default is now 0.6 m/s.

## Obstacles and the 2026 courses (in progress)

Added after the first field test, to teach the network to drive **around** things rather than only along empty corridors. `train.py` now mixes four kinds of track (`--obstacle-fraction`, `--course-fraction`), and every round is evaluated on each kind:

| Kind | What it is |
|---|---|
| `random` | the original procedural corridors |
| `obstacles` | the same, with 3–8 boxes (0.20–0.45 m) placed mostly on the natural driving line, always leaving a gap of at least 0.74 m |
| `course_style` | built from the *2026 Course Layouts* PDF: sections 20", 32", 36", 41.5" and 48" wide, occasional 11 ft open areas with clusters of 2–9 five-gallon buckets (0.30 m) |
| `obstacle_course` | a replica of the PDF's **Obstacle Course** (`training/courses/`), below |

How the expert handles obstacles:
- The racing line is squeezed through the open side of each obstacle.
- The squeeze tapers as a 2.5 m-radius parabola, so the line curves into the gap instead of kinking.
- The line keeps 0.22 m of side clearance to account for the car's front corner swinging out.
- On course-style tracks it slows in narrow sections.

Expert completion is 100% at 0.5 and 1.5 m/s on obstacle and course-style tracks. It is lower at 3 m/s with randomised physics (58%), which is far above the car's speeds.

Student results so far (completion on held-out tracks, clean conditions, 0.5 / 1.5 m/s):

| Model | Training mix | Obstacle tracks | Course-style tracks | Plain tracks |
|---|---|---|---|---|
| v2 (shipped) | plain only | not trained | not trained | 100% / 100% |
| v3 | 60% obstacles, 5 rounds | 83% / 100% | — | 100% / 100% |
| v4 | 40% course-style, 60% obstacles, 8 rounds | 92% / 100% | 83% / 92% | 100% / 100% |

v5 (10 rounds; 25% replica, 35% course-style, 30% obstacles, 10% plain) was trained next, with the Obstacle Course replica in the mix.

**Held-out comparison** (`evaluate.py --models`, identical episodes for every model, caps 0.5 / 1.5 / 2.5 / 3.0 m/s, clean and randomised; results in `scratch/compare_v2_v4_v5.json`):

| Suite | Expert | v2 | v4 | v5 |
|---|---|---|---|---|
| Speed Course | 12/12 | 10/12 | 12/12 | **12/12** |
| Obstacle Course replica | 38/48 | 0/48 | 0/48 | **27/48** |
| Course-style tracks | 58/72 | 25/72 | 42/72 | **63/72** |
| Obstacle tracks | 54/72 | 10/72 | **68/72** | 58/72 |
| Plain tracks | 72/72 | 72/72 | 72/72 | 72/72 |
| **Overall** | | 42% | 70% | **84%** |

**v5 is the default** (`models/laksa_tinylidarnet_v5.npz`) and isn't worse than v2 on any suite. v4 is better on the obstacle tracks. v5 is not yet on the car: the bench Jetson's launcher still passes v2 (see [Known issues](11_known_issues_and_next_steps.md#work-by-karsha-on-the-bench-jetson)).

### Obstacle Course replica

`training/courses/build_obstacle_course_2026.py --pdf "2026 Course Layouts.pdf"` builds `training/courses/obstacle_course_2026/`:
- **Walls** are read from the PDF's **vector barrier blocks** at 17.9 pt per ft; the 48 ft dimension matches the barrier extents exactly. Each block is drawn at 1 cm and downsampled, so the 20" narrow path measures 0.50 m in the map.
- The helical ramp's rails, the straight ramp's rail and the bank's edges are added from the drawing.
- **The route** (75.9 m) follows the drawing's arrows. The bridge over the tunnel becomes a figure-8 crossing in 2-D, and ramps, gravel, bank and potholes are flat.
- **Per episode:** a random direction, 2–9 buckets in the bucket box, and the three hoops (posts 0.55 m apart) on their dashed lines.
- Each layout is checked with an expert lap at 0.5 m/s and redrawn up to 12 times if the expert can't complete it.

**Status: in v5's training mix** (25% of episodes). Hoop 3 is kept within 0.35 m of the natural line. Before that fix, the expert completed only about 5 of 8 layouts. The failures are all at **hoop 3 on the right-hand loop**: with this car's right-steering limit (1.09 m radius), some legal hoop positions there can't be driven. See [Known issues](11_known_issues_and_next_steps.md#model-limits).

## Training on maps the car explored

The `field_map` track kind trains on places the real car has driven. It uses maps saved with **SAVE MAP FOR TRAINING** in the console's Trial & explore mode (see [Saving a map for training](07_console_and_operation.md#saving-a-map-for-training)).

1. **Explore and save.** In Trial & explore, drive a loop with the console open, come back within 1.5 m of the start (at least 8 m driven), and tap SAVE MAP. The Jetson writes `~/laksa_maps/<timestamp>/` with `map.png`, `map.yaml`, `trail.csv` and `info.json`.
2. **Copy the folder** into the training tree, keeping the folder name:
   ```bash
   scp -r samyak@<jetson-address>:laksa_maps/<timestamp> firmware/esp32-s3/jetson/laksa_learned_driver/training/field_maps/
   ```
   Only folders whose `info.json` says `"loop": true` are used; the rest are skipped.
3. **Train** with a share of field-map episodes (default 0.15, ignored when there are no usable maps). Write to a new file so the default model isn't overwritten:
   ```bash
   python train.py --field-fraction 0.2 --output ../models/laksa_tinylidarnet_v6.npz
   ```
4. **Compare** the new model with the default on identical episodes before switching: `python evaluate.py --models v5=../models/laksa_tinylidarnet_v5.npz v6=../models/laksa_tinylidarnet_v6.npz`.

How a saved map becomes a training track (`tracks.load_field_map`):
- The driven trail is thinned to 0.15 m steps, closed and smoothed into a loop. That loop is the route.
- The occupancy grid becomes the walls. **Unknown cells count as walls**, so only mapped space is drivable.
- Each episode picks a random direction. On 70% of episodes it adds 1–4 boxes where the corridor is at least 0.9 m wide.
- The expert's line is limited to the drivable width on each side, and it slows where the space is narrow.

Field maps are real rooms and yards, so they cover what the procedural corridors don't: clutter, irregular walls and open areas. Keep the share modest (0.1–0.25); one or two small maps repeated too often would overfit.

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
- `train.py`: writes `../models/laksa_tinylidarnet_v5.npz` and its `.report.json` by default. **Pass `--output` with a new name** so a retrain doesn't overwrite the shipped default. Mix options: `--obstacle-fraction`, `--course-fraction`, `--replica-fraction`, `--field-fraction`.
- `evaluate.py --models NAME=PATH ...`: compares several models on identical episodes (`--no-expert` skips the expert).
- `evaluate.py`: runs the competition course and fresh tracks, expert against student.

Notes:
- The first import takes about 80 s while numba compiles the simulator physics; later runs reuse the cache.
- Generated maps go to `scratch/learned_driver_maps/`. The maps from the shipped training run are committed there for reference.
- Before shipping a new model, run the package tests from `test/`: `PYTHONPATH=.. python -m unittest test_hold_latch test_learned_driver` (62 tests; pytest works too if installed). The tests load the model and check its contracts.
