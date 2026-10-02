# Console and operation

[Index](README.md) · Previous: [Camera perception](06_camera_perception.md) · Next: [Setup and deployment](08_setup_and_deployment.md)

There is no Xbox controller on the bench car. Two operator tools replace it. Both publish `sensor_msgs/Joy` **exactly like the Xbox path**, so `drive_supervisor` keeps every gate. Their sticks are always neutral: they can't steer or throttle. They only say "an operator is present" and press buttons (A engage, B stop, Y rearm).

## LAKSA Console

`laksa_learned_driver/console_node.py` (aiohttp server) and `console_page.html`, on port **8095**.

| Panel | Content |
|---|---|
| Map | RTAB-Map occupancy grid, the car's pose and driven track, live LiDAR points, detections, START and END markers, the planned route (dashed orange; the driven track is solid blue, cleared with **Clear driven track**). Drag to pan; scroll or **+ / −** to zoom |
| STOP / REARM | Always visible: at the top of the side panel on a laptop, pinned to the bottom of the screen on a phone |
| Mode | Three tabs: **Obstacle course**, **Speed course**, **Trial & explore** (see [Drive modes](#drive-modes)). A green dot marks the modes the car's software is currently running |
| Settings | Per-mode speed, steer around obstacles, camera layer, back away when boxed in, look-ahead |
| Race run | ARM, DISARM, START NOW and the race status line (Obstacle and Speed tabs) |
| Explore | HOLD TO RUN, EXPLORE (2 min), SAVE MAP FOR TRAINING, PLAN ROUTE, HOLD TO GO (Trial & explore tab) |
| Live | supervisor mode, autonomy health, emergency stop, driver status, active profile, measured speed, whether the **ESP32 accepts drive commands**, the driver's command (speed and steering), free distance ahead and what blocks it, battery, nearest person |
| Camera | ZED image with detected objects |

Endpoints: `/` (page), `/camera.jpg`, `/ws` (a WebSocket for state and commands). **Every request needs the token.** A wrong token gets 403.

### Drive modes

The console offers three modes. Each tab keeps its own settings in the browser (`localStorage`), so a phone remembers them between visits. The defaults come from `laksa_learned_driver/profiles.py`, and the car clamps everything it receives: 0.2–3.0 m/s (1.0 m/s for Trial & explore), look-ahead 2–12 m.

| Mode | Purpose | Speed | Steer around obstacles | Camera layer | Back away when boxed in | Look-ahead | Car software |
|---|---|---|---|---|---|---|---|
| **Obstacle course** | fast, but must avoid obstacles | 2.0 m/s | on (0.9 m clearance) | on | on | 8 m | RACE |
| **Speed course** | as fast as possible on a clear track | 3.0 m/s (the model's trained maximum) | on (1.2 m clearance) | off | off | 10 m | RACE |
| **Trial & explore** | drive around, build a map, save it for training | 0.6 m/s (slider up to 1.0) | on (0.6 m clearance) | on | on | 4 m | TRIAL |

- **Speed course, camera off:** the LiDAR clearance check still stops the car for real obstacles. At speed, phantom camera obstacles only cost time.
- **Settings take effect** in Trial & explore when you press **APPLY SETTINGS**: they are published live on `/laksa/drive_profile`, and the driver switches without a restart. Obstacle and Speed settings travel with **ARM**.
- **Profile check:** the driver echoes what it actually uses on `/laksa/drive_profile/active`. The Live panel shows it as *Profile*.
- **Telemetry:** the driver publishes `/laksa/driver/telemetry` at 5 Hz (commanded speed and steering, free distance, what blocks it, current cap, inference time). The Live panel shows it as *Command* and *Free ahead*.

**Mode mismatch.** Obstacle and Speed need the car software in **RACE** (no operator). Trial & explore needs **TRIAL** (an operator holds a button). If the selected tab doesn't match what the car is running, the page:
- shows a banner;
- disables that tab's run buttons;
- offers **SWITCH**. Tap it, then tap again within 4 s to confirm.

After SWITCH, the car software restarts in the other mode. That takes about 1–2 minutes. The choice is kept across reboots in `~/.config/laksa/car_mode`. The network watcher performs the restart, so the web page needs no admin rights. STOP and REARM always work, whatever the mode.

### Controls

| Control | Behaviour |
|---|---|
| APPLY SETTINGS | Trial & explore only: sends the tab's settings to the driver now |
| HOLD TO RUN | The page acts as the deadman while held. The first 1.5 s hold A (`engage_hold_sec`), which engages `LIDAR_CRUISE` after the supervisor's 1 s A-hold (3.5 s and 3 s until 1 Oct). Releasing, sliding off, closing the page or losing the connection stops `/joy`, and the supervisor brakes within 0.5 s. |
| EXPLORE | Same as holding, for up to 120 s. The page must stay open and on screen. Tapping again stops it. |
| SAVE MAP FOR TRAINING | Saves the live map and the driven track to `~/laksa_maps/<timestamp>/` on the Jetson; see [Saving a map for training](#saving-a-map-for-training) |
| STOP | Presses B: latches the emergency stop. Also disarms a race run |
| REARM | Presses Y: clears the emergency stop |
| Set START / Set END | Publishes `/laksa/console/start` and `/laksa/console/goal` |
| Clear start/end & route | Removes both markers and the planned route (dashed orange). A route being driven ends (the car brakes); a plan still in progress is discarded |
| PLAN ROUTE | Asks Nav2's planner for a route from the car to END and draws it on the map. **Planning only; nothing moves.** If Nav2 is still starting (about 15 s after the stack starts), the page shows "Nav2 is still starting" and plans as soon as the planner answers, for up to 30 s. Until 1 Oct it failed at once with "Nav2 planner is not running". |
| HOLD TO GO | A second deadman hold that **doesn't** press A. After 0.5 s of heartbeat it asks the supervisor for `NAVIGATING` mode and sends END to Nav2's navigator. Releasing it cancels the goal, and the supervisor brakes. You must release it before starting another route. |

**Re-arm after a dropped connection** (KarSha's `HoldLatch`, `hold_latch.py`). If the heartbeat lapses while HOLD TO RUN or HOLD TO GO is held (Wi-Fi drop, tunnel stall, a throttled browser), the car brakes. The button then has to be **released and pressed again** before it counts. A hold that simply resumes after the gap is ignored, and the Driver line says so. A deliberate release, STOP and REARM work as before.

The heavy view subscriptions (odometry, scan, camera) exist only while a browser is connected, so an idle console costs almost no CPU.

Route status on the page: `WAITING_NAV2`, `PLANNING`, `PLANNED` (with length), `NAVIGATING`, `ARRIVED`, `STOPPED` (with reason) or `FAILED` (with reason). The END pose faces away from the car's current position. Planning needs the car's pose on the map, so RTAB-Map must be running.

### Saving a map for training

**SAVE MAP FOR TRAINING** writes one folder per save, `~/laksa_maps/<YYYYMMDDTHHMMSS>/`, on the Jetson:

| File | Content |
|---|---|
| `map.png`, `map.yaml` | The RTAB-Map occupancy grid, in the standard ROS map_server format (white free, black occupied, grey unknown) |
| `trail.csv` | The car's driven track in the map frame (`x_m,y_m`; the last 600 poses the console kept) |
| `info.json` | Save time, track length, gap between the start and end of the track, and `loop` (true when the gap is at most 1.5 m and the track is at least 8 m) |

The console records the track only while the page is open, and only while the pose is in the `map` frame, so keep the page open while driving. Training uses only **loops**. If the status line says "not a loop yet", drive back near the start and save again. Each save is a new folder; nothing is overwritten.

For the training steps, see [Training on maps the car explored](04_training_pipeline.md#training-on-maps-the-car-explored).

### Network binding and token

- **At home:** the console binds to `127.0.0.1` only. Reach it through an SSH tunnel:
  ```bash
  ssh -L 8095:127.0.0.1:8095 samyak@<jetson-address>
  ```
  Then open `http://localhost:8095/?token=<token>`. The Tailscale or home Wi-Fi address (`http://100.78.235.3:8095/...`) is **refused on purpose**; only `localhost` through the tunnel works.
- **At the field:** when the car's own hotspot is up, it binds to `10.42.0.1`, the hotspot address only. It is never exposed on home Wi-Fi or Tailscale.
- **Fixed token:** generated once on the Jetson by the launcher into `~/.config/laksa/console_token` (mode 600). It is never committed and never shared outside the Jetson. The full link is written to `~/laksa_run/console_url.txt`.
- **To revoke:** delete the token file. A new one is generated at the next start.

Anyone who has the link and has joined the hotspot can drive the car, so treat the link like a key.

## Race mode

For competition runs (the **Obstacle course** and **Speed course** tabs), the car drives **without an operator holding anything**. It waits at the start line, starts when the camera sees a **green** signal, and stops at a **red** stop signal (a red light or a stop sign).

The car software must be in **RACE** (`dryrun_bringup.sh race`, or SWITCH on the page; see [Drive modes](#drive-modes)). **Race mode and operator-free driving are not deployed on the car without the owner's explicit approval.**

**Race panel:**

| Control | Behaviour |
|---|---|
| ARM | Sends the selected tab's mode and settings, then waits for the green signal. The race manager clamps the settings again |
| DISARM | back to idle; stops the car if it is running |
| START NOW | manual start without the green signal, for testing |
| Status line | IDLE → ARMED → RUNNING (with time) → FINISHED or ABORTED, plus the share of the image the camera currently sees as green and red |

**Signal detection** (`signals.py`):
- It looks for bright, saturated green or red blobs in the upper 80% of the image.
- A blob must cover at least 0.06% of the image and hold for 4 consecutive frames (about 0.4 s).
- The red band stops short of orange, so orange buckets don't count as a stop signal.
- Stop signals are ignored for the first 3 s of a run.

**What still stops the car in race mode:**
- STOP on the page;
- B on a paired gamepad;
- the hardware e-stop input `/laksa/estop_hw` (a Bool `true` latches the emergency stop; the physical switch still has to be wired to it);
- the clearance governor and obstacle avoidance;
- any stale sensor or VESC fault;
- a blocked path.

**Race-mode settings** (`dryrun_bringup.sh race`):
- supervisor cap 12,500 eRPM (about 3.0 m/s);
- `require_operator:=false`;
- odometry sanity limit 4 m/s;
- clearance look-ahead 8 m at start-up. ARM then sets each mode's own look-ahead.

**Bluetooth controller:** the gamepad driver runs in every mode. Pair it once on the Jetson (`bluetoothctl`, then `scan on`, `pair <MAC>`, `trust <MAC>`, `connect <MAC>`). B = emergency stop, Y = rearm, either stick = take over manually.

## Terminal operator

`laksa_operator` (`operator_cli.py`) works from any SSH terminal, including a phone app such as Termius:

```bash
ros2 run laksa_learned_driver laksa_operator status
ros2 run laksa_learned_driver laksa_operator run --duration 60
ros2 run laksa_learned_driver laksa_operator stop
ros2 run laksa_learned_driver laksa_operator rearm
```

- `status` shows mode, autonomy health and the emergency stop.
- `run` means "operator present". It holds A for 1.5 s (3.5 s until 1 Oct), then keeps the heartbeat going. Enter, Ctrl-C, the end of `--duration`, a closed terminal or a dropped SSH session (SIGHUP) all stop it and **latch the emergency stop**.
- `stop` latches the emergency stop from any terminal.
- `rearm` clears it.

Each terminal first needs the ROS environment:

```bash
source /opt/ros/humble/setup.bash && source ~/zed_ws/install/setup.bash && source ~/laksa_ws/install/setup.bash && export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 && unset RMW_IMPLEMENTATION CYCLONEDDS_URI
```

## Networking at the field

The Jetson has one Wi-Fi radio. The `laksa-network-watch` service (`setup/laksa_network_watch.sh`, runs as root) manages it:

| Situation | What happens |
|---|---|
| No Wi-Fi for 45 s | starts the hotspot `LAKSA-CAR` (`10.42.0.1/24`, WPA2) |
| On the hotspot | rescans every 60 s; if a known Wi-Fi is visible, drops the hotspot so the Jetson rejoins it |
| Console bound to the wrong address after a change | restarts the **whole** `laksa-car` stack, not just the console (see below) |

The hotspot profile has autoconnect priority −10 and home Wi-Fi +10, so home wins at boot. The hotspot's password is set by the owner on the Jetson and isn't recorded in the repo.

**Why the whole stack restarts:** at the field, the network changed under running processes, and the console restarted on the new network. It then couldn't see the supervisor (Mode and Autonomy showed `UNKNOWN`), while the ESP32 path still worked. Restarting everything together fixed it after a power cycle. The live switch with the new behaviour still needs a field re-test.

Other ways in: a USB-C cable to the Jetson (`192.168.55.1`), or Tailscale.

## Field checklist

1. At home, save the fixed link: `http://10.42.0.1:8095/?token=<token>`.
2. Power on the car, pointed at open space, with at least 2–3 m clear. Wait about 2 minutes. Everything starts **braked**.
3. Join Wi-Fi `LAKSA-CAR` on the phone. When it says "no internet", choose to **stay connected**.
4. Open the link. Check: Mode `MANUAL`, Autonomy a reason such as `XBOX_STALE` (not `UNKNOWN`), E-stop clear, battery about 15–16 V.
5. Select **Trial & explore**. The badge at the top should read `car: TRIAL`. Check the speed (0.6 m/s by default) and press **APPLY SETTINGS**.
6. Tap **EXPLORE** or hold **HOLD TO RUN**. After about 1.5 s the mode shows `LIDAR_CRUISE`. The car keeps moving until an obstacle is 0.30 m (1 ft) ahead, then stops and backs up.
7. Stop with STOP, a second EXPLORE tap, or by releasing HOLD. Stay next to the car.
8. To collect a training map, drive a loop back near the start and tap **SAVE MAP FOR TRAINING**.

| Problem | Fix |
|---|---|
| No `LAKSA-CAR` after 3 minutes | power-cycle, or use USB-C at `192.168.55.1` with an SSH tunnel |
| Mode or Autonomy shows `UNKNOWN` | the page isn't hearing the supervisor; power-cycle |
| `EMERGENCY_STOP` | REARM |
| Run buttons greyed out, SWITCH banner shown | the selected tab needs the other car mode: pick the right tab, or SWITCH (1–2 min restart) |
| Driver says "connection lost: release and press again" | lift your finger off HOLD, then press again |
| Autonomy not `READY` after pressing | a sensor isn't ready; wait 30 s. The Autonomy line names which. |
