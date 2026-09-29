# Console and operation

[Index](README.md) · Previous: [Camera perception](06_camera_perception.md) · Next: [Setup and deployment](08_setup_and_deployment.md)

There is no Xbox controller on the bench car. Two operator tools replace it. Both publish `sensor_msgs/Joy` **exactly like the Xbox path**, so `drive_supervisor` keeps every gate. Their sticks are always neutral: they can't steer or throttle. They only say "an operator is present" and press buttons (A engage, B stop, Y rearm).

## LAKSA Console

`laksa_learned_driver/console_node.py` (aiohttp server) and `console_page.html`, on port **8095**.

| Panel | Content |
|---|---|
| Map | RTAB-Map occupancy grid, the car's pose, live LiDAR points, START and END markers |
| Camera | ZED image with detected objects |
| Status | mode, autonomy health, emergency stop and reason, driver status, battery voltage, nearest person |
| Controls | **HOLD TO RUN**, **EXPLORE (2 min, slow)**, **STOP**, **REARM**, set START, set END, **PLAN ROUTE**, **HOLD TO GO** |

Endpoints: `/` (page), `/camera.jpg`, `/ws` (a WebSocket for state and commands). **Every request needs the token.** A wrong token gets 403.

### Controls

| Control | Behaviour |
|---|---|
| HOLD TO RUN | The page acts as the deadman while held. The first 3.5 s hold A, which engages `LIDAR_CRUISE`. Releasing, sliding off, closing the page or losing the connection stops `/joy`, and the supervisor brakes within 0.5 s. |
| EXPLORE | Same as holding, for up to 120 s. The page must stay open and on screen. Tapping again stops it. |
| STOP | Presses B: latches the emergency stop |
| REARM | Presses Y: clears the emergency stop |
| Set START / Set END | Publishes `/laksa/console/start` and `/laksa/console/goal` |
| PLAN ROUTE | Asks Nav2's planner for a route from the car to END and draws it on the map. **Planning only; nothing moves.** |
| HOLD TO GO | A second deadman hold that **doesn't** press A. After 0.5 s of heartbeat it asks the supervisor for `NAVIGATING` mode and sends END to Nav2's navigator. Releasing it cancels the goal, and the supervisor brakes. You must release it before starting another route. |

The heavy view subscriptions (odometry, scan, camera) exist only while a browser is connected, so an idle console costs almost no CPU.

Route status on the page: `PLANNING`, `PLANNED` (with length), `NAVIGATING`, `ARRIVED`, `STOPPED` (with reason) or `FAILED` (with reason). The END pose faces away from the car's current position. Planning needs the car's pose on the map, so RTAB-Map must be running.

### Network binding and token

- **At home:** the console binds to `127.0.0.1` only. Reach it through an SSH tunnel:
  ```bash
  ssh -L 8095:127.0.0.1:8095 samyak@<jetson-address>
  ```
  Then open `http://localhost:8095/?token=<token>`.
- **At the field:** when the car's own hotspot is up, it binds to `10.42.0.1`, the hotspot address only. It is never exposed on home Wi-Fi or Tailscale.
- **Fixed token:** generated once on the Jetson by the launcher into `~/.config/laksa/console_token` (mode 600). It is never committed and never shared outside the Jetson. The full link is written to `~/laksa_run/console_url.txt`.
- **To revoke:** delete the token file. A new one is generated at the next start.

Anyone who has the link and has joined the hotspot can drive the car, so treat the link like a key.

## Terminal operator

`laksa_operator` (`operator_cli.py`) works from any SSH terminal, including a phone app such as Termius:

```bash
ros2 run laksa_learned_driver laksa_operator status
ros2 run laksa_learned_driver laksa_operator run --duration 60
ros2 run laksa_learned_driver laksa_operator stop
ros2 run laksa_learned_driver laksa_operator rearm
```

- `status` shows mode, autonomy health and the emergency stop.
- `run` means "operator present". It holds A for 3.5 s, then keeps the heartbeat going. Enter, Ctrl-C, the end of `--duration`, a closed terminal or a dropped SSH session (SIGHUP) all stop it and **latch the emergency stop**.
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
5. Tap **EXPLORE** or hold **HOLD TO RUN**. After about 3.5 s the mode shows `LIDAR_CRUISE`.
6. Stop with STOP, a second EXPLORE tap, or by releasing HOLD. Stay next to the car.

| Problem | Fix |
|---|---|
| No `LAKSA-CAR` after 3 minutes | power-cycle, or use USB-C at `192.168.55.1` with an SSH tunnel |
| Mode or Autonomy shows `UNKNOWN` | the page isn't hearing the supervisor; power-cycle |
| `EMERGENCY_STOP` | REARM |
| Autonomy not `READY` after pressing | a sensor isn't ready; wait 30 s. The Autonomy line names which. |
