# LAKSA Jetson bringup

## ZED 2i diagnostic and live preview

The ZED 2i is a USB 3.x stereo camera, not a serial device. Run
`jetson/scripts/laksa-zed-diagnostic` to inventory the USB link, JetPack, ZED
SDK, video devices, and ROS 2 packages. From a terminal inside the active
NoMachine desktop, run `jetson/scripts/laksa-zed-preview` to open the official
Stereolabs ZED Explorer live view. The preview intentionally does not start the
LAKSA navigation stack or any actuator process.

This package uses the standard ROS 2 `joy`, Nav2, and SLAM Toolbox packages.
It does not implement a joystick driver or a USB protocol. The ESP32 native
USB CDC interface is owned by the standard micro-ROS Agent.

## Manual command path

```text
Xbox -> game_controller_node -> /joy -> drive_supervisor
     -> /laksa/command (laksa_interfaces/DriveCommand)
     -> /laksa/brake (std_msgs/Bool, dedicated safety channel)
     -> micro-ROS Agent -> ESP32 watchdog/limits -> actuators
```

The left stick Y axis selects forward or reverse after crossing its deadzone;
its magnitude does not scale speed. The dashboard selects a fixed manual VESC
setpoint from 900, 1300, 1500, 2000, or 3000 eRPM. The right stick X axis
commands steering. B (and the console STOP) cancels autonomy and brakes; only
the hardware e-stop latches an emergency stop, and only Y rearms it, always into manual mode. X
returns to manual, and holding A for three seconds enables forward-priority
LiDAR Cruise. Loss of Xbox or ESP32 telemetry also requests active braking.

## Web mission control

`laksa_system.launch.py` serves the dashboard on TCP port 8088. From a device
on the same Wi-Fi network open `http://JETSON_IP:8088`. It displays the live
SLAM map, pose, IMU, VESC telemetry, currents, temperatures, command, and mode.
Drag on the map from a destination toward the desired heading to submit a Nav2
goal. The dashboard changes the supervisor to goal-navigation mode; Xbox X or
the dashboard cancel button returns control to manual. It also exposes the
mission state, LiDAR Cruise status, autonomous speed limit, planned path, Home,
Go Home, Navigate To Point, and a confirmed Reset Map action. LiDAR Cruise and
dashboard goals are mutually exclusive.

Nav2 uses Hybrid-A* with the Dubins motion model and nonnegative MPPI velocity,
so ordinary point and Home paths are forward-only. Reverse exists only in the
explicit, collision-checked BackUp recovery behavior. MPPI varies forward speed
up to the 1500 eRPM navigation ceiling.

LiDAR Cruise is intentionally simpler than frontier exploration. It consumes
the live 360-degree scan directly, centers between visible side walls, and
steers toward forward open space while SLAM keeps mapping in the background.
It travels forward at 1000 eRPM. Reverse cannot be selected as an ordinary
trajectory: it is enabled only after forward rollouts remain blocked for two
seconds and the rear has at least 0.70 m clearance. Recovery is a bounded K-turn
with at most three phases. Each phase ends from measured displacement/heading,
not an assumed instantaneous motor response. This mode is intended for indoor
corridor/room demonstrations, but it does not claim proof of complete coverage
of an arbitrary floor plan.

The planner and controller use a conservative 1.09 m minimum turning radius,
derived from the measured 0.324 m wheelbase and 16.5-degree right road-wheel
limit. The supervisor compensates for the measured asymmetric left/right
steering endpoints. RF2O estimates odometry directly from consecutive filtered
LiDAR scans, so an inferred servo angle is no longer allowed to publish
`laksa_odom -> laksa_base_footprint`. MPPI runs at 10 Hz with 900 sampled
trajectories to leave compute budget for SLAM and LiDAR processing on the Orin
Nano.

LiDAR Cruise requires fresh Xbox, ESP32/VESC telemetry, and LiDAR data. Global
point navigation additionally requires fresh odometry, map, and `map -> base`
transform data. The dashboard exposes failures by name, including decoded VESC
faults rather than an unexplained integer.

Reset Map validates the exact PID, owner, and systemd cgroup of the existing
`laksa-lidar-mapping.service`, then terminates only that unprivileged SLAM
process. Its existing `Restart=always` policy starts a fresh mapping session.
The dashboard also resets wheel odometry, Home, goals, and mission state, then
clears both Nav2 costmaps after the first map of the new session arrives. No
root permission is exposed to ROS or HTTP. A failed reset releases its protected
interlock and returns to MANUAL instead of trapping the system in an error.
Autonomous movement still requires Start LiDAR Cruise, Go Home, or a confirmed
map destination.

## Build in a Jetson overlay

Install the pinned RF2O odometry dependency, copy
`laksa_interfaces` and `laksa_bringup` into the workspace, then build:

```bash
mkdir -p ~/laksa_ws/src
jetson/scripts/install-rf2o-laser-odometry
cp -a extra_ros_packages/laksa_interfaces ~/laksa_ws/src/
cp -a jetson/laksa_bringup ~/laksa_ws/src/
cd ~/laksa_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Start the micro-ROS Agent in one terminal and the controller launch in another:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/laksa_microros -b 115200 -v4
ros2 launch laksa_bringup xbox_drive.launch.py
```

Before permitting motion, verify `/laksa_esp32`, `/laksa/command`, and
`/laksa/vesc/state`. Perform the first nonzero test with the driven wheels
raised and the vendor ROSOrin bringup stopped.
