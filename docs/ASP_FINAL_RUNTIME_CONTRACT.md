# ASP Final Runtime Contract

## Summary

`asp_final_*` packages provide a single launchable full-mission runtime for the
current Gazebo/PX4 setup:

```text
Mission1:
  UGV carries the UAV through the carrier path.

Mission2:
  UAV takes off from the carrier handoff pose and follows the Mission2 waypoint path.

Mission3:
  UGV follows the rendezvous path in parallel with Mission2.

Mission4:
  UAV lands on top of the UGV after both Mission2 and Mission3 are complete.
```

The runtime uses `/asp_final/*` topics and does not require legacy command topics
for the final mission path.

## Launch Contract

Primary launch:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch asp_final_bringup final_mission.launch.py
```

Start trigger:

```bash
ros2 topic pub --once /asp_final/mission/start std_msgs/msg/Bool "{data: true}"
```

The launch starts:

```text
MicroXRCEAgent udp4 -p 8888
asp_final_gazebo_bridge
asp_final_px4_bridge
asp_final_mission
asp_final_ugv
asp_final_uav
asp_final_perception
```

The PX4 DDS link must be alive for UAV operation. Healthy runtime checks:

```bash
ros2 topic info /fmu/out/vehicle_local_position -v
ros2 topic info /fmu/in/trajectory_setpoint -v
ros2 topic echo /asp_final/px4/status
```

Expected link state:

```text
/fmu/out/vehicle_local_position publisher count: 1
/fmu/in/trajectory_setpoint subscription count: 1
/asp_final/px4/status has_px4_anchor: true
```

## Mission State Contract

Mission supervisor states:

```text
IDLE
MISSION1_CARRIER
MISSION2_3_PARALLEL
MISSION4_LANDING
COMPLETE
```

Main transitions:

```text
/asp_final/mission/start true
  -> /asp_final/ugv/mission1_start true

/asp_final/ugv/mission1_complete true
  -> /asp_final/uav/mission2_start true
  -> /asp_final/ugv/rendezvous_start true

/asp_final/uav/mission2_complete true
/asp_final/ugv/rendezvous_reached true
  -> /asp_final/landing/start true

/asp_final/landing/complete true
  -> /asp_final/mission/complete true
```

Mission4 must wait for both Mission2 and Mission3 completion flags.

## UGV Runtime Contract

Mission1 path:

```text
src/asp_final_ugv/path/mission1_carrier.csv
```

Mission3 path:

```text
src/asp_final_ugv/path/mission3_rendezvous.csv
```

Command output:

```text
/asp_final/ugv/cmd_vel
  -> /model/X1_asp/cmd_vel
```

Mission1 uses the carrier profile:

```yaml
mission1_carrier_mode: true
mission1_max_linear_speed: 1.8
mission1_max_angular_speed: 1.8
mission1_max_linear_accel: 1.8
mission1_max_angular_accel: 1.8
mission1_corner_slowdown_enabled: true
```

Mission3 uses the rendezvous profile:

```yaml
mission3_cruise_speed: 2.0
mission3_max_linear_speed: 2.5
mission3_max_angular_speed: 2.5
mission3_lookahead_distance: 3.5
```

## UAV Runtime Contract

Mission2 path:

```text
src/asp_final_uav/path/mission2_uav_waypoints.csv
```

The runtime file contains the full Mission2 scan sequence with roof, wall, and
cruise transition waypoints. The file format is:

```text
x,y,z,yaw_rad,gimbal_pitch_deg,marker_budget
```

Mission2 takeoff origin is latched from the UAV TF at the Mission2 start moment:

```text
/asp_final/uav/mission2_takeoff_origin
```

The PX4 bridge keeps one map/PX4 local anchor pair and converts
`/asp_final/uav/cmd_pose` map-frame poses into PX4 NED setpoints:

```text
/asp_final/uav/cmd_pose
  -> /fmu/in/trajectory_setpoint

/asp_final/uav/land
  -> /fmu/in/vehicle_command NAV_LAND
```

## Mission4 UGV Landing Contract

Mission4 landing target is the UGV, not a static environment marker.

Landing target priority:

```text
1. map -> X1_asp/aruco_marker_10_link
2. map -> X1_asp/base_link
3. hold current UAV XY until a UGV target is available
```

Landing marker detection may refine the UGV target only when all conditions are true:

```text
marker_id == 10
has_map == true
fresh within landing_detection_timeout_s
distance(map marker, UGV target) <= landing_marker_max_ugv_distance_m
```

Detection fields:

```json
{
  "marker_id": 10,
  "camera_x": 0.0,
  "camera_y": 0.0,
  "camera_z": 0.0,
  "map_x": 0.0,
  "map_y": 0.0,
  "map_z": 0.0,
  "has_map": true
}
```

Landing parameters:

```yaml
ugv_base_frame: X1_asp/base_link
ugv_landing_frame: X1_asp/aruco_marker_10_link
landing_marker_id: 10
landing_marker_max_ugv_distance_m: 2.0
landing_detection_timeout_s: 2.0
landing_approach_altitude_m: 8.0
landing_hover_altitude_m: 4.0
landing_xy_tolerance_m: 1.0
landing_descent_step_m: 1.0
landing_complete_altitude_m: 3.0
landing_timeout_s: 30.0
```

Descent guard:

```text
If XY error > landing_xy_tolerance_m:
  hold at or above landing_approach_altitude_m.

If XY error <= landing_xy_tolerance_m:
  descend toward landing_hover_altitude_m, then step down.

Publish /asp_final/uav/land only when:
  altitude <= landing_complete_altitude_m
  and XY error <= landing_xy_tolerance_m
```

Timeout handling:

```text
Timeout does not allow landing away from the UGV.
Timeout can complete landing only while the UAV is already on the UGV target.
```

## Perception Contract

UAV and landing detector topics:

```text
/asp_final/perception/uav/marker_detections
/asp_final/perception/uav/marker_id
/asp_final/perception/landing/marker_detections
/asp_final/perception/landing/marker_id
```

Marker pose outputs contain camera-frame coordinates and map-frame coordinates.
Landing control must use map-frame coordinates only when `has_map` is true.

## Monitoring

Mission:

```bash
ros2 topic echo /asp_final/mission/state
ros2 topic echo /asp_final/mission/status
ros2 topic echo /asp_final/mission/complete
```

UGV:

```bash
ros2 topic echo /asp_final/ugv/state
ros2 topic echo /asp_final/ugv/mission1_complete
ros2 topic echo /asp_final/ugv/rendezvous_reached
ros2 topic echo /asp_final/ugv/cmd_vel
ros2 run tf2_ros tf2_echo map X1_asp/base_link
```

UAV:

```bash
ros2 topic echo /asp_final/uav/exploration_state
ros2 topic echo /asp_final/uav/exploration_event
ros2 topic echo /asp_final/uav/cmd_pose
ros2 topic echo /asp_final/uav/mission2_complete
ros2 run tf2_ros tf2_echo map x500_gimbal_0/base_link
```

Landing:

```bash
ros2 topic echo /asp_final/landing/state
ros2 topic echo /asp_final/landing/event
ros2 topic echo /asp_final/landing/complete
ros2 topic echo /asp_final/perception/landing/marker_detections
ros2 run tf2_ros tf2_echo map X1_asp/aruco_marker_10_link
```

PX4:

```bash
ros2 topic echo /asp_final/px4/status
ros2 topic info /fmu/out/vehicle_local_position -v
ros2 topic info /fmu/in/trajectory_setpoint -v
```

## Diagnostics

```bash
bash tools/asp_final_diagnostics/final_pipeline_audit.sh
bash tools/asp_final_diagnostics/final_tf_audit.sh
python3 -m py_compile \
  src/asp_final_uav/asp_final_uav/uav_mission_node.py \
  src/asp_final_perception/asp_final_perception/aruco_detector.py
colcon build --packages-select asp_final_uav asp_final_perception
```

## Required Static Strings

These strings should remain visible in static audits:

```text
/asp_final/mission/start
/asp_final/uav/mission2_start
/asp_final/ugv/rendezvous_start
/asp_final/landing/start
/asp_final/uav/cmd_pose
/asp_final/uav/land
/asp_final/px4/status
X1_asp/base_link
X1_asp/aruco_marker_10_link
x500_gimbal_0/base_link
landing_marker_id
landing_marker_max_ugv_distance_m
landing_approach_altitude_m
has_map
map_x
map_y
camera_x
camera_y
```

## No-Touch Areas

This runtime contract does not require editing:

```text
PX4 repository
Gazebo world/model files
default.sdf
legacy keyboard nodes
legacy command topic names
```
