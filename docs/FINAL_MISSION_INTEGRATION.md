# Final Mission Integration

## Architecture

```text
UGV Mission1 path follower
  -> /ugv/mission_event: MISSION2_START_REACHED
  -> mission_manager_node
  -> /uav/exploration_start
  -> uav_exploration_node
  -> /command/pose
  -> offboard_control
  -> PX4 UAV

uav_aruco_detector_node
  -> /perception/uav/marker_detections
  -> uav_exploration_node records marker IDs
  -> precision_landing_node uses marker map coordinates

mission_manager_node
  -> /ugv/rendezvous_start
  -> ugv_rendezvous_node
  -> /command/ugv_cmd_vel
  -> UGV bridge

mission_manager_node
  waits for both:
    /mission/uav_exploration_complete
    /ugv/rendezvous_reached
  -> /mission/precision_landing_start
  -> precision_landing_node
  -> /command/pose
  -> /command/land
```

Mission2 UAV exploration and Mission3 UGV rendezvous start together when
`/ugv/mission_event: MISSION2_START_REACHED` is received. Mission4 precision landing starts only
after both UAV exploration and UGV rendezvous are complete.

## Mission FSM

```text
INIT
READY
MISSION1_RUNNING
MISSION2_TRIGGERED
UAV_TAKEOFF_READY
UAV_TAKEOFF_REQUESTED
UAV_EXPLORATION_READY
UAV_EXPLORATION_RUNNING
MISSION2_UAV_EXPLORATION_AND_UGV_RENDEZVOUS
UAV_EXPLORATION_COMPLETE
UGV_RENDEZVOUS_READY
UGV_RENDEZVOUS_RUNNING
UGV_RENDEZVOUS_COMPLETE
PRECISION_LANDING_READY
PRECISION_LANDING_RUNNING
MISSION_COMPLETE
MISSION_ABORTED
```

Main transitions:

```text
/ugv/mission_event == MISSION2_START_REACHED
  -> /uav/exploration_start true
  -> /ugv/rendezvous_start true

/mission/uav_exploration_complete true
/ugv/rendezvous_reached true
  -> /mission/precision_landing_start true

/status/landing_complete true
  -> /mission/mission_complete true
```

## Mission2 Marker Timeout Policy

Waypoint progression is not blocked by marker detection. The UAV moves to a waypoint, holds, records any marker detections, then continues.

```yaml
continue_on_marker_timeout: true
marker_wait_timeout_sec: 3.0
hold_even_if_marker_detected: true
do_not_block_waypoint_progress_on_marker: true
waypoint_stuck_timeout_sec: 12.0
ignore_yaw_for_waypoint_reached: true
skip_close_waypoints: true
min_waypoint_separation: 4.0
max_same_waypoint_hold_sec: 12.0
max_total_hover_at_marker_sec: 8.0
minimum_unique_markers: 0
complete_on_path_done: true
```

Events:

```text
MARKER_DETECTED:<id>
MARKER_TIMEOUT_CONTINUE:<tag>
WAYPOINT_HOLD_COMPLETE:<tag>
NEXT_WAYPOINT:<index>:<tag>
CLOSE_WAYPOINT_REMOVED:<tag>
WAYPOINT_STUCK_SKIP:<tag>
EXPLORATION_COMPLETE
EXPLORATION_TIMEOUT
```

## Mission2 Path Policy

Use:

```text
src/asp_uav_control/path/uav_path_mission2_senior.csv
src/asp_uav_control/launch/uav_exploration_mission2.launch.py
```

The Mission2 runtime path is based on the senior team waypoints. Marker-generated paths remain
debug/reference only:

```text
src/asp_uav_control/path/uav_path_generated.csv
src/asp_uav_control/path/uav_path_mission2.csv
```

The runtime launch still uses Mission2 origin latch and forced takeoff. Static spawn prefixes and
generated wall/roof scan rows are not used as runtime takeoff origins or Mission2 scan path.

Mission2 origin is a one-shot latch:

```text
/uav/exploration_event: MISSION2_TAKEOFF_ORIGIN_PUBLISHED_ONCE
/uav/exploration_event: FIRST_TAKEOFF_POSE_CONFIRMED mission2_origin=(...) first_takeoff_pose=(...) xy_error=(...)
/debug/offboard/frame_report: allow_external_origin_reanchor=false
/debug/offboard/frame_report: duplicate_origin_ignored_count=<n>
```

`offboard_control` keeps the first Mission2 map/PX4 local anchor pair until `/mission/reset true`.
Repeated `/uav/mission2_takeoff_origin` messages do not change `mission2_px4_anchor_ned` when
`allow_external_origin_reanchor=false`.

## Mission3 Rendezvous Policy

`ugv_rendezvous_node` waits for:

```text
/ugv/rendezvous_start true
```

`mission_manager_node` publishes this at the same `MISSION2_START_REACHED` event that starts UAV
exploration. Before start, `ugv_rendezvous_node` does not publish `/command/ugv_cmd_vel`. After
start it follows:

```text
src/asp_ugv_control/path/rendezvous.csv
```

Mission1 `ugv_path_follower_node` publishes only a short zero Twist burst after
`MISSION2_START_REACHED`, then releases `/command/ugv_cmd_vel`:

```yaml
zero_publish_after_stop_count: 5
disable_cmd_after_stop: true
```

Rendezvous speed tuning:

```yaml
cruise_speed: 1.1
max_linear_speed: 1.4
lookahead_distance: 3.5
final_tolerance: 1.4
```

Arrival publishes:

```text
/ugv/rendezvous_reached true
/ugv/mission_event: RENDEZVOUS_REACHED
```

## Mission4 Precision Landing Policy

`precision_landing_node` waits for:

```text
/mission/precision_landing_start true
```

`mission_manager_node` publishes this only after both completion flags are true:

```text
/mission/uav_exploration_complete true
/ugv/rendezvous_reached true
```

It approaches above the UGV using TF, then requires fresh UAV marker detections for descent.

Safety rules:

```text
Marker without map coordinate: hold/search only
Fresh marker with map coordinate: align using /command/pose
Marker lost during descent: stop descending and search
Descent step: 1m
Final land: publish /command/land true
Complete: /status/landing_complete true
```

## Execution Order

Terminal 1:

```bash
cd ~/PX4-Autopilot_ASP
make px4_sitl gz_x500_gimbal
```

Terminal 2:

```text
QGroundControl 실행
```

Terminal 3:

```bash
px4humble
source ~/ros2_ws/install/setup.bash
ros2 launch gazebo_env_setup turn_interfaces.launch.py
```

Terminal 4:

```bash
px4humble
source ~/ros2_ws/install/setup.bash
ros2 launch gazebo_env_setup ugv_bridge.launch.py
```

Terminal 5:

```bash
px4humble
source ~/ros2_ws/install/setup.bash
ros2 launch asp_mission_manager full_mission.launch.py
```

Terminal 6:

```bash
px4humble
source ~/ros2_ws/install/setup.bash
ros2 launch asp_ugv_control ugv_path_follower.launch.py
```

## Monitoring

```bash
ros2 topic echo /mission/state
ros2 topic echo /mission/status
ros2 topic echo /ugv/mission_event
ros2 topic echo /uav/exploration_event
ros2 topic echo /ugv/rendezvous_start
ros2 topic echo /mission/uav_exploration_complete
ros2 topic echo /ugv/rendezvous_reached
ros2 topic echo /mission/precision_landing_start
ros2 topic echo /precision_landing/event
ros2 topic echo /status/landing_complete
ros2 topic echo /perception/uav/marker_detections
ros2 topic echo /command/pose
ros2 topic echo /command/ugv_cmd_vel
```

## Diagnostics

```bash
bash tools/diagnostics/uav_pose_source_audit.sh
bash tools/diagnostics/ugv_cmd_source_audit.sh
python3 tools/diagnostics/mission_path_audit.py
```

## Tuning Parameters

Mission2:

```text
takeoff_relative_height
marker_wait_timeout_sec
waypoint_stuck_timeout_sec
min_waypoint_separation
safe_altitude
transition_altitude
max_transition_step
```

Mission3:

```text
max_linear_speed
cruise_speed
lookahead_distance
waypoint_tolerance
final_tolerance
stuck_timeout_sec
```

Mission4:

```text
landing_marker_id
approach_altitude
align_altitude
descend_step_m
final_land_altitude
xy_tolerance_m
marker_timeout_sec
max_lost_marker_time_sec
```

## Known Limitations

* Precision landing depends on stable UAV camera marker detections and a valid map transform for final descent.
* `landing_marker_id` is `-1` by default, so any in-range UAV marker can be used until the final UGV landing marker ID is fixed.
* Mission2 path generation uses marker pose candidates and conservative altitude rules; real obstacle clearance should still be checked in Gazebo/RViz.
* The full mission launch does not start PX4/Gazebo, QGC, `turn_interfaces`, `ugv_bridge`, or Mission1 UGV path follower.
