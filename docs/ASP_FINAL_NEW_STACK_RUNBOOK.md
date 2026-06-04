# ASP Final New Stack Runbook

## Mission2 UAV Waypoint Progression

Mission2 UAV uses the `asp_final_*` stack and publishes final-stack topics only:

```text
/asp_final/uav/cmd_pose
/asp_final/uav/exploration_state
/asp_final/uav/exploration_event
/asp_final/uav/mission2_takeoff_origin
```

The CSV file contains scan waypoints:

```text
src/asp_final_uav/path/mission2_uav_waypoints.csv
```

The first scan waypoint is far from the Mission2 origin, so the UAV mission node creates a runtime
transition corridor after takeoff. The CSV is not edited for this; transition waypoints are generated
in memory and placed before the original CSV waypoints.

Progression policy:

```text
Mission2 start
  -> forced takeoff climb
  -> takeoff hold complete
  -> runtime transition corridor
  -> original CSV scan waypoints
```

Arrival uses separate XY/Z checks:

```text
waypoint_xy_tolerance_m
waypoint_z_tolerance_m
```

Yaw is not used to block waypoint advancement when `ignore_yaw_for_waypoint_reached` is true. If the
UAV stays on one waypoint too long, `WAYPOINT_STUCK_SKIP` advances to the next waypoint. If XY is close
but Z remains off for `xy_close_timeout_sec`, `WAYPOINT_XY_CLOSE_FORCE_ADVANCE` advances the path.

Marker detection is recording/event input for Mission2. It does not generate dynamic route targets and
does not block path progression.

## Runtime Check

```bash
cd ~/PX4-Autopilot_ASP
make px4_sitl gz_x500_gimbal
```

```bash
px4humble
source ~/ros2_ws/install/setup.bash
ros2 launch asp_final_bringup final_mission.launch.py
```

```bash
px4humble
source ~/ros2_ws/install/setup.bash
ros2 topic pub --once /asp_final/mission/start std_msgs/msg/Bool "{data: true}"
```

Watch:

```bash
ros2 topic echo /asp_final/uav/exploration_event
ros2 topic echo /asp_final/uav/exploration_state
ros2 topic echo /asp_final/uav/cmd_pose
ros2 topic echo /asp_final/px4/status
ros2 run tf2_ros tf2_echo map x500_gimbal_0/base_link
```

Expected event shape:

```text
TAKEOFF_CLIMB_STARTED
TAKEOFF_HOLD_COMPLETE
MISSION2_TRANSITION_CORRIDOR_CREATED
NEXT_WAYPOINT:...:mission2_transition_001
NEXT_WAYPOINT:...:mission2_transition_002
NEXT_WAYPOINT:...:csv_wp_000
WAYPOINT_REACHED or WAYPOINT_XY_CLOSE_FORCE_ADVANCE or WAYPOINT_STUCK_SKIP
NEXT_WAYPOINT:...
```
