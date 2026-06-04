# Mission Runtime Guards

## Summary

The current workspace has the intended high-level structure for the full
mission:

```text
MISSION2_START_REACHED
  -> start UAV exploration
  -> start UGV rendezvous

UAV exploration complete + UGV rendezvous reached
  -> start precision landing
```

The remaining runtime risk is source selection and command ownership: UAV must
not return to Mission1/tree-zone coordinates, generated paths must not become
the full mission path, UGV Mission3 should use the configured runtime route, and
Mission1 cmd_vel ownership must be explicitly released.

## Current Matches

| Area | Current status |
| --- | --- |
| Mission2/Mission3 parallel start | `mission_manager_node.py` starts UAV exploration and UGV rendezvous on `MISSION2_START_REACHED`. |
| Mission4 gating | Precision landing waits for both `/mission/uav_exploration_complete` and `/ugv/rendezvous_reached`. |
| Mission2 UAV runtime path | Full mission launch points to `uav_path_mission2_senior.csv`. |
| Marker non-blocking flow | `continue_on_marker_timeout`, `minimum_unique_markers: 0`, and waypoint stuck/hold limits are configured. |
| Mission2 origin latch | UAV origin is latched from TF at `MISSION2_START_REACHED` and published once. |
| Forced takeoff prefix | Removed from runtime. The UAV starts Mission2 path following from the runtime path after Mission2 origin latch. |
| Mission1 cmd_vel release | `ugv_path_follower_node.cpp` stops publishing after a short zero burst when `disable_cmd_after_stop` is true. |

## Guards Fixed By This Update

| Guard | Impact | Fix |
| --- | --- | --- |
| Generated UAV path can be passed as runtime path by mistake | UAV may fly a generated circular/wall scan route instead of the Mission2 runtime path. | Add runtime path guard: default `allow_generated_path_runtime: false`, default `runtime_path_must_contain: senior`. |
| No explicit return-zone guard after path following starts | A bad waypoint or stale file could command UAV back toward Mission1/tree-zone. | Add forbidden return zone guard after `PATH_FOLLOWING_STARTED`. |
| UGV Mission3 path name was generic `rendezvous.csv` | Harder to verify that runtime uses the configured Mission3 route. | Add `mission3_rendezvous_senior.csv` and update runtime launch paths. |
| Mission1 release event was implicit | Logs showed `MISSION2_START_REACHED` but not cmd_vel ownership release. | Publish `MISSION1_CMD_RELEASED` after the zero Twist burst. |
| Diagnostics still audited older generic path names | Audits could pass while runtime launches the wrong path. | Update path audit and flow audit around runtime filenames and guard strings. |
| UGV moved too soon for UAV takeoff timing | UGV Mission3 could start before UAV takeoff stabilized. | Apply `start_delay_sec: 1.5` only inside `ugv_rendezvous_node`. |

## Path Decisions

### UAV Mission2

Runtime path:

```text
src/asp_uav_control/path/uav_path_mission2_senior.csv
```

Debug paths:

```text
src/asp_uav_control/path/uav_path_generated.csv
src/asp_uav_control/path/uav_path_mission2.csv
src/asp_uav_control/path/uav_path_safe.csv
```

Runtime should reject generated/debug filenames unless an explicit debug launch
overrides the guard.

### UGV Mission3

Runtime path:

```text
src/asp_ugv_control/path/mission3_rendezvous_senior.csv
```

Selected endpoint:

```text
x=-57.949, y=101.780
```

## Forbidden Return Zone

The return-zone guard protects the Mission1/tree-zone handoff corridor after UAV
path following has started. It is intentionally not the marker/exploration area,
because Mission2 contains valid waypoints around `x=-74..-62` and `y=74..76`.

Runtime defaults:

```yaml
forbidden_return_zone_enabled: true
forbidden_return_zone_name: "mission1_tree_zone"
forbidden_return_zone_min_x: -140.0
forbidden_return_zone_max_x: -115.0
forbidden_return_zone_min_y: 35.0
forbidden_return_zone_max_y: 68.0
```

Allowed:

```text
Mission2 origin latch and start waiting states before PATH_FOLLOWING_STARTED
```

Blocked after path following starts:

```text
/uav/exploration_event: POSE_BLOCKED_FORBIDDEN_RETURN_ZONE
```

The node then skips the current waypoint so the mission can continue instead of
holding on a bad command forever.

## Required Static Strings

The following strings should remain visible in static audits:

```text
uav_path_mission2_senior.csv
mission3_rendezvous_senior.csv
allow_generated_path_runtime
runtime_path_must_contain
forbidden_return_zone
POSE_BLOCKED_FORBIDDEN_RETURN_ZONE
MISSION1_CMD_RELEASED
MISSION2_START_REACHED
PARALLEL_MISSION2_3_STARTED
RENDEZVOUS_START_DELAY
RENDEZVOUS_REACHED
EXPLORATION_COMPLETE
```

## No-Touch Areas

This update does not edit:

```text
PX4 repository
default.sdf
Gazebo world/model files
bridge_and_tf.launch.py
turn_interfaces.launch.py
ugv_bridge.launch.py
keyboard nodes
existing topic names
```
