# Mission4 Precision Landing

`precision_landing_node` aligns the UAV above the UGV marker and descends in small steps.

Key topics:

```text
Sub:
/mission/precision_landing_start
/perception/uav/marker_detections
/mission/state

Pub:
/command/pose
/command/land
/status/landing_complete
/precision_landing/state
/precision_landing/event
```

Safety policy:

```text
No marker: hold/search
Marker without map coordinate: hold/search
Marker with map coordinate: align
Marker lost during descent: stop descent and return to search
Final altitude reached: publish /command/land true
```

Run:

```bash
ros2 launch asp_precision_landing precision_landing.launch.py
```

In the full mission, this node is launched by:

```bash
ros2 launch asp_mission_manager full_mission.launch.py
```
