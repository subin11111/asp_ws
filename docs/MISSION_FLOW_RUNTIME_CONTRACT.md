# Mission Flow Runtime Contract

## Mission-Level Flow

The full mission uses one ordered runtime sequence, not isolated platform
features:

```text
Mission1:
  UGV carries the UAV through the tree-zone/takeoff corridor.

Mission2:
  UAV starts exploration at the Mission2 start point.
  UAV follows the locked Mission2 runtime waypoints.
  Marker detection records observations, but marker misses must not block flow.

Mission3:
  UGV continues from the Mission2 start point through the obstacle route.
  This starts in parallel with UAV exploration, with only the UGV-local start
  delay applied inside ugv_rendezvous_node.

Mission4:
  Precision landing starts only after:
    UAV exploration is complete.
    UGV has reached the rendezvous point.
```

## Mission State Contract

The important integration pattern is the condition ordering:

```text
UGV reaches Mission2 start
  -> UAV exploration starts immediately
  -> UGV rendezvous receives start and waits only its local delay
  -> UAV exploration and UGV rendezvous proceed together
  -> landing waits for both completion flags
```

Mission4 must not start from only one completion condition.

## Mission1 and Mission2 Start Coordinates

Current Mission1 runtime path:

```text
src/asp_ugv_control/path/mission.csv
  -120.36252764327848,36.03555436034368,1,0.5
  -126.12116513749258,46.650538629829526,1,0.5
  -131.5354213347476,61.829714929681145,2,0.0
```

The `mission_type=2` row is the Mission2 start handoff. Mission2 takeoff origin
is latched from the actual UAV pose when UGV reaches that point. A hard-coded
takeoff prefix must not be inserted before the Mission2 runtime path.

## Mission2 UAV Path

Current Mission2 runtime path:

```text
src/asp_uav_control/path/uav_path_mission2_senior.csv
```

Generated wall/circular paths are useful only for design and diagnostics. They
must not be selected by the full mission runtime unless explicitly overridden by
a debug launch.

## Mission3 UGV Rendezvous Path

Current Mission3 runtime path:

```text
src/asp_ugv_control/path/mission3_rendezvous_senior.csv
```

Configured final rendezvous point:

```text
x=-57.949, y=101.780
```

## Precision Landing Start Condition

Precision landing starts after both conditions are satisfied:

```text
UAV exploration complete
UGV rendezvous complete
```

## Runtime Failure Patterns To Avoid

```text
UAV returns to Mission1/tree-zone after Mission2 starts.
UAV runtime selects generated circular/wall path instead of the Mission2 runtime path.
UGV Mission3 follows the wrong path or wrong rendezvous endpoint.
Mission3 starts after Mission2 completion instead of in parallel with Mission2.
Mission4 starts with only one of the two required completion conditions.
Mission2 marker miss blocks progression forever.
Mission1 UGV path follower keeps publishing zero cmd_vel and starves Mission3.
```
