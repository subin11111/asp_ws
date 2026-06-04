# ASP Final Carrier Mode

Mission1 carries the UAV on top of the UGV, so the UGV must drive with a carrier-safe motion profile. Mission3 uses a separate rendezvous profile because the UAV has already taken off.

## Mission1 Carrier Profile

- Uses `mission1_*` parameters from `src/asp_final_ugv/config/ugv_params.yaml`.
- Limits linear speed to `mission1_max_linear_speed`.
- Limits angular speed to `mission1_max_angular_speed`.
- Applies linear and angular acceleration limits before every `/asp_final/ugv/cmd_vel` publish.
- Slows down on large heading error and corner sections.
- Supports per-waypoint target speed in `mission1_carrier.csv`.

## Mission3 Rendezvous Profile

Mission3 uses `mission3_*` parameters and keeps a faster rendezvous speed. Mission1 carrier limits do not apply after the UAV has taken off.
