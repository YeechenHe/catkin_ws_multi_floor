# Session Memory Anchor

## User Preference

- Always respond in Chinese.
- Experiment must run with Gazebo GUI visible, so elevator entering can be visually verified.

## Current Code State

- Module A integration is enabled via `nav.launch` arg `enable_module_a`.
- `multi_floor_navigation.cpp` current key updates:
  - `check_robot_pose()` uses both linear and angular error.
  - L0/L1 per-floor sigma is loaded from `spa_amcl_region_init.yaml`.
  - Elevator entering logic (`ENTER_LIFT_LEVEL_0`) has been upgraded to closed-loop entry:
    - large heading error -> in-place alignment,
    - moderate error -> forward motion with continuous yaw correction,
    - front obstacle -> hard-stop or creep logic based on distance.
  - Relocation curve logging has been added:
    - auto-create CSV during `INIT_POSE`/`CHECK_INITPOSE`,
    - fields: `elapsed_sec,linear_error,angular_error,tr_sigma,C_t,pose_ok,pass_now`.
- `multi_floor_navigation.h` includes new relocation-curve logging members and methods.
- New script added:
  - `scripts/plot_reloc_recovery_curve.py` for plotting error-time curves from CSV.

## Current Config State

- `spa_amcl_region_init.yaml` currently:
  - L0 sigma = 0.0 / 0.0 / 0.0
  - L1 sigma = 0.2 / 0.2 / 0.12
  - `max_linear_error = 0.2`
  - `max_linear_error_L1 = 0.5` (still considered loose for strict validation)
  - `max_angular_error = 0.15`

## Key Findings Discussed

- Earlier confusion came from mixed historical logs; strict real-time validation is required.
- Robot could report pass in logs while still showing poor elevator-entry behavior visually.
- Main elevator collision risk cause:
  - fixed elevator-entry heading target may not match true door-center line,
  - door-area planning/pose uncertainty,
  - old "rotate then straight" style is brittle in narrow doorway.

## Recent Experiment Outcomes

- There was at least one successful full navigation run (cross-floor closed loop success observed).
- In another run, robot still showed doorway misalignment and failed to enter smoothly.
- Latest logs show closed-loop elevator entry commands are running (`Lift entry closed-loop`).
- During L1 relocation stage, retry behavior was observed in logs (indicates relocation may still need tuning under some runs).

## What User Asked Most Recently

- Repeated full clean restarts and re-runs.
- Root-cause re-analysis for "cannot enter elevator".
- Save full context for cross-session restore with trigger phrase "恢复记忆".

## Recommended Next Strict Steps

1. Run clean single-stack experiment (`enable_module_a:=true`, GUI on).
2. Verify milestones in both logs and visual behavior:
   - approach elevator,
   - enter elevator without wall hit,
   - map switch to L1,
   - final goal reached.
3. Export relocation CSV and draw curves:
   - check `experiment_logs/reloc_curve_*.csv`,
   - plot with `scripts/plot_reloc_recovery_curve.py`.
4. If elevator entry still unstable, next targeted improvement:
   - replace fixed `desired_goal_pose.theta` as elevator-entry reference yaw with captured near-door heading or explicit doorway-axis yaw.
5. For AMCL vs SP-AMCL comparison, keep identical perturbation setting (currently no perturbation by default).

## Runtime Status At Save Time

- All ROS/Gazebo/RViz/navigation processes were killed by request before this save.
- Workspace is in a clean stopped state for next run.

## Restore Instruction

If user says "恢复记忆" in a new window, load this file first and continue from this context:

- `/home/eethanhe/catkin_ws/src/robot-multi-floor-navigation/multi_floor_nav/experiment_logs/SESSION_MEMORY_CURRENT.md`
