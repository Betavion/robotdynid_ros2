# Temporary Excitation Trajectory Optimization Plan

This document captures the current design decisions for improving excitation
trajectory generation, validation, execution, and acceleration preprocessing in
`robotdynid_ros2`. It is intentionally temporary and should be replaced by
formal docs once the implementation stabilizes.

## Implementation Status

Implemented in the current package:

- Dynamics-aware trajectory CSV schema with analytic position, velocity, and
  acceleration columns.
- URDF joint-limit based scaling for `safe_multisine`, `friction_sweep`,
  `gravity_sweep`, and the default `composite` profile.
- Deterministic candidate search with limit metrics and optional regressor
  rank/conditioning score.
- `robotdynid-send-trajectory` now sends positions, velocities, and
  accelerations through `FollowJointTrajectory`.
- `robotdynid-validate-excitation` validates schema, timing, joint limits,
  optional Pinocchio torque screening, optional MoveIt `/check_state_validity`,
  and optional fake-hardware dry-run.
- Split motion CSVs can include acceleration columns, and the core loader uses
  them when present.
- `robotdynid-preprocess-dataset` and `robotdynid-identify-codegen` perform
  trajectory-aware acceleration preprocessing with filtered fallback.
- `collect_with_trajectory.launch.py` and `full_pipeline.launch.py` can generate
  configured trajectories before recording, and manifests record the commanded
  trajectory path.

Remaining future work:

- Add a native continuous optimizer after deterministic candidate search.
- Add richer collision reports with contact pairs when MoveIt exposes contact
  detail through the chosen service/backend.
- Replace this temporary plan with versioned user documentation after real
  hardware validation.

## Goals

- Generate excitation trajectories that improve robot dynamics identification
  quality while staying safe to execute.
- Export trajectories with analytic position, velocity, and acceleration.
- Use trajectory-aware preprocessing to compute robust acceleration for
  identification.
- Validate trajectories before execution with joint-limit, dynamics, and
  collision checks.
- Keep the runtime user flow simple: generate, validate, collect, identify, and
  export.

## Current Gaps

- `robotdynid-generate-excitation` currently emits single-frequency position
  samples only.
- `robotdynid-send-trajectory` currently fills only
  `JointTrajectoryPoint.positions`.
- Split motion datasets do not carry acceleration columns; acceleration is
  estimated from measured velocity by numerical differentiation.
- Generated trajectories are not scored against regressor quality.
- There is no preflight collision check or simulation dry-run step.
- URDF joint limits are not yet used to automatically scale excitation
  amplitudes.

## Key Design Decisions

### Trajectory CSV Schema

Use a dynamics-aware trajectory CSV schema:

```text
time_from_start,
joint1_position,joint1_velocity,joint1_acceleration,
joint2_position,joint2_velocity,joint2_acceleration,
...
```

Rules:

- `time_from_start` is strictly increasing and starts at zero.
- Joint order must match `robot.joint_names`.
- Generated velocity and acceleration are analytic derivatives of the generated
  position profile whenever possible.
- The sender must populate `positions`, `velocities`, and `accelerations` in
  `trajectory_msgs/msg/JointTrajectoryPoint`.
- CSV parsing should reject mixed old/new schemas rather than guessing silently.

### Motion Dataset Acceleration

Extend split motion CSVs to optionally include acceleration:

```text
timestamp,
joint1_position,joint1_velocity,joint1_acceleration,
...
```

Identification loading policy:

1. If acceleration columns exist, use them.
2. If acceleration columns are absent, estimate from velocity as the fallback.
3. Record the source in the manifest/report as:
   - `measured_or_fitted`
   - `estimated_from_velocity`
   - `commanded_only`

`commanded_only` acceleration is useful for debugging and controller feedforward
but should not be the default identification source unless the real robot state
cannot provide adequate motion data.

## Excitation Profiles

Provide built-in profiles rather than requiring users to hand-tune arbitrary
signals.

### `safe_multisine`

Default profile for broad dynamics excitation.

- Finite Fourier series per joint.
- Multiple harmonics with joint-dependent phases.
- Analytic `q`, `qd`, and `qdd`.
- Amplitudes automatically scaled from URDF joint limits.
- Intended to excite inertial, gravity, Coriolis, centrifugal, and friction
  terms while staying away from limits.

### `friction_sweep`

Profile for low-speed friction identification.

- Small amplitude and slow velocity crossings.
- Dense samples around zero velocity.
- Designed to improve viscous, Coulomb, and Stribeck parameter estimation.
- Should usually be combined with `safe_multisine`.

### `gravity_sweep`

Profile for quasi-static gravity coverage.

- Slow posture changes.
- Low acceleration.
- Useful when gravity terms are weakly observed in dynamic profiles.

### Composite Trajectory

Default run layout:

```text
move_to_start -> friction_sweep -> safe_multisine -> gravity_sweep -> return_home
```

Transition segments should be excluded from identification by default unless
their motion quality is explicitly validated.

## Automatic URDF-Based Scaling

The generator should parse URDF joint limits and derive conservative defaults:

- Center: midpoint of finite joint limits, or current/home position if provided.
- Position amplitude: limited by joint range minus a configurable margin.
- Velocity limit: scaled by `trajectory.velocity_scale`.
- Acceleration limit: estimated from velocity limits or configured explicitly.
- Effort limit: used for model-based torque screening when available.

Recommended defaults:

```yaml
trajectory:
  profile: safe_multisine
  duration: 60.0
  sample_period: 0.01
  harmonics: 5
  position_margin_ratio: 0.15
  velocity_scale: 0.3
  acceleration_scale: 0.2
  low_speed_ratio: 0.25
```

## Trajectory Quality Scoring

Generated candidates should be scored before execution. Start with deterministic
multi-seed search, then add continuous optimization later if needed.

Metrics:

- Regressor rank.
- Observation matrix condition number.
- Minimum singular value.
- Per-parameter excitation magnitude.
- Low-speed sample ratio for friction.
- Maximum position, velocity, acceleration, and model-estimated torque.
- Joint-limit margins.
- Collision-free sample ratio.
- Tracking feasibility score from controller limits.

Candidate selection:

1. Generate several candidate Fourier coefficient sets.
2. Reject candidates that violate hard constraints.
3. Choose the candidate with best weighted score, primarily condition number and
   rank, then low-speed coverage.
4. Save an `excitation_report.json` with all scores and reasons for rejection.

## Acceleration Preprocessing Strategy

Do not use raw velocity differentiation as the primary method when the
excitation trajectory is known. Use a trajectory-aware pipeline.

### Default Path With Generated Excitation

```text
generated q/qd/qdd
        +
measured q/qd/tau
        ->
time alignment
        ->
fit measured q using generated-profile basis
        ->
analytic derivative of fitted q
        ->
identification q/qd/qdd/tau
```

Steps:

1. Estimate time offset between commanded trajectory and measured motion.
2. Fit measured position using the same profile basis where possible.
3. Compute velocity and acceleration analytically from fitted coefficients.
4. Reject or warn if fit error exceeds thresholds.
5. Drop transition samples and low-quality intervals.

### Fallback Path

Use smoothed numerical differentiation when:

- no generated excitation file is available;
- the profile basis is unknown;
- tracking quality is poor;
- the data came from a bag or manual motion.

Supported filters should include:

- Savitzky-Golay for local polynomial smoothing.
- Smoothing spline for non-periodic or mixed-profile data.
- Optional low-pass filtering with zero-phase offline processing.

The preprocessing report must include:

- acceleration source;
- estimated trajectory time offset;
- optional torque time offset;
- position and velocity fit RMSE;
- discarded time intervals;
- maximum and RMS acceleration;
- warning/failure thresholds.

Recommended config:

```yaml
preprocessing:
  acceleration_source: fitted   # fitted | filtered | measured
  time_alignment:
    enabled: true
    max_offset_sec: 0.2
  fitting:
    method: generated_profile   # generated_profile | smoothing_spline | savgol
    discard_start_sec: 1.0
    discard_end_sec: 1.0
    max_position_rmse: 0.02
  filtering:
    method: savgol
    window_sec: 0.15
    poly_order: 3
```

## Collision And Simulation Preflight

Add a preflight command:

```bash
ros2 run robotdynid_ros2 robotdynid-validate-excitation \
  --config config/sia_example.yaml \
  --trajectory runs/<timestamp>/excitation.csv
```

Validation layers:

1. CSV schema and monotonic time.
2. Joint order and joint count.
3. Position, velocity, and acceleration limits.
4. Start-state distance from current robot state.
5. Model-estimated torque limits when possible.
6. Collision checking.
7. Optional simulation dry-run.

### Collision Backend

Preferred backend: MoveIt2 PlanningScene.

- Use `robot_description` and SRDF/MoveIt config when available.
- Resample trajectory at validation frequency.
- For each waypoint, set the `RobotState`.
- Call PlanningScene collision checks.
- Report failing timestamps, joint values, and colliding link pairs.

Fallback backend:

- URDF joint-limit validation only.
- Optional coarse workspace or bounding-volume checks if configured.
- Must not claim full collision-free validation without a collision backend.

## User-Facing Workflow

Generate and validate:

```bash
ros2 run robotdynid_ros2 robotdynid-generate-excitation \
  --config config/sia_example.yaml \
  --profile safe_multisine \
  --output runs/<timestamp>/excitation.csv \
  --validate
```

Collect while sending:

```bash
ros2 launch robotdynid_ros2 collect_with_trajectory.launch.py \
  config:=config/sia_example.yaml \
  trajectory_csv:=runs/<timestamp>/excitation.csv
```

Preprocess and identify:

```bash
ros2 run robotdynid_ros2 robotdynid-identify-codegen \
  --config config/sia_example.yaml \
  --manifest runs/<timestamp>/manifest.yaml
```

## Suggested Config Additions

Keep the normal YAML concise. Only advanced users should need to override these.

```yaml
trajectory:
  profile: safe_multisine
  duration: 60.0
  sample_period: 0.01
  harmonics: 5
  position_margin_ratio: 0.15
  velocity_scale: 0.3
  acceleration_scale: 0.2
  low_speed_ratio: 0.25
  search_candidates: 32
  random_seed: 42
  validate:
    enabled: true
    collision: true
    moveit_group: sia_arm
    max_torque_scale: 0.6

preprocessing:
  acceleration_source: fitted
  time_alignment:
    enabled: true
    max_offset_sec: 0.2
  fitting:
    method: generated_profile
    discard_start_sec: 1.0
    discard_end_sec: 1.0
    max_position_rmse: 0.02
  filtering:
    method: savgol
    window_sec: 0.15
    poly_order: 3
```

## Implementation Phases

### Phase 1: Schema And Sender

- Add trajectory schema helpers.
- Generate `position`, `velocity`, and `acceleration` columns.
- Update sender to fill `JointTrajectoryPoint` positions, velocities, and
  accelerations.
- Add tests for CSV parsing and trajectory message creation.

### Phase 2: Acceleration-Aware Datasets

- Add optional acceleration columns to split motion CSV.
- Update `robotdynid` split CSV reader to use acceleration columns when present.
- Update recorder schema and manifest metadata.
- Add preprocessing source metadata.

### Phase 3: Multisine And Friction Profiles

- Implement analytic multisine profile.
- Implement friction sweep profile.
- Add URDF joint-limit scaling.
- Add plots for position, velocity, acceleration, and low-speed coverage.

### Phase 4: Quality Scoring

- Build candidate generation and scoring.
- Use `robotdynid` regressor matrix to rank candidates.
- Save `excitation_report.json`.
- Fail fast on poor rank, extreme condition number, or limit violations.

### Phase 5: Trajectory-Aware Preprocessing

- Associate generated excitation CSV with recorded manifest.
- Estimate time offset.
- Fit measured position using generated profile basis.
- Compute fitted velocity and acceleration analytically.
- Fall back to smoothing-based differentiation when needed.

### Phase 6: Collision And Simulation Preflight

- Add `robotdynid-validate-excitation`.
- Implement joint-limit and timing checks first.
- Add MoveIt2 PlanningScene collision backend.
- Add optional simulation dry-run hook.

### Phase 7: Full Pipeline Integration

- Make `full_pipeline.launch.py` optionally run generate -> validate -> collect
  -> preprocess -> identify -> codegen.
- Keep each step callable independently for debugging.
- Document failure modes and recovery actions.

## References

- ROS2 `trajectory_msgs/msg/JointTrajectoryPoint` supports positions,
  velocities, accelerations, effort, and `time_from_start`.
- `ros2_control` `joint_trajectory_controller` supports interpolation behavior
  based on available position, velocity, and acceleration fields.
- Classic robot dynamic identification work uses finite Fourier trajectories and
  observation-matrix conditioning to improve base-parameter estimation.
- MoveIt2 PlanningScene is the preferred ROS2 collision-checking layer for
  trajectory preflight.
