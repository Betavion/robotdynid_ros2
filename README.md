# robotdynid_ros2

`robotdynid_ros2` is a ROS 2 package for repeatable robot dynamics identification runs:

- collect joint motion and measured torque data from ROS 2 topics;
- preserve each run with a manifest and timestamped output directory;
- run offline parameter identification through the `robotdynid` core library;
- export C/C++ dynamics kernels for controller integration.

The core symbolic/numeric identification library is kept as a git submodule at
`robotdynid`. This keeps the ROS package focused on ROS integration while
preserving `robotdynid` as a reusable non-ROS Python library.

## Layout

```text
robotdynid_ros2/
  robotdynid_ros2/   Python nodes, CLIs, data tools
  launch/                  ROS 2 launch files
  config/                  Example parameter files
  examples/                Example robot assets
  scripts/                 Installed ROS 2 executables
  robotdynid/              Core library submodule
```

## Setup

From a ROS 2 workspace:

```bash
git submodule update --init --recursive
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select robotdynid_ros2
source install/setup.bash
```

For source-tree development without installation, keep using the project virtual
environment if desired:

```bash
python3 -m venv .venv
. .venv/bin/activate
pip install -e robotdynid
```

## Configuration

The unified config only needs the sections you actually use. Missing fields use
code defaults. For most SIA runs, `config/sia_example.yaml` is enough.

Common sections:

- `robot`: URDF, DOF, joint order.
- `run`: output root and optional run name.
- `recording`: measured/estimated JointState topics and recorder timing.
- `trajectory`: optional FollowJointTrajectory action and excitation generation.
- `identification`: dataset inputs and optimization settings.
- `codegen`: generated-language and namespace settings.
- `export_runtime`: controller-package export paths.
- `bag_to_csv` and `validation`: offline conversion and dataset checks.

## Collect Data

Most runtime behavior is configured through one YAML file. Start from
`config/sia_example.yaml` for the current SIA setup, or
`config/generic_joint_state.yaml` for a blank template.

Record split motion and torque CSV files from the configured topics:

```bash
ros2 launch robotdynid_ros2 collect_dataset.launch.py \
  config:=config/sia_example.yaml
```

Generate and send the configured excitation trajectory while recording:

```bash
ros2 run robotdynid_ros2 robotdynid-generate-excitation \
  --config config/sia_example.yaml \
  --output runs/sia_excitation.csv \
  --validate

ros2 run robotdynid_ros2 robotdynid-validate-excitation \
  --config config/sia_example.yaml \
  --trajectory runs/sia_excitation.csv

ros2 launch robotdynid_ros2 collect_with_trajectory.launch.py \
  config:=config/sia_example.yaml \
  trajectory_csv:=runs/sia_excitation.csv
```

The generated trajectory CSV contains analytic position, velocity, and
acceleration for every joint:

```text
time_from_start,
joint1_position,joint1_velocity,joint1_acceleration,
...
```

The generator scales amplitudes from URDF joint limits and can build the default
`composite` profile:

```text
friction_sweep -> safe_multisine -> gravity_sweep
```

If the full URDF range contains self-collision regions, configure a narrower
excitation workspace under `trajectory.generation`. These limits are in radians,
use the configured joint order, and are intersected with the URDF hard limits:

```yaml
trajectory:
  generation:
    position_lower: [-1.0, -1.1, -0.8, -0.3, -1.0, -0.8]
    position_upper: [ 1.0,  1.1,  0.8,  0.3,  1.0,  0.8]
```

Acceleration limits are read from the optional URDF `limit acceleration`
attribute unless `trajectory.generation.acceleration_limits` explicitly
overrides them. The multisine selector reports velocity, acceleration,
aperiodicity, and combined dynamic utilization. It searches common, per-joint
adaptive, and lightly detuned frequency sets. Detuned candidates use
Schroeder-style phases when useful to reduce crest factor and avoid visually
repeating cycles while keeping analytic position, velocity, and acceleration.
The default composite profile keeps dedicated friction and gravity content:
friction uses smooth multi-level velocity sweeps with positive and negative
low/mid-speed samples, gravity uses quasi-static pose sweeps with zero-velocity
dwell points, and two detuned multisine packets provide inertial excitation.
Use `trajectory.generation.friction_speed_levels` to set the number of friction
velocity levels, and `trajectory.generation.gravity_pose_count` to set the
number of quasi-static gravity poses.

Before running on real hardware, start the SIA fake RTDE chain and run
collision/state validation. This validation samples the trajectory and queries
MoveIt's `/check_state_validity` service; it does not publish `/joint_states`
and does not send a controller goal:

```bash
ros2 launch sia_moveit_config sia_arm.launch.py \
  hardware_backend:=rtde_fake \
  use_rviz:=false \
  db:=false \
  auto_motion_enable:=true

ros2 run robotdynid_ros2 robotdynid-validate-excitation \
  --config config/sia_example.yaml \
  --trajectory runs/sia_excitation.csv \
  --require-collision
```

If `move_group` is running, enable `trajectory.validate.collision` and set
`trajectory.validate.moveit_group`; validation will query MoveIt's
`/check_state_validity` service. If the service is not available, the report
records that collision was not checked instead of claiming the path is
collision-free.

For RViz-only trajectory preview, publish a MoveIt DisplayTrajectory message.
This does not publish `/joint_states`, does not wait for robot state feedback,
and does not send a controller action goal:

```bash
ros2 run robotdynid_ros2 robotdynid-preview-trajectory \
  --config config/sia_example.yaml \
  --trajectory runs/sia_excitation.csv
```

In RViz, use the MoveIt MotionPlanning display subscribed to
`/display_planned_path`.

The `--dry-run` option is an explicit controller-send test: after validation it
sends the trajectory to the configured `FollowJointTrajectory` action. Only use
it when the active action server is known to be a fake/simulation controller.

`follow_joint_trajectory` is not a sampled topic. It is only used by
`robotdynid-send-trajectory` as an action client when you explicitly ask the
tool to send an excitation trajectory.

The recorder writes:

```text
runs/<timestamp>/
  manifest.yaml
  data/motion.csv
  data/torque_measure_data.csv
  data/torque_estimate_data.csv  # only when estimate_joint_state_topic is set
```

When a trajectory is sent through `collect_with_trajectory.launch.py`, the
manifest records `collection.commanded_trajectory_csv`. The identification CLI
then preprocesses split datasets before solving:

- with a known generated trajectory, it estimates timing offset and fits
  measured position to the commanded profile before using the analytic
  trajectory derivatives;
- if the fit is poor or no trajectory is available, it falls back to offline
  Savitzky-Golay smoothing;
- the preprocessed split dataset is written under
  `runs/<timestamp>/identify/preprocess/` and carries explicit acceleration
  columns.

The recorder keeps callback work small: callbacks reorder the configured joints
and enqueue samples, while a timer flushes CSV batches. This matches the useful
parts of older SIA scripts while avoiding fixed output filenames and silent
working-directory coupling.

## Identify And Generate Code

```bash
ros2 run robotdynid_ros2 robotdynid-identify-codegen \
  --config config/sia_example.yaml \
  --manifest runs/<timestamp>/manifest.yaml
```

Outputs are written under `runs/<timestamp>/identify` unless `--output-dir` is
provided. The main artifacts are `identify_result.json`,
`identified_linear_parameters.csv`, `identified_stribeck_parameters.csv`,
`base_metadata.json`, `prediction.png`, and `codegen/<language>/`.

The optimizer supports diagonal measurement covariance weighting, Gaussian
prior/Tikhonov regularization for BIP + joint-dynamics linear parameters, and
robust losses for noisy datasets. The formulas and tuning guidance are recorded
in `robotdynid/docs/identification_optimization_theory.md`.

The complete collection and identification flow can also be launched from one
configured entry point:

```bash
ros2 launch robotdynid_ros2 full_pipeline.launch.py \
  config:=config/sia_example.yaml
```

## Export Runtime Kernel

```bash
ros2 run robotdynid_ros2 robotdynid-export-runtime \
  --config config/sia_example.yaml \
  --run-dir runs/<timestamp>/identify \
  --target-root /path/to/controller_package
```

This copies the generated `predict_tau` C++ kernel into the controller package
and installs `identified_linear_parameters.csv` plus
`identified_stribeck_parameters.csv` under `runtime/robotdynid`. Updating only
the identified parameters requires re-exporting the CSV files and reloading the
controller, not recompiling the controller package.

## GUI

`robotdynid_ros2` also provides a PySide6 desktop GUI for the same configured
workflow. The GUI is intentionally a thin workflow layer over the existing
CLI/launch entries: it edits the YAML config, runs the same commands through
Qt processes, previews CSV/plot artifacts, and scans timestamped `runs/`
directories.

Install the optional GUI packages in the workspace Python environment:

```bash
/home/betavion/work/siaupper_ros2/venv/bin/python -m pip install -r src/robotdynid_ros2/requirements-gui.txt
```

Launch from a sourced workspace:

```bash
ros2 run robotdynid_ros2 robotdynid-gui \
  --config src/robotdynid_ros2/config/sia_example.yaml \
  --workspace /home/betavion/work/siaupper_ros2 \
  --language zh
```

The GUI pages follow the normal workflow: load the robot config, generate and
validate an excitation trajectory, collect a dataset, run identification/codegen,
export the runtime kernel, and browse historical run artifacts.
