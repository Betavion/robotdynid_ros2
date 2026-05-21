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

Before running on real hardware, start the SIA fake RTDE chain and do a dry run:

```bash
ros2 launch sia_moveit_config sia_arm.launch.py \
  hardware_backend:=rtde_fake \
  use_rviz:=false \
  db:=false \
  auto_motion_enable:=true

ros2 run robotdynid_ros2 robotdynid-validate-excitation \
  --config config/sia_example.yaml \
  --trajectory runs/sia_excitation.csv \
  --dry-run
```

If `move_group` is running, enable `trajectory.validate.collision` and set
`trajectory.validate.moveit_group`; validation will query MoveIt's
`/check_state_validity` service. If the service is not available, the report
records that collision was not checked instead of claiming the path is
collision-free.

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

This copies the generated `predict_tau` C++ kernel and writes an
`identified_params.hpp` header with fixed `linear_parameters` and
`stribeck_parameters` arrays.
