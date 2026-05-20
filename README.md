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

## Collect Data

Record split motion and torque CSV files from a `sensor_msgs/JointState` topic:

```bash
ros2 launch robotdynid_ros2 collect_dataset.launch.py \
  joint_names:="[joint1,joint2,joint3,joint4,joint5,joint6]" \
  joint_state_topic:=/joint_state_broadcaster/joint_states \
  estimate_joint_state_topic:=/joint_states \
  duration_sec:=30.0
```

Generate and send a simple excitation trajectory while recording:

```bash
ros2 run robotdynid_ros2 robotdynid-generate-excitation \
  --joint-names joint1,joint2,joint3,joint4,joint5,joint6 \
  --output /tmp/robotdynid_excitation.csv

ros2 launch robotdynid_ros2 collect_with_trajectory.launch.py \
  joint_names:="[joint1,joint2,joint3,joint4,joint5,joint6]" \
  joint_state_topic:=/joint_state_broadcaster/joint_states \
  estimate_joint_state_topic:=/joint_states \
  trajectory_csv:=/tmp/robotdynid_excitation.csv \
  action_name:=/joint_trajectory_controller/follow_joint_trajectory
```

The recorder writes:

```text
runs/<timestamp>/
  manifest.yaml
  data/motion.csv
  data/torque_measure_data.csv
  data/torque_estimate_data.csv  # only when estimate_joint_state_topic is set
```

The recorder keeps callback work small: callbacks reorder the configured joints
and enqueue samples, while a timer flushes CSV batches. This matches the useful
parts of older SIA scripts while avoiding fixed output filenames and silent
working-directory coupling.

## Identify And Generate Code

```bash
ros2 run robotdynid_ros2 robotdynid-identify-codegen \
  --manifest runs/<timestamp>/manifest.yaml \
  --export-code \
  --codegen-languages c,cpp
```

Outputs are written under `runs/<timestamp>/identify` unless `--output-dir` is
provided.

## Export Runtime Kernel

```bash
ros2 run robotdynid_ros2 robotdynid-export-runtime \
  --run-dir runs/<timestamp>/identify \
  --target-root /path/to/controller_package \
  --namespace robotdynid::generated
```

This copies the generated `predict_tau` C++ kernel and writes an
`identified_params.hpp` header with fixed `theta_lin` and `qds` arrays.
