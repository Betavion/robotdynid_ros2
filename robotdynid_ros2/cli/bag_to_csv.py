"""Convert a rosbag2 JointState topic to robotdynid split CSV files."""

from __future__ import annotations

import argparse
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from sensor_msgs.msg import JointState

from robotdynid_ros2.data.csv_writer import SplitDatasetCsvWriter
from robotdynid_ros2.data.manifest import MANIFEST_NAME, build_collection_manifest, write_manifest
from robotdynid_ros2.data.ros_extractors import ordered_joint_state_sample
from robotdynid_ros2.paths import timestamped_run_dir


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, help="rosbag2 directory.")
    parser.add_argument("--joint-state-topic", default="/joint_states")
    parser.add_argument("--joint-names", required=True, help="Comma-separated joint names in identification order.")
    parser.add_argument("--output-root", default="runs")
    parser.add_argument("--run-name", default="")
    parser.add_argument("--urdf", default="")
    parser.add_argument("--storage-id", default="sqlite3")
    parser.add_argument("--allow-missing-effort", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    joint_names = [part.strip() for part in args.joint_names.split(",") if part.strip()]
    if not joint_names:
        raise ValueError("--joint-names must not be empty.")

    run_dir = timestamped_run_dir(args.output_root, args.run_name or None)
    data_dir = run_dir / "data"
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(Path(args.bag).expanduser()), storage_id=args.storage_id),
        ConverterOptions(input_serialization_format="", output_serialization_format=""),
    )
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    if topic_types.get(args.joint_state_topic) != "sensor_msgs/msg/JointState":
        raise ValueError(f"{args.joint_state_topic} is not a sensor_msgs/msg/JointState topic in {args.bag}.")

    with SplitDatasetCsvWriter(data_dir, len(joint_names)) as writer:
        while reader.has_next():
            topic, raw, timestamp_ns = reader.read_next()
            if topic != args.joint_state_topic:
                continue
            msg = deserialize_message(raw, JointState)
            sample = ordered_joint_state_sample(
                msg,
                joint_names,
                fallback_timestamp=timestamp_ns * 1e-9,
                allow_missing_effort=args.allow_missing_effort,
            )
            writer.append(sample.timestamp, sample.position, sample.velocity, sample.effort)
        manifest = build_collection_manifest(
            run_dir=run_dir,
            data_dir=data_dir,
            motion_csv=writer.motion_path,
            torque_csv=writer.torque_path,
            joint_names=joint_names,
            joint_state_topic=args.joint_state_topic,
            sample_count=writer.sample_count,
            urdf_path=args.urdf,
        )
    write_manifest(run_dir / MANIFEST_NAME, manifest)
    print(run_dir)


if __name__ == "__main__":
    main()
