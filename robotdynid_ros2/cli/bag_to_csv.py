"""Convert a rosbag2 JointState topic to robotdynid split CSV files."""

from __future__ import annotations

import argparse
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from sensor_msgs.msg import JointState

from robotdynid_ros2.config import bag_to_csv_config, read_config
from robotdynid_ros2.data.csv_writer import SplitDatasetCsvWriter
from robotdynid_ros2.data.manifest import MANIFEST_NAME, build_collection_manifest, write_manifest
from robotdynid_ros2.data.ros_extractors import ordered_joint_state_sample
from robotdynid_ros2.paths import timestamped_run_dir


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="", help="Unified robotdynid_ros2 config file.")
    parser.add_argument("--bag", default="", help="rosbag2 directory.")
    parser.add_argument("--joint-state-topic", default="")
    parser.add_argument("--joint-names", default="", help="Comma-separated joint names in identification order.")
    parser.add_argument("--output-root", default="")
    parser.add_argument("--run-name", default="")
    parser.add_argument("--urdf", default="")
    parser.add_argument("--storage-id", default="")
    parser.add_argument("--allow-missing-effort", action="store_true", default=None)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = bag_to_csv_config(read_config(args.config))
    joint_names_raw = args.joint_names or config["joint_names"]
    joint_names = [part.strip() for part in joint_names_raw.split(",") if part.strip()]
    if not joint_names:
        raise ValueError("--joint-names must not be empty.")
    bag = args.bag or config["bag"]
    if not bag:
        raise ValueError("--bag is required unless bag_to_csv.bag is configured.")

    run_dir = timestamped_run_dir(args.output_root or config["output_root"], args.run_name or config["run_name"] or None)
    data_dir = run_dir / "data"
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(Path(bag).expanduser()), storage_id=args.storage_id or config["storage_id"]),
        ConverterOptions(input_serialization_format="", output_serialization_format=""),
    )
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    joint_state_topic = args.joint_state_topic or config["joint_state_topic"]
    if topic_types.get(joint_state_topic) != "sensor_msgs/msg/JointState":
        raise ValueError(f"{joint_state_topic} is not a sensor_msgs/msg/JointState topic in {bag}.")
    allow_missing_effort = bool(config["allow_missing_effort"]) if args.allow_missing_effort is None else args.allow_missing_effort

    with SplitDatasetCsvWriter(data_dir, len(joint_names)) as writer:
        while reader.has_next():
            topic, raw, timestamp_ns = reader.read_next()
            if topic != joint_state_topic:
                continue
            msg = deserialize_message(raw, JointState)
            sample = ordered_joint_state_sample(
                msg,
                joint_names,
                fallback_timestamp=timestamp_ns * 1e-9,
                allow_missing_effort=allow_missing_effort,
            )
            writer.append(sample.timestamp, sample.position, sample.velocity, sample.effort)
        manifest = build_collection_manifest(
            run_dir=run_dir,
            data_dir=data_dir,
            motion_csv=writer.motion_path,
            torque_csv=writer.torque_path,
            joint_names=joint_names,
            joint_state_topic=joint_state_topic,
            sample_count=writer.sample_count,
            urdf_path=args.urdf or config["urdf"],
        )
    write_manifest(run_dir / MANIFEST_NAME, manifest)
    print(run_dir)


if __name__ == "__main__":
    main()
