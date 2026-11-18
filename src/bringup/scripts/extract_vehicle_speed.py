#!/usr/bin/env python3
"""Extract /vehicle_state speed_mps samples from a trackdot rosbag2 (mcap) recording.

Usage:
    source ~/trackdot/install/setup.bash   # needed so trackdot_msgs is importable
    python3 extract_vehicle_speed.py <bag_dir_or_mcap_file> [--csv out.csv]

Works on:
  - normal bags (a bag directory containing metadata.yaml) via rosbag2_py.
  - bags with a missing/broken metadata.yaml (e.g. the recorder crashed or was
    killed instead of shutting down cleanly) by falling back to a raw linear
    scan of the .mcap file with the `mcap` library, which doesn't need an
    index to read messages in order.

The same read_via_raw_mcap() approach generalizes to any other topic
(camera /triton_*/images/compressed, /aeva/atlas/point_cloud_compensated,
GNSS, ...) by changing TOPIC and the message type below.
"""
import argparse
import csv
import os
import sys

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

TOPIC = '/vehicle_state'
VehicleState = get_message('trackdot_msgs/msg/VehicleState')


def read_via_rosbag2(bag_path):
    import rosbag2_py
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    samples = []
    t0 = None
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic != TOPIC:
            continue
        if t0 is None:
            t0 = t
        msg = deserialize_message(data, VehicleState)
        samples.append(((t - t0) / 1e9, msg.speed_mps))
    return samples


def read_via_raw_mcap(mcap_file):
    from mcap.stream_reader import StreamReader
    from mcap.records import Channel, Message
    reader = StreamReader(mcap_file, validate_crcs=False)
    channels = {}
    vs_channel_id = None
    samples = []
    t0 = None
    for record in reader.records:
        if isinstance(record, Channel):
            channels[record.id] = record.topic
            if record.topic == TOPIC:
                vs_channel_id = record.id
            continue
        if not isinstance(record, Message):
            continue
        if record.channel_id != vs_channel_id:
            continue
        if t0 is None:
            t0 = record.log_time
        msg = deserialize_message(bytes(record.data), VehicleState)
        samples.append(((record.log_time - t0) / 1e9, msg.speed_mps))
    return samples


def find_mcap_file(bag_path):
    if bag_path.endswith('.mcap'):
        return bag_path
    for name in sorted(os.listdir(bag_path)):
        if name.endswith('.mcap'):
            return os.path.join(bag_path, name)
    raise FileNotFoundError(f"No .mcap file found in {bag_path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag', help='bag directory (preferred) or path to a .mcap file')
    ap.add_argument('--csv', help='optional path to write (time_s,speed_mps) rows to')
    args = ap.parse_args()

    bag_path = args.bag
    has_metadata = os.path.isdir(bag_path) and os.path.exists(os.path.join(bag_path, 'metadata.yaml'))

    if has_metadata:
        print("[info] metadata.yaml found - reading via rosbag2_py")
        samples = read_via_rosbag2(bag_path)
    else:
        mcap_file = find_mcap_file(bag_path)
        print(f"[info] no metadata.yaml - falling back to raw mcap scan of {mcap_file}")
        print("       (this happens when ros2 bag record did not shut down cleanly;")
        print("        run `ros2 bag reindex -s mcap <bag_dir>` once to fix it permanently)")
        samples = read_via_raw_mcap(mcap_file)

    if not samples:
        print(f"No {TOPIC} messages found.")
        sys.exit(1)

    speeds = [s for _, s in samples]
    print(f"{len(samples)} samples, span {samples[-1][0]:.1f}s")
    print(f"min={min(speeds):.3f}  max={max(speeds):.3f}  mean={sum(speeds)/len(speeds):.3f} m/s")

    if args.csv:
        with open(args.csv, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['time_s', 'speed_mps'])
            w.writerows(samples)
        print(f"wrote {args.csv}")


if __name__ == '__main__':
    main()
