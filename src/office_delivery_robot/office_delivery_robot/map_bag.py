"""Build an occupancy grid from a recorded bag using odometry poses.

Equivalent of Lab 3's ``teaching.map_bag`` for this package:

    ros2 run office_delivery_robot map_bag \
        --bag ~/office_delivery_robot_2-main/bags/office_mapping_20261025_182803 \
        --out ~/office_delivery_robot_2-main/results/lab03_odometry

Each /scan is matched to /odom by *header* time (linear interpolation between
the two surrounding odometry samples). A scan with no odometry close enough in
time is skipped and counted in report.json; it never borrows an unrelated pose.
The laser mount offset (URDF laser_joint, x = 0.10 m) is applied rotated by
the body heading.

The mapper trusts the odometry as given, so wheel slip or heading drift will
show up as thick or bent walls. That is exactly what SLAM is meant to fix.
"""

import argparse
import bisect
import math
import os
import sys

import numpy as np

from office_delivery_robot.occupancy_grid import (
    OccupancyGrid, sensor_pose_from_body, write_report)


def stamp_to_sec(stamp):
    """builtin_interfaces/Time -> float seconds."""
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_from_quaternion(q):
    """Yaw (rotation about z) of a geometry_msgs Quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def read_bag(bag_dir, scan_topic='/scan', odom_topic='/odom'):
    """Return (odom, scans) lists read from a rosbag2 directory.

    odom: list of (t, x, y, yaw);  scans: list of (t, LaserScan message).
    Imports are local so the rest of this module works without ROS.
    """
    import yaml
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from rosidl_runtime_py.utilities import get_message

    with open(os.path.join(bag_dir, 'metadata.yaml')) as f:
        meta = yaml.safe_load(f)['rosbag2_bagfile_information']
    storage_id = meta.get('storage_identifier', 'sqlite3')

    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_dir, storage_id=storage_id),
                ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    for topic in (scan_topic, odom_topic):
        if topic not in types:
            raise RuntimeError(f'topic {topic} not found in bag {bag_dir}')
    msg_types = {topic: get_message(types[topic])
                 for topic in (scan_topic, odom_topic)}

    odom, scans = [], []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == odom_topic:
            msg = deserialize_message(data, msg_types[topic])
            p = msg.pose.pose
            odom.append((stamp_to_sec(msg.header.stamp), p.position.x,
                         p.position.y, yaw_from_quaternion(p.orientation)))
        elif topic == scan_topic:
            msg = deserialize_message(data, msg_types[topic])
            scans.append((stamp_to_sec(msg.header.stamp), msg))
    odom.sort(key=lambda s: s[0])
    scans.sort(key=lambda s: s[0])
    return odom, scans


def interpolate_pose(odom, odom_times, t, max_gap):
    """Pose at time t from the two surrounding odometry samples, or None.

    Returns None when t is outside the odometry range or the surrounding
    samples are more than max_gap seconds apart.
    """
    i = bisect.bisect_left(odom_times, t)
    if i == 0 or i >= len(odom):
        if i < len(odom) and abs(odom_times[i] - t) <= 1e-6:
            return odom[i][1:]
        return None
    t0, x0, y0, a0 = odom[i - 1]
    t1, x1, y1, a1 = odom[i]
    if t1 - t0 > max_gap or t1 == t0:
        return None
    w = (t - t0) / (t1 - t0)
    da = math.atan2(math.sin(a1 - a0), math.cos(a1 - a0))   # shortest turn
    return x0 + w * (x1 - x0), y0 + w * (y1 - y0), a0 + w * da


def build_map(odom, scans, resolution=0.05, laser_x=0.10, laser_y=0.0,
              max_gap=0.2, margin=1.0):
    """Integrate all scans that have a usable pose. Returns (grid, report)."""
    odom_times = [s[0] for s in odom]
    used, skipped = [], 0
    for t, scan in scans:
        pose = interpolate_pose(odom, odom_times, t, max_gap)
        if pose is None:
            skipped += 1
            continue
        used.append((pose, scan))

    if not used:
        raise RuntimeError('no scan could be matched to odometry')

    # First pass: map extent from sensor poses and valid return endpoints.
    xs, ys = [], []
    for (xb, yb, yaw), scan in used:
        sx, sy, syaw = sensor_pose_from_body(xb, yb, yaw, laser_x, laser_y)
        xs.append(sx)
        ys.append(sy)
        r = np.asarray(scan.ranges, dtype=float)
        ang = syaw + scan.angle_min + np.arange(r.size) * scan.angle_increment
        ok = np.isfinite(r) & (r >= scan.range_min) & (r < scan.range_max)
        xs.extend(sx + r[ok] * np.cos(ang[ok]))
        ys.extend(sy + r[ok] * np.sin(ang[ok]))
    x_min, x_max = min(xs) - margin, max(xs) + margin
    y_min, y_max = min(ys) - margin, max(ys) + margin

    grid = OccupancyGrid(x_max - x_min, y_max - y_min, resolution,
                         origin_x=math.floor(x_min / resolution) * resolution,
                         origin_y=math.floor(y_min / resolution) * resolution)

    totals = {'hits': 0, 'misses': 0, 'ignored': 0}
    for (xb, yb, yaw), scan in used:
        sx, sy, syaw = sensor_pose_from_body(xb, yb, yaw, laser_x, laser_y)
        stats = grid.integrate_scan(
            sx, sy, syaw, list(scan.ranges), scan.angle_min,
            scan.angle_increment, scan.range_min, scan.range_max)
        for key in totals:
            totals[key] += stats[key]

    report = {
        'scans_in_bag': len(scans),
        'scans_used': len(used),
        'scans_skipped_no_close_odom': skipped,
        'odom_samples': len(odom),
        'laser_offset_m': [laser_x, laser_y],
        'beams': totals,
        'map': grid.summary(),
    }
    return grid, report


def main(argv=None):
    """Command line entry point (ros2 run office_delivery_robot map_bag)."""
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--bag', required=True, help='rosbag2 directory')
    parser.add_argument('--out', required=True, help='output directory')
    parser.add_argument('--stem', default='map', help='output file stem')
    parser.add_argument('--resolution', type=float, default=0.05)
    parser.add_argument('--laser-x', type=float, default=0.10,
                        help='laser offset ahead of body (URDF laser_joint)')
    parser.add_argument('--laser-y', type=float, default=0.0)
    parser.add_argument('--max-gap', type=float, default=0.2,
                        help='max seconds between odom samples to interpolate')
    args, _ = parser.parse_known_args(argv if argv is not None else sys.argv[1:])

    odom, scans = read_bag(os.path.expanduser(args.bag))
    grid, report = build_map(odom, scans, args.resolution, args.laser_x,
                             args.laser_y, args.max_gap)
    out = os.path.expanduser(args.out)
    grid.save(out, args.stem)
    write_report(os.path.join(out, 'report.json'), report)
    print(f'Saved {args.stem}.pgm/.yaml/.png and report.json in {out}')
    print(f"Scans used {report['scans_used']}/{report['scans_in_bag']}, "
          f"skipped {report['scans_skipped_no_close_odom']}")


if __name__ == '__main__':
    main()
