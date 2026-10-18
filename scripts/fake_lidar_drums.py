#!/usr/bin/env python3
"""Drum obstacles for the simulator, as a LiDAR point cloud, so the REAL
cone_avoidance_1005 stack can run in stanley_dynamics_sim.launch.py.

Publishes, at --rate Hz, /simulation/points (sensor_msgs/PointCloud2 in
base_link: a ground plane at z = --ground-z plus the vehicle-facing surface of
every drum within view), /simulation/obstacles (MarkerArray of the true drums,
for RViz) and relays /simulation/odometry to /ekf/odometry (the cone stack's
command_guard reads that topic and it cannot be remapped from the launch).

Drums are rows across the road, placed along the route from the start of the
map's first avoidance zone:

    --obstacles "10:+1.0,17:-1.0"   two rows: 10 m into the zone centred 1.0 m
                                    LEFT of the route, 17 m in centred 1.0 m RIGHT
    --drums 3 --pitch 0.7           three drums per row, 0.7 m apart (across)

The vehicle pose is the simulator's TF (utm -> base_link). Test domain only.
"""

import argparse
import csv
import math
import sys
from pathlib import Path

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker, MarkerArray

from krowd_stanley_control.avoidance_handoff import read_avoidance_zones


def route_point(points, cum, s, lateral):
    """(x, y, heading) `s` metres along the polyline, offset `lateral` to the left."""

    s = min(max(s, 0.0), cum[-1])
    i = int(np.searchsorted(cum, s, side="right") - 1)
    i = min(max(i, 0), len(points) - 2)
    (x0, y0), (x1, y1) = points[i], points[i + 1]
    length = cum[i + 1] - cum[i]
    t = (s - cum[i]) / length if length > 0 else 0.0
    heading = math.atan2(y1 - y0, x1 - x0)
    return (x0 + t * (x1 - x0) - lateral * math.sin(heading),
            y0 + t * (y1 - y0) + lateral * math.cos(heading), heading)


class FakeLidarDrums:
    def __init__(self, args, ros_args=None):
        self.args = args
        rclpy.init(args=[sys.argv[0]] + (ros_args or []))
        self.node = rclpy.create_node("fake_lidar_drums")
        self.cloud_pub = self.node.create_publisher(PointCloud2, "/simulation/points", qos_profile_sensor_data)
        self.marker_pub = self.node.create_publisher(MarkerArray, "/simulation/obstacles", 1)
        self.odom_pub = self.node.create_publisher(Odometry, "/ekf/odometry", 10)
        self.node.create_subscription(Odometry, "/simulation/odometry", self.odom_pub.publish, qos_profile_sensor_data)
        self.buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buffer, self.node)
        self.drums = None
        if args.map:
            self.load(Path(args.map).expanduser())
        else:
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
            latched = QoSProfile(depth=1)
            latched.reliability = ReliabilityPolicy.RELIABLE
            latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.node.create_subscription(String, "/localization/map_dir", lambda m: self.load(Path(m.data.strip())), latched)
            self.node.get_logger().info("waiting for the map on /localization/map_dir")
        self.node.create_timer(1.0 / args.rate, self.tick)
        # ground grid in base_link, fixed
        xs = np.arange(0.5, 22.0, 0.25)
        ys = np.arange(-4.0, 4.01, 0.25)
        gx, gy = np.meshgrid(xs, ys)
        self.ground = np.column_stack([gx.ravel(), gy.ravel(), np.full(gx.size, args.ground_z)])

    def load(self, map_dir: Path) -> None:
        if self.drums is not None:
            return
        with open(map_dir / "waypoints.csv", newline="") as stream:
            pts = [(float(r["utm_x"]), float(r["utm_y"])) for r in csv.DictReader(stream)]
        cum = np.r_[0.0, np.cumsum([math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:])])]
        zones = read_avoidance_zones(map_dir)
        start_idx = self.args.zone_start_idx if self.args.zone_start_idx is not None else (zones[0][0] if zones else 0)
        zone_s = cum[min(start_idx, len(cum) - 1)]
        drums = []
        for spec in self.args.obstacles.split(","):
            s_off, lateral = (float(v) for v in spec.split(":"))
            x, y, heading = route_point(pts, cum, zone_s + s_off, lateral)
            for k in range(self.args.drums):
                off = (k - (self.args.drums - 1) / 2.0) * self.args.pitch
                drums.append((x - off * math.sin(heading), y + off * math.cos(heading)))
        self.drums = np.array(drums)
        self.node.get_logger().info(
            f"{len(drums)} drums ({self.args.drums} per row, {self.args.pitch} m pitch, "
            f"{self.args.diameter} m x {self.args.height} m) from zone start wp {start_idx} "
            f"(s={zone_s:.0f} m): rows at {self.args.obstacles}"
        )

    def tick(self) -> None:
        if self.drums is None:
            return
        try:
            t = self.buffer.lookup_transform("utm", "base_link", Time()).transform
        except Exception:
            return
        q = t.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        d = self.drums - np.array([t.translation.x, t.translation.y])
        local = np.column_stack([c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]])
        r = self.args.diameter / 2.0
        pts = [self.ground]
        for cx, cy in local:
            dist = math.hypot(cx, cy)
            if not (0.5 < cx < 25.0 and abs(cy) < 6.0):
                continue
            towards = math.atan2(-cy, -cx)             # drum -> sensor
            az = towards + np.linspace(-math.pi / 2 + 0.05, math.pi / 2 - 0.05, max(8, int(60 / max(dist / 5.0, 1.0))))
            zs = self.args.ground_z + np.linspace(0.05, self.args.height - 0.02, max(4, int(16 / max(dist / 5.0, 1.0))))
            ring_x = cx + r * np.cos(az)
            ring_y = cy + r * np.sin(az)
            drum = np.column_stack([np.repeat(ring_x, len(zs)), np.repeat(ring_y, len(zs)), np.tile(zs, len(az))])
            drum += np.random.normal(0.0, 0.01, drum.shape)
            pts.append(drum)
        xyz = np.vstack(pts).astype(np.float32)
        header = Header()
        header.stamp = self.node.get_clock().now().to_msg()
        header.frame_id = "base_link"
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, xyz.tolist()))
        self.publish_markers()

    def publish_markers(self) -> None:
        markers = MarkerArray()
        for i, (x, y) in enumerate(self.drums):
            m = Marker()
            m.header.frame_id = "utm"
            m.header.stamp = self.node.get_clock().now().to_msg()
            m.ns = "drums"
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y = float(x), float(y)
            m.pose.position.z = self.args.ground_z + self.args.height / 2.0 + self.args.drum_z
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = self.args.diameter
            m.scale.z = self.args.height
            m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 0.3, 0.9, 0.9
            markers.markers.append(m)
        self.marker_pub.publish(markers)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--map", default="", help="map dir with waypoints.csv / avoidance_zones.yaml (default: follow /localization/map_dir)")
    parser.add_argument("--zone-start-idx", type=int, default=None, help="waypoint idx the obstacle distances count from (default: first avoidance zone)")
    parser.add_argument("--obstacles", default="10:+1.0,17:-1.0", help="rows as s_offset:lateral (m, lateral + = left)")
    parser.add_argument("--drums", type=int, default=3)
    parser.add_argument("--pitch", type=float, default=0.7, help="drum spacing across the road (m)")
    parser.add_argument("--diameter", type=float, default=0.58)
    parser.add_argument("--height", type=float, default=0.88)
    parser.add_argument("--ground-z", type=float, default=-0.27, help="ground height in base_link (cone config frames.ground_z)")
    parser.add_argument("--drum-z", type=float, default=0.0, help="extra z for the RViz markers (utm z of the road is unknown)")
    parser.add_argument("--rate", type=float, default=10.0)
    args, ros_args = parser.parse_known_args()   # ros2 launch appends --ros-args
    app = FakeLidarDrums(args, ros_args)
    try:
        rclpy.spin(app.node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
