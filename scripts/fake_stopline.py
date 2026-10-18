#!/usr/bin/env python3
"""A stand-in for the lane detector's stop line, for the simulator: publishes
/glane3d/stopline_markers (LINE_STRIP in base_link) for a stop line placed
across the route at one waypoint of a map.

    ROS_DOMAIN_ID=77 ros2 run krowd_stanley_control fake_stopline.py \\
        ~/krowd_maps/20260913_103732_repaired 68

Arguments: <map_dir> <waypoint idx> [min_visible_m=3.0] [max_visible_m=20.0]
The line is published only while it is between min and max metres ahead of
base_link, like a camera that loses it under the bonnet. The vehicle pose is
the simulator's TF (utm -> base_link).

NEVER run this on the vehicle's domain: it publishes on the real topic name.
"""

import csv
import math
import os
import sys
from pathlib import Path

import rclpy
import tf2_ros
from geometry_msgs.msg import Point
from rclpy.time import Time
from visualization_msgs.msg import Marker, MarkerArray


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    map_dir, index = Path(sys.argv[1]).expanduser(), int(sys.argv[2])
    min_visible = float(sys.argv[3]) if len(sys.argv) > 3 else 3.0
    max_visible = float(sys.argv[4]) if len(sys.argv) > 4 else 20.0
    with open(map_dir / "waypoints.csv", newline="") as stream:
        points = [(float(r["utm_x"]), float(r["utm_y"])) for r in csv.DictReader(stream)]
    x0, y0 = points[index]
    xa, ya = points[max(index - 1, 0)]
    xb, yb = points[min(index + 1, len(points) - 1)]
    heading = math.atan2(yb - ya, xb - xa)
    half = 1.5
    ends = [(x0 - half * math.sin(heading) * k, y0 + half * math.cos(heading) * k) for k in (-1.0, 0.0, 1.0)]
    if os.environ.get("ROS_DOMAIN_ID", "0") in ("", "0"):
        print("WARNING: publishing a FAKE stop line on the default ROS domain", file=sys.stderr)

    rclpy.init()
    node = rclpy.create_node("fake_stopline")
    publisher = node.create_publisher(MarkerArray, "/glane3d/stopline_markers", 5)
    buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(buffer, node)

    def tick():
        try:
            t = buffer.lookup_transform("utm", "base_link", Time()).transform
        except Exception:
            return
        q = t.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        local = []
        for x, y in ends:
            dx, dy = x - t.translation.x, y - t.translation.y
            local.append((c * dx + s * dy, -s * dx + c * dy))
        stamp = node.get_clock().now().to_msg()
        clear = Marker()
        clear.header.frame_id = "base_link"
        clear.header.stamp = stamp
        clear.action = Marker.DELETEALL
        markers = [clear]
        distance = local[1][0]
        if min_visible <= distance <= max_visible:
            line = Marker()
            line.header.frame_id = "base_link"
            line.header.stamp = stamp
            line.ns = "stopline"
            line.id = 1
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.pose.orientation.w = 1.0
            line.scale.x = 0.18
            line.color.r = line.color.g = line.color.b = line.color.a = 1.0
            line.points = [Point(x=x, y=y, z=0.0) for x, y in local]
            markers.append(line)
        publisher.publish(MarkerArray(markers=markers))

    node.create_timer(0.1, tick)
    node.get_logger().info(
        f"fake stop line across waypoint {index} of {map_dir.name}, visible {min_visible}-{max_visible} m ahead"
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
