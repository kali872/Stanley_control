#!/usr/bin/env python3
"""A stand-in for traffic_light_ros2, for testing the red-light stop in the
simulator. Publishes /traffic_light/detections_json at 20 Hz.

    ROS_DOMAIN_ID=77 ros2 run krowd_stanley_control fake_traffic_light.py red
    ROS_DOMAIN_ID=77 ros2 run krowd_stanley_control fake_traffic_light.py red:20 green   # 20 s red, then green
    ... fake_traffic_light.py red:10 left          # red, then the left arrow
    ... fake_traffic_light.py unknown

Phases: red | green (straight go) | left (left-arrow go) | unknown, each
optionally `:seconds`; the last phase lasts until Ctrl-C.

NEVER run this on the vehicle's domain while the real detector or the vehicle
is up: it publishes on the real topic name. Use the test domain (77).
"""

import json
import os
import sys

import rclpy
from std_msgs.msg import String

PHASES = {
    "red": dict(state="red", go_straight=False, go_left=False),
    "green": dict(state="green", go_straight=True, go_left=False),
    "left": dict(state="green", go_straight=False, go_left=True),
    "unknown": dict(state="unknown", go_straight=False, go_left=False),
}
CONFIRM_FRAMES = 5  # the detector confirms a go after 5 fresh GO frames


def main():
    plan = []
    for word in sys.argv[1:] or ["red"]:
        name, _, seconds = word.partition(":")
        if name not in PHASES:
            sys.exit(f"unknown phase '{name}' (red | green | left | unknown, optional :seconds)")
        plan.append((name, float(seconds) if seconds else None))
    if os.environ.get("ROS_DOMAIN_ID", "0") in ("", "0"):
        print("WARNING: publishing a FAKE traffic light on the default ROS domain", file=sys.stderr)
    rclpy.init()
    node = rclpy.create_node("fake_traffic_light")
    publisher = node.create_publisher(String, "/traffic_light/detections_json", 10)
    state = {"index": 0, "since": node.get_clock().now(), "go_run": {"straight": 0, "left": 0}}

    def tick():
        now = node.get_clock().now()
        name, seconds = plan[state["index"]]
        if seconds is not None and state["index"] < len(plan) - 1 and (now - state["since"]).nanoseconds * 1e-9 >= seconds:
            state["index"] += 1
            state["since"] = now
            name, _ = plan[state["index"]]
            node.get_logger().info(f"fake light -> {name}")
        phase = PHASES[name]
        for direction in ("straight", "left"):
            state["go_run"][direction] = state["go_run"][direction] + 1 if phase[f"go_{direction}"] else 0
        stamp = now.to_msg()
        publisher.publish(String(data=json.dumps({
            "stamp": {"sec": stamp.sec, "nanosec": stamp.nanosec},
            "state": phase["state"], "selection": "fake", "held": False,
            "go_straight": phase["go_straight"], "go_left": phase["go_left"],
            "go_straight_confirmed": state["go_run"]["straight"] >= CONFIRM_FRAMES,
            "go_left_confirmed": state["go_run"]["left"] >= CONFIRM_FRAMES,
            "detections": [],
        })))

    node.create_timer(0.05, tick)
    node.get_logger().info(f"fake light: {' -> '.join(n for n, _ in plan)}")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
