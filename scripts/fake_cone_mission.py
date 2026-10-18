#!/usr/bin/env python3
"""Stand-in for cone_avoidance_1005 in the simulator: drives straight at 7 km/h
and reports the mission state, DONE after --done-after seconds (and keeps
driving afterwards, like the real stack). Run by avoidance_supervisor_node in
stanley_dynamics_sim.launch.py (avoidance_demo_sec > 0); never on the vehicle.

    fake_cone_mission.py --done-after 20 --angle-topic /simulation/target_angle \\
        --speed-topic /simulation/target_speed --brake-topic /simulation/brake
"""

import argparse
import json
import time

import rclpy
from std_msgs.msg import Bool, Float32, String


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--done-after", type=float, default=20.0)
    parser.add_argument("--speed-kmph", type=float, default=7.0)
    parser.add_argument("--angle", type=float, default=None,
                        help="command that means straight; default = vehicle.yaml steering_command_offset_rad")
    parser.add_argument("--angle-topic", default="/target_angle")
    parser.add_argument("--speed-topic", default="/target_speed")
    parser.add_argument("--brake-topic", default="/brake")
    parser.add_argument("--status-topic", default="/cone_avoidance_1005/mission_status")
    args = parser.parse_args()
    if args.angle is None:
        import yaml
        from krowd_stanley_control.config_paths import source_config
        with open(source_config("krowd_stanley_control", "vehicle.yaml")) as stream:
            params = yaml.safe_load(stream)["/**"]["ros__parameters"]
        args.angle = float(params.get("steering_command_offset_rad", 0.0))
    rclpy.init()
    node = rclpy.create_node("fake_cone_mission")
    angle = node.create_publisher(Float32, args.angle_topic, 10)
    speed = node.create_publisher(Float32, args.speed_topic, 10)
    brake = node.create_publisher(Bool, args.brake_topic, 10)
    status = node.create_publisher(String, args.status_topic, 10)
    t0 = time.monotonic()

    def tick():
        state = "DONE" if time.monotonic() - t0 >= args.done_after else "CRUISE"
        brake.publish(Bool(data=False))
        angle.publish(Float32(data=args.angle))
        speed.publish(Float32(data=args.speed_kmph))
        status.publish(String(data=json.dumps({"state": state, "v_cmd": args.speed_kmph / 3.6})))

    node.create_timer(0.05, tick)
    node.get_logger().info(f"fake cone mission: straight (angle {args.angle:+.2f} rad) at {args.speed_kmph} km/h, DONE after {args.done_after} s")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
