"""Select localization odometry and adapt it into the control VehicleState.

The local ESKF odometry is the primary source because it combines the sensors
needed for smooth control. A map-referenced global ESKF pose is the fallback.
Every pose is transformed into output_frame before publication so the
controller and mapped trajectory always use the same coordinates.
"""

from copy import deepcopy
import math

import rclpy
import tf2_ros
from krowd_control_interfaces.msg import VehicleState
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import TransformException


class VehicleStateNode(Node):
    """Publish one frame-consistent VehicleState with primary/fallback failover."""

    def __init__(self):
        super().__init__("vehicle_state_node")

        # input_odom_topic remains as a compatibility default. A non-empty
        # primary_odom_topic takes precedence.
        self.declare_parameter("input_odom_topic", "/gnss_imu/pose")
        self.declare_parameter("primary_odom_topic", "")
        self.declare_parameter("fallback_odom_topic", "")
        self.declare_parameter("output_frame", "map")
        self.declare_parameter("primary_timeout_sec", 0.5)
        self.declare_parameter("primary_recovery_messages", 3)
        self.declare_parameter("transform_timeout_sec", 0.5)
        self.declare_parameter("vehicle_state_topic", "/localization/vehicle_state")
        self.declare_parameter("min_speed_for_curvature_kmph", 1.8)
        self.declare_parameter("report_period_sec", 10.0)

        self._min_speed_for_curvature = (
            float(self.get_parameter("min_speed_for_curvature_kmph").value) / 3.6
        )
        self._output_frame = str(self.get_parameter("output_frame").value).strip()
        self._primary_timeout_ns = int(
            float(self.get_parameter("primary_timeout_sec").value) * 1e9
        )
        self._recovery_required = int(
            self.get_parameter("primary_recovery_messages").value
        )
        if self._primary_timeout_ns <= 0 or self._recovery_required < 1:
            raise RuntimeError(
                "primary_timeout_sec must be positive and "
                "primary_recovery_messages must be at least 1"
            )
        self._transform_timeout = Duration(
            seconds=float(self.get_parameter("transform_timeout_sec").value)
        )

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._publisher = self.create_publisher(
            VehicleState, str(self.get_parameter("vehicle_state_topic").value), 10
        )

        configured_primary = str(
            self.get_parameter("primary_odom_topic").value
        ).strip()
        primary_topic = configured_primary or str(
            self.get_parameter("input_odom_topic").value
        ).strip()
        fallback_topic = str(self.get_parameter("fallback_odom_topic").value).strip()
        if not primary_topic:
            raise RuntimeError("primary_odom_topic/input_odom_topic is empty")
        if fallback_topic == primary_topic:
            raise RuntimeError("primary and fallback odometry topics must differ")

        self.create_subscription(Odometry, primary_topic, self._on_primary, 20)
        if fallback_topic:
            self.create_subscription(Odometry, fallback_topic, self._on_fallback, 20)

        self._active_source = "primary"
        self._last_primary_ns = None
        self._primary_recovery_count = 0
        self._warned_transform = set()
        self._received = 0
        self._moved_without_speed = 0
        self._last_position = None
        self.create_timer(
            float(self.get_parameter("report_period_sec").value), self._report
        )

        fallback_description = fallback_topic or "disabled"
        output_description = self._output_frame or "input frame"
        self.get_logger().info(
            f"vehicle_state_node: primary '{primary_topic}', fallback "
            f"'{fallback_description}' -> "
            f"'{self.get_parameter('vehicle_state_topic').value}' in "
            f"'{output_description}', curvature above "
            f"{self._min_speed_for_curvature * 3.6:.1f} km/h"
        )

    def _on_primary(self, msg: Odometry) -> None:
        transformed = self._in_output_frame(msg, "primary")
        if transformed is None:
            return
        now_ns = self.get_clock().now().nanoseconds
        self._last_primary_ns = now_ns
        if self._active_source == "fallback":
            self._primary_recovery_count += 1
            if self._primary_recovery_count < self._recovery_required:
                return
            self._switch_source("primary")
        self._publish(transformed)

    def _on_fallback(self, msg: Odometry) -> None:
        now_ns = self.get_clock().now().nanoseconds
        primary_fresh = (
            self._last_primary_ns is not None
            and now_ns - self._last_primary_ns <= self._primary_timeout_ns
        )
        if primary_fresh:
            return
        transformed = self._in_output_frame(msg, "fallback")
        if transformed is None:
            return
        if self._active_source != "fallback":
            self._switch_source("fallback")
        else:
            # A stale interval breaks the consecutive-primary recovery streak.
            self._primary_recovery_count = 0
        self._publish(transformed)

    def _switch_source(self, source: str) -> None:
        self._active_source = source
        self._primary_recovery_count = 0
        if source == "fallback":
            self.get_logger().warn(
                "primary odometry is unavailable; using map-referenced fallback"
            )
        else:
            self.get_logger().info(
                f"primary odometry recovered for {self._recovery_required} "
                "consecutive messages; switching back"
            )

    def _in_output_frame(self, msg: Odometry, source: str) -> Odometry | None:
        source_frame = str(msg.header.frame_id).strip()
        if not source_frame:
            self.get_logger().warn(f"discarding {source} odometry with empty frame_id")
            return None
        if not self._output_frame or source_frame == self._output_frame:
            return msg
        try:
            transform = self._latest_transform(source_frame, msg)
            output = deepcopy(msg)
            output.header.frame_id = self._output_frame
            output.pose.pose = do_transform_pose(msg.pose.pose, transform)
            self._warned_transform.discard((source, source_frame))
            return output
        except TransformException as error:
            key = (source, source_frame)
            if key not in self._warned_transform:
                self.get_logger().warn(
                    f"cannot transform {source} odometry {source_frame} -> "
                    f"{self._output_frame}: {error}"
                )
                self._warned_transform.add(key)
            return None

    def _latest_transform(self, source_frame: str, msg: Odometry):
        """output_frame <- source_frame, without ever waiting for it.

        Until 2026-09-20 this looked the transform up AT the message stamp with
        a 0.1 s timeout. The ESKF publishes its odometry and its odom TF
        together, so the TF for a stamp is never in the buffer yet when the
        odometry callback runs, and waiting for it blocks the very executor
        that would deliver it: every message cost the full timeout. Measured
        on the vehicle: 100 Hz in, 6-10 Hz out, each pose 213 ms old (the
        input itself is 17 ms old) - 0.3 m behind the vehicle at 5 km/h.

        The latest transform is the right one anyway: map <- odom only moves
        when the ESKF corrects its drift, centimetres per second, so one that
        is a cycle old places the pose as well as the exact one would. What
        must not happen is using one that stopped updating, hence the age
        check against transform_timeout_sec.
        """

        transform = self._tf_buffer.lookup_transform(
            self._output_frame, source_frame, Time()
        )
        transform_stamp = Time.from_msg(transform.header.stamp)
        message_stamp = Time.from_msg(msg.header.stamp)
        # A chain of static edges only has stamp 0: nothing there can go stale.
        if transform_stamp.nanoseconds and message_stamp.nanoseconds:
            age = message_stamp - transform_stamp
            if age > self._transform_timeout:
                raise TransformException(
                    f"latest transform is {age.nanoseconds * 1e-9:.2f} s older than "
                    f"the odometry (limit {self._transform_timeout.nanoseconds * 1e-9:.2f} s)"
                )
        return transform

    def _publish(self, msg: Odometry) -> None:
        speed = float(msg.twist.twist.linear.x)
        yaw_rate = float(msg.twist.twist.angular.z)
        if not math.isfinite(speed) or not math.isfinite(yaw_rate):
            self.get_logger().warn(
                f"discarding non-finite twist: speed={speed}, yaw_rate={yaw_rate}"
            )
            return

        self._track_speedless_motion(msg, speed)
        state = VehicleState()
        state.odometry = msg
        state.curvature = (
            yaw_rate / speed
            if abs(speed) >= self._min_speed_for_curvature
            else 0.0
        )
        self._publisher.publish(state)
        self._received += 1

    def _track_speedless_motion(self, msg: Odometry, speed: float) -> None:
        position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self._last_position is not None and speed == 0.0:
            moved = math.hypot(
                position[0] - self._last_position[0],
                position[1] - self._last_position[1],
            )
            if moved > 0.05:
                self._moved_without_speed += 1
        self._last_position = position

    def _report(self) -> None:
        if self._moved_without_speed > 0:
            self.get_logger().warn(
                f"{self._moved_without_speed} of {self._received} poses moved while "
                "reporting exactly zero speed; check the selected odometry twist"
            )
        self._moved_without_speed = 0


def main(args=None):
    rclpy.init(args=args)
    node = VehicleStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
