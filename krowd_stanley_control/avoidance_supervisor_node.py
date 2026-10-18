"""Runs cone_avoidance_1005 while stanley_node has handed the vehicle over.

    /stanley_node/handoff (JSON, latched)   active, zone, past_end
        -> start `launch_command` as a child process group
    /cone_avoidance_1005/mission_status      state == done_state
    handoff.past_end                         the vehicle passed the zone end
    no command on command_topic for          first_command_timeout_sec
    max_duration_sec                         whichever comes FIRST
        -> SIGINT the child group, wait, SIGKILL if needed
        -> /stanley_node/resume true          (stanley_node drives again)

The cone stack keeps driving after DONE, so ending it promptly matters: the
CAN bridge zeroes the speed 0.2 s after its last command, stanley_node resumes
only once the child has exited. Nothing here touches cone_avoidance_1005's
code; it is launched exactly as from a terminal.
"""

import json
import os
import shlex
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, String


class AvoidanceSupervisorNode(Node):
    def __init__(self):
        super().__init__("avoidance_supervisor_node")
        self.declare_parameter("handoff_topic", "/stanley_node/handoff")
        self.declare_parameter("resume_topic", "/stanley_node/resume")
        self.declare_parameter("mission_status_topic", "/cone_avoidance_1005/mission_status")
        self.declare_parameter("command_topic", "/target_speed")
        self.declare_parameter(
            "launch_command",
            "ros2 launch cone_avoidance_1005 cone_avoidance_1005.launch.py use_can:=false use_rviz:=false",
        )
        self.declare_parameter("done_state", "DONE")
        self.declare_parameter("first_command_timeout_sec", 45.0)
        self.declare_parameter("guard_status_topic", "/cone_avoidance_1005/command_guard_status")
        # cone_avoidance_1005's command_guard starts with its controller-brake
        # input TRUE and, in the standalone launch, nothing ever publishes that
        # topic (it is integration_control's in the integrated launch): the
        # guard stays in CONTROLLER_BRAKE and the vehicle never moves (found
        # 2026-10-08 in the sim; same on the vehicle on 2026-10-07). While the
        # cone launch runs this node publishes false there at 20 Hz, and true
        # while it is being shut down. Empty = do not.
        self.declare_parameter("controller_brake_topic", "/cone_avoidance_1005/controller/brake")
        self.declare_parameter("max_duration_sec", 120.0)
        self.declare_parameter("shutdown_grace_sec", 5.0)
        self.declare_parameter("status_topic", "/avoidance_supervisor/status")

        self._command = shlex.split(str(self.get_parameter("launch_command").value))
        self._done_state = str(self.get_parameter("done_state").value)
        self._first_timeout = float(self.get_parameter("first_command_timeout_sec").value)
        self._max_duration = float(self.get_parameter("max_duration_sec").value)
        self._grace = float(self.get_parameter("shutdown_grace_sec").value)

        self._child: subprocess.Popen | None = None
        self._child_started: float | None = None
        self._stopping_since: float | None = None
        self._stop_reason = ""
        self._active = False
        self._past_end = False
        self._zone = ""
        self._mission_state = ""
        self._last_command: float | None = None
        self._commands = 0
        self._guard_status = ""
        self._mission_msgs = 0
        self._ended = 0
        # After a resume, ignore the (latched) hand-off until stanley has
        # reported it inactive once: on 2026-10-07 the still-active message
        # restarted the cone launch 0.2 s after it had been ended.
        self._await_inactive = False

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._resume_pub = self.create_publisher(Bool, str(self.get_parameter("resume_topic").value), 10)
        brake_topic = str(self.get_parameter("controller_brake_topic").value).strip()
        self._brake_pub = self.create_publisher(Bool, brake_topic, 10) if brake_topic else None
        if self._brake_pub is not None:
            self.create_timer(0.05, self._release_brake)
        self._status_pub = self.create_publisher(String, str(self.get_parameter("status_topic").value), 10)
        self.create_subscription(String, str(self.get_parameter("handoff_topic").value), self._on_handoff, latched)
        self.create_subscription(String, str(self.get_parameter("mission_status_topic").value), self._on_mission, 10)
        self.create_subscription(Float32, str(self.get_parameter("command_topic").value), self._on_command, 10)
        self.create_subscription(String, str(self.get_parameter("guard_status_topic").value), self._on_guard, 10)
        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            "avoidance supervisor: on hand-off runs `" + " ".join(self._command) + "`; ends it on "
            f"mission {self._done_state} / zone end / {self._max_duration:.0f} s / no command in "
            f"{self._first_timeout:.0f} s, then resumes stanley"
        )

    # ------------------------------------------------------------ inputs
    def _on_handoff(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            active = bool(payload.get("active", False))
        except ValueError:
            return
        if not active:
            self._await_inactive = False
        if self._await_inactive:
            return
        self._active = active
        self._past_end = bool(payload.get("past_end", False)) if active else False
        self._zone = str(payload.get("zone", "")) if active else ""

    def _on_mission(self, msg: String) -> None:
        self._mission_msgs += 1
        try:
            self._mission_state = str(json.loads(msg.data).get("state", ""))
        except (ValueError, AttributeError):
            pass

    def _on_command(self, msg: Float32) -> None:
        if self._child is not None:
            if self._commands == 0:
                self.get_logger().info(
                    f"first cone_avoidance command seen {time.monotonic() - self._child_started:.1f} s after launch"
                )
            self._last_command = time.monotonic()
            self._commands += 1

    def _on_guard(self, msg: String) -> None:
        self._guard_status = msg.data[:160]
        if self._child is not None and self._last_command is None:
            # The guard publishes the actuator commands itself: its status is
            # proof that commands are flowing even if /target_speed is remapped.
            self._last_command = time.monotonic()

    # ------------------------------------------------------------ control
    def _tick(self) -> None:
        now = time.monotonic()
        if self._child is None:
            if self._active and self._stopping_since is None:
                self._start(now)
            self._publish_status("idle" if not self._active else "starting")
            return

        exited = self._child.poll() is not None
        if self._stopping_since is not None:
            if exited:
                self._finish(now)
            elif now - self._stopping_since > self._grace:
                self.get_logger().error("cone_avoidance did not exit on SIGINT: SIGKILL")
                self._signal(signal.SIGKILL)
                self._stopping_since = now  # wait another grace period for the kill
            self._publish_status(f"stopping ({self._stop_reason})")
            return

        reason = None
        if exited:
            reason = f"cone_avoidance exited by itself (code {self._child.returncode})"
        elif not self._active:
            reason = "stanley withdrew the hand-off"
        elif self._mission_state == self._done_state:
            reason = f"cone mission {self._done_state}"
        elif self._past_end:
            reason = f"vehicle passed the end of zone #{self._zone}"
        elif self._last_command is None and now - self._child_started > self._first_timeout:
            reason = f"no command from cone_avoidance within {self._first_timeout:.0f} s"
        elif now - self._child_started > self._max_duration:
            reason = f"cone mission exceeded {self._max_duration:.0f} s"
        if reason is not None:
            self._stop(reason, now)
        self._publish_status(f"running zone=#{self._zone} mission={self._mission_state or '?'} t={now - self._child_started:.0f}s")

    def _start(self, now: float) -> None:
        self._mission_state = ""
        self._last_command = None
        self._commands = 0
        self._mission_msgs = 0
        self._guard_status = ""
        try:
            self._child = subprocess.Popen(self._command, start_new_session=True)
        except OSError as error:
            self.get_logger().error(f"cannot start cone_avoidance (`{' '.join(self._command)}`): {error}; resuming stanley")
            self._resume_pub.publish(Bool(data=True))
            return
        self._child_started = now
        self.get_logger().warning(f"zone #{self._zone}: started cone_avoidance (pid {self._child.pid})")

    def _stop(self, reason: str, now: float) -> None:
        self._stop_reason = reason
        self._stopping_since = now
        self.get_logger().warning(
            f"ending cone_avoidance after {now - self._child_started:.1f} s: {reason} "
            f"[commands seen {self._commands}, mission_status msgs {self._mission_msgs}, "
            f"last mission state '{self._mission_state}', guard: {self._guard_status or 'none'}]"
        )
        self._signal(signal.SIGINT)

    def _signal(self, sig) -> None:
        if self._child is None or self._child.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(self._child.pid), sig)
        except ProcessLookupError:
            pass

    def _finish(self, now: float) -> None:
        self._child = None
        self._stopping_since = None
        self._ended += 1
        self.get_logger().warning(f"cone_avoidance ended ({self._stop_reason}); resuming stanley")
        self._resume_pub.publish(Bool(data=True))
        # The hand-off message goes inactive once stanley resumes; until then
        # do not start the child again.
        self._active = False
        self._await_inactive = True

    def _release_brake(self) -> None:
        """The controller-brake input of cone_avoidance's guard: released
        while the cone launch runs, applied while it is being ended."""

        if self._child is None:
            return
        self._brake_pub.publish(Bool(data=self._stopping_since is not None))

    def _publish_status(self, text: str) -> None:
        self._status_pub.publish(String(data=text))

    def destroy_node(self):
        if self._child is not None and self._child.poll() is None:
            self._signal(signal.SIGINT)
            try:
                self._child.wait(timeout=self._grace)
            except subprocess.TimeoutExpired:
                self._signal(signal.SIGKILL)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AvoidanceSupervisorNode()
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
