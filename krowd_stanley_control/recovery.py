"""Drive back to the route after leaving the corridor, instead of just stopping.

Until 2026-09-23 a vehicle that left the corridor (|offset| > half width)
braked - STOP OUTSIDE_CORRIDOR - and stayed braked until somebody pushed it
back inside. On the 2026-09-22 laps that happened in every bend the steering
could not make, with the vehicle a few centimetres over the line and pointing
roughly the right way.

RECOVERING instead: at a crawl speed, steer at a point on the route some
metres AHEAD of the projection (never at the nearest point - that arrives
square-on and trips the heading gate), with Pure Pursuit's arc law, which does
not saturate on a large offset the way Stanley's cross-track term does. Back
inside `done_offset_m` AND within `done_heading_rad` of the route direction,
tracking resumes; both, so a vehicle crossing the route at an angle is not
handed to Stanley mid-crossing. The two thresholds sit inside the corridor,
which is the hysteresis.

It gives up - STOP RECOVERY_FAILED, until the vehicle is back on track by
other means - when the offset exceeds `max_offset_m`, when `max_duration_s`
passes, or when the target is not in front of the vehicle. And it never starts
on an offset that JUMPED: an offset that moves more than `max_jump_m` between
two cycles is the pose estimate moving, not the vehicle, and driving on it
would be driving on a guess.

Plain numbers, no ROS, like start_ramp.py.
"""

import math


class Recovery:
    OFF = "OFF"
    RECOVERING = "RECOVERING"
    DONE = "DONE"
    FAILED = "FAILED"

    def __init__(
        self,
        done_offset_m: float,
        done_heading_rad: float,
        max_offset_m: float,
        max_duration_s: float,
        max_jump_m: float,
    ):
        if min(done_offset_m, done_heading_rad, max_offset_m, max_duration_s, max_jump_m) <= 0.0:
            raise ValueError("recovery thresholds must all be positive")
        if max_offset_m <= done_offset_m:
            raise ValueError("recovery max_offset_m must exceed done_offset_m")
        self.done_offset_m = done_offset_m
        self.done_heading_rad = done_heading_rad
        self.max_offset_m = max_offset_m
        self.max_duration_s = max_duration_s
        self.max_jump_m = max_jump_m
        self._started_ns: int | None = None
        self._failed = False
        self._last_offset: float | None = None
        self.last_duration_s = 0.0
        """How long the last recovery took (DONE or FAILED)."""

    @property
    def active(self) -> bool:
        return self._started_ns is not None

    def elapsed_s(self, now_ns: int) -> float:
        return 0.0 if self._started_ns is None else (now_ns - self._started_ns) * 1e-9

    def reset(self) -> None:
        self._started_ns = None
        self._failed = False

    def forget_offset(self) -> None:
        """The next offset is not compared with the last one: another
        controller moved the vehicle meanwhile (cone-avoidance hand-off), so a
        large change is not a localization jump."""

        self._last_offset = None

    def step(
        self,
        now_ns: int,
        signed_offset_m: float,
        heading_error_rad: float,
        on_track: bool,
        target_ahead: bool,
        hold_timer: bool = False,
    ) -> str:
        """What to do this cycle. OFF: track normally. RECOVERING: steer at the
        recovery target. DONE: the route is regained, track normally from here.
        FAILED: stop; stays FAILED until the vehicle is on track again.

        DONE needs `on_track` as well (2026-10-17): before the first waypoint
        the projection is clamped to the route start, the offset and heading
        look fine, but the corridor says off - without this the controller
        flipped RECOVERING/TRACKING every cycle and the speed command with it.
        `hold_timer` keeps the recovery clock at zero (the vehicle is still
        driving up to the route start, not failing to regain it)."""

        jumped = (
            self._last_offset is not None
            and abs(signed_offset_m - self._last_offset) > self.max_jump_m
        )
        self._last_offset = signed_offset_m

        if self._failed:
            if on_track:
                self._failed = False
                return self.OFF
            return self.FAILED

        if not self.active:
            if on_track:
                return self.OFF
            if jumped:
                self._failed = True
                return self.FAILED
            self._started_ns = now_ns
            return self.RECOVERING

        if hold_timer:
            self._started_ns = now_ns
        if (
            on_track
            and abs(signed_offset_m) <= self.done_offset_m
            and abs(heading_error_rad) <= self.done_heading_rad
        ):
            self.last_duration_s = self.elapsed_s(now_ns)
            self._started_ns = None
            return self.DONE
        if (
            abs(signed_offset_m) > self.max_offset_m
            or self.elapsed_s(now_ns) > self.max_duration_s
            or not target_ahead
        ):
            self.last_duration_s = self.elapsed_s(now_ns)
            self._started_ns = None
            self._failed = True
            return self.FAILED
        return self.RECOVERING


def pursuit_steering(target_x_body: float, target_y_body: float, wheelbase_m: float) -> float:
    """Road-wheel angle that drives the rear axle onto the arc through a point
    given in the body frame (x forward, y left): atan2(2 L y, x^2 + y^2)."""

    d2 = target_x_body * target_x_body + target_y_body * target_y_body
    if d2 <= 1e-9:
        return 0.0
    return math.atan2(2.0 * wheelbase_m * target_y_body, d2)


def to_body_frame(x: float, y: float, yaw: float, px: float, py: float) -> tuple[float, float]:
    dx, dy = px - x, py - y
    c, s = math.cos(yaw), math.sin(yaw)
    return c * dx + s * dy, -s * dx + c * dy
