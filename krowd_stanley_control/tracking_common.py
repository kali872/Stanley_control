"""What a route-tracking controller needs besides its steering law.

The Trajectory as the controller walks it (TrackedTrajectory, track_trajectory),
where a route ends (end_of_route_reason), the yaw of a pose, and the two limits
every steering command passes through (clamp_steering, rate_limited).

These lived in pure_pursuit_node.py / pure_pursuit.py while this package still
carried krowd_path_tracking's Pure Pursuit controller next to Stanley. The
controller was removed on 2026-09-20 (this package drives Stanley only; Pure
Pursuit remains in krowd_slam/krowd_path_tracking) and what Stanley shared with
it moved here unchanged.
"""

from dataclasses import dataclass, replace
import math

from krowd_control_interfaces.msg import Trajectory

from mola_slam_mapping.route_io import (
    RouteProjection,
)
from krowd_stanley_control.route_geometry import (
    unroll_loop,
)
from mola_slam_mapping.route_visualization import RoutePoint, make_route_point


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


@dataclass(frozen=True)
class TrackedTrajectory:
    """One Trajectory, converted once into the shapes the controller walks.

    `route`/`planar` are the trajectory as published and are what the vehicle
    is projected onto. `ahead_route`/`ahead_planar` are what is walked forwards
    (and, for Stanley's heading window, backwards) from that projection: the
    same route for an open one, three laps of it for a closed loop, entered at
    the middle lap (`lap_offset`) so a walk crosses the seam instead of stopping
    at the last waypoint. See ahead_of.
    """

    trajectory_id: str
    frame_id: str
    goal_mode: int
    valid_until_ns: int
    route: list[RoutePoint]
    planar: list[tuple[float, float]]
    target_speeds: list[float]
    closed_loop: bool
    ahead_route: list[RoutePoint]
    ahead_planar: list[tuple[float, float]]
    lap_offset: int
    # Per waypoint of ahead_route (1/m, signed): the route publisher's smoothed
    # curvature, walked ahead to shorten the lookahead into corners.
    ahead_curvatures: list[float] = ()

    def ahead_of(self, projection: RouteProjection) -> RouteProjection:
        """The projection re-expressed on ahead_route/ahead_planar."""

        if self.lap_offset == 0:
            return projection
        return replace(
            projection, segment_index=projection.segment_index + self.lap_offset
        )


def track_trajectory(msg: Trajectory, corridor_half_width: float) -> TrackedTrajectory:
    """Convert a validated Trajectory for either controller.

    A trajectory is a closed loop when its last pose repeats its first, which is
    how route_publisher_node publishes one. STOP_AT_END never is: a goal at the
    end of a loop that has no end would be driven through, not stopped at.
    """

    route = []
    planar = []
    for pose in msg.path.poses:
        position = pose.pose.position
        route.append(
            make_route_point(
                float(position.x),
                float(position.y),
                float(position.z),
                yaw_from_quaternion(pose.pose.orientation),
                corridor_half_width,
            )
        )
        planar.append((float(position.x), float(position.y)))

    goal_mode = int(msg.goal_mode)
    closed_loop = (
        goal_mode != Trajectory.STOP_AT_END
        and len(planar) >= 4
        and math.hypot(planar[-1][0] - planar[0][0], planar[-1][1] - planar[0][1])
        < 1e-3
    )
    return TrackedTrajectory(
        trajectory_id=msg.trajectory_id,
        frame_id=msg.path.header.frame_id,
        goal_mode=goal_mode,
        valid_until_ns=int(msg.valid_until.sec) * 1_000_000_000
        + int(msg.valid_until.nanosec),
        route=route,
        planar=planar,
        target_speeds=[float(value) for value in msg.target_speeds],
        closed_loop=closed_loop,
        ahead_route=unroll_loop(route, 3) if closed_loop else route,
        ahead_planar=unroll_loop(planar, 3) if closed_loop else planar,
        lap_offset=len(route) - 1 if closed_loop else 0,
        ahead_curvatures=(
            unroll_loop([float(value) for value in msg.curvatures], 3)
            if closed_loop
            else [float(value) for value in msg.curvatures]
        ),
    )


def end_of_route_reason(
    trajectory: TrackedTrajectory, remaining_m: float, goal_tolerance_m: float
) -> str | None:
    """Why tracking stops at the end of the route; None when it does not.

    STOP_AT_END reaches a goal and the caller latches it. Any other goal mode on
    an open route stops at the end too - past the last waypoint there is nothing
    mapped to follow, and the lookahead would clamp onto a point the vehicle has
    already driven over - but does not latch, so tracking resumes as soon as the
    vehicle is somewhere else along the route. A closed loop has no end.
    """

    if trajectory.closed_loop or remaining_m > goal_tolerance_m:
        return None
    if trajectory.goal_mode == Trajectory.STOP_AT_END:
        return f"GOAL_REACHED remaining={remaining_m:.2f}m"
    return f"END_OF_ROUTE remaining={remaining_m:.2f}m"


def rate_limited(
    previous_rad: float, desired_rad: float, max_rate_rad_s: float, dt_s: float
) -> float:
    """Clamp how far the steering command may move in one control period.

    Pure Pursuit recomputes its angle from scratch every frame, so a single
    outlier pose becomes a full-amplitude step at the actuator. MPC had this
    constraint built in (MAX_TAU); here it has to be applied explicitly.
    """

    if max_rate_rad_s <= 0.0 or dt_s <= 0.0:
        return desired_rad
    step = max_rate_rad_s * dt_s
    return max(previous_rad - step, min(previous_rad + step, desired_rad))


def clamp_steering(angle_rad: float, limit_rad: float) -> float:
    return max(-limit_rad, min(limit_rad, angle_rad))
