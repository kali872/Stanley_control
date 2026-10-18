import math

import pytest

from krowd_stanley_control.route_geometry import (
    back_at_route_start,
    travelled_arc_length,
)


WITHIN = 10.0
CORRIDOR = 3.0
CLEAR = 2.0


def _on_straight(x):
    """Segment index and fraction of a point at x on a 1 m-spaced +x route."""

    index = max(int(math.floor(x)), 0)
    return index, min(max(x - index, 0.0), 1.0)


def test_travelled_arc_length_sums_segments_and_the_fraction(straight_route):
    route = straight_route(10)
    assert travelled_arc_length(route, 0, 0.0) == pytest.approx(0.0)
    assert travelled_arc_length(route, 3, 0.5) == pytest.approx(3.5)
    assert travelled_arc_length(route, 20, 1.0) == pytest.approx(9.0)  # clamped


def test_a_restarted_drive_first_seen_several_metres_in_resumes(straight_route):
    """The RTK pose reappears ~5 m into a drive; 7 m along still counts."""

    route = straight_route(60)
    assert back_at_route_start(
        route, *_on_straight(7.0), 7.0, 0.5, WITHIN, CORRIDOR, CLEAR
    )


def test_further_along_than_the_restart_distance_stays_stopped(straight_route):
    route = straight_route(60)
    assert not back_at_route_start(
        route, *_on_straight(12.0), 12.0, 0.0, WITHIN, CORRIDOR, CLEAR
    )


def test_beside_the_start_but_outside_the_corridor_stays_stopped(straight_route):
    route = straight_route(60)
    assert not back_at_route_start(
        route, *_on_straight(2.0), 2.0, 4.0, WITHIN, CORRIDOR, CLEAR
    )


def test_behind_the_start_stays_stopped(straight_route):
    """Projection clamps to the first point; the vehicle is 15 m away from it."""

    route = straight_route(60)
    assert not back_at_route_start(route, 0, 0.0, -15.0, 0.0, WITHIN, CORRIDOR, CLEAR)


def test_a_loop_whose_end_touches_its_start_does_not_restart_where_it_stopped(
    circular_route,
):
    route = circular_route(10.0, 62)  # 61 m of a 62.8 m circle
    assert math.dist(route[0], route[-1]) < CLEAR
    x, y = route[0]
    assert not back_at_route_start(route, 0, 0.0, x, y, WITHIN, CORRIDOR, CLEAR)


def test_the_same_loop_restarts_once_clear_of_its_end_point(circular_route):
    route = circular_route(10.0, 62)
    x, y = route[4]
    assert math.dist((x, y), route[-1]) > CLEAR
    assert back_at_route_start(route, 4, 0.0, x, y, WITHIN, CORRIDOR, CLEAR)


def test_restart_distance_zero_disables_the_release(straight_route):
    route = straight_route(60)
    assert not back_at_route_start(
        route, *_on_straight(1.0), 1.0, 0.0, 0.0, CORRIDOR, CLEAR
    )


# ---- stanley_node: a loop whose end lies next to its start (2026-09-26) ----

def _loop_node(front_xy):
    """StanleyNode's release check on a rectangle loop ending 3 m from its
    start, latched at the goal (0, 3) with the front axle at front_xy."""

    from types import SimpleNamespace

    from geometry_msgs.msg import PoseStamped
    from krowd_control_interfaces.msg import Trajectory, VehicleState

    from krowd_stanley_control.stanley_node import StanleyNode
    from krowd_stanley_control.tracking_common import track_trajectory

    points = (
        [(float(x), 0.0) for x in range(0, 31)]
        + [(30.0, float(y)) for y in range(1, 11)]
        + [(float(x), 10.0) for x in range(29, -1, -1)]
        + [(0.0, float(y)) for y in range(9, 2, -1)]
    )
    msg = Trajectory()
    msg.trajectory_id = "loop"
    msg.goal_mode = Trajectory.STOP_AT_END
    msg.valid_until.sec = 2_000_000_000
    msg.path.header.frame_id = "map"
    for x, y in points:
        pose = PoseStamped()
        pose.pose.position.x, pose.pose.position.y = x, y
        pose.pose.orientation.w = 1.0
        msg.path.poses.append(pose)
    msg.curvatures = [0.0] * len(points)
    msg.target_speeds = [1.0] * len(points)
    state = VehicleState()
    state.odometry.header.frame_id = "map"
    state.odometry.pose.pose.position.x = front_xy[0] - 1.34  # yaw 0: front 1.34 m ahead
    state.odometry.pose.pose.position.y = front_xy[1]
    state.odometry.pose.pose.orientation.w = 1.0
    now_ns = 10_000_000_000
    node = SimpleNamespace(
        _restart_near_start=WITHIN,
        _vehicle_state=state,
        _vehicle_state_ns=now_ns,
        _trajectory=track_trajectory(msg, CORRIDOR),
        _state_timeout_ns=500_000_000,
        _vehicle=SimpleNamespace(rear_axle_offset_m=0.0, wheelbase_m=1.34),
        _corridor_half_width=CORRIDOR,
        _goal_tolerance=CLEAR / 2.0,
        _goal_latched=True,
        _latched_trajectory_id="loop",
        _goal_latch_xy=(0.0, 3.0),
        _left_goal=False,
        _last_projection_position=float(len(points) - 1),
        get_logger=lambda: SimpleNamespace(info=lambda *_: None),
    )
    return StanleyNode, node, now_ns


def test_coasting_past_the_goal_onto_the_start_does_not_restart():
    """(0, 0.5) is on the route start and 2.5 m clear of the end - the
    2026-09-26 flapping case - but the vehicle never left the stop."""

    cls, node, now_ns = _loop_node((0.5, 0.2))
    assert not cls._release_goal_latch_at_route_start(node, now_ns)
    assert node._goal_latched


def test_driven_away_and_back_to_the_start_restarts_from_the_start():
    cls, node, now_ns = _loop_node((15.0, 0.0))  # 15 m from the stop
    assert not cls._release_goal_latch_at_route_start(node, now_ns)  # not at start
    assert node._left_goal
    cls2, node2, _ = _loop_node((3.0, 0.1))
    node2._left_goal = True
    assert cls2._release_goal_latch_at_route_start(node2, now_ns)
    assert not node2._goal_latched
    # projection continues from the start, not from the end it latched at
    assert node2._last_projection_position == pytest.approx(3.0, abs=0.2)
