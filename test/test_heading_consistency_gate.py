"""The drive gate must reject a latched-but-wrong ESKF map heading."""

import math
from types import SimpleNamespace

from krowd_control_interfaces.msg import VehicleState
from nav_msgs.msg import Odometry

from krowd_stanley_control.stanley_node import StanleyNode


def pose(yaw_deg):
    message = Odometry()
    message.header.frame_id = "map"
    message.header.stamp.sec = 100
    message.child_frame_id = "base_link"
    message.pose.pose.orientation.z = math.sin(math.radians(yaw_deg) / 2.0)
    message.pose.pose.orientation.w = math.cos(math.radians(yaw_deg) / 2.0)
    return message


def harness(vehicle_yaw=124.0, map_yaw=0.0):
    now_ns = 10_000_000_000
    state = VehicleState()
    state.odometry = pose(vehicle_yaw)
    return SimpleNamespace(
        _map_heading_consistency_required=True,
        _heading_verified_ns=now_ns,
        _heading_source="rtk_course",
        _heading_source_ns=now_ns,
        _heading_status_timeout_ns=2_000_000_000,
        _map_heading_timeout_ns=500_000_000,
        _map_heading_max_disagreement=math.radians(15.0),
        _map_heading_verified=True,
        _map_heading_verified_ns=now_ns,
        _map_heading_pose=pose(map_yaw),
        _map_heading_pose_ns=now_ns,
        _vehicle_state=state,
    ), now_ns


def test_verified_but_wrong_eskf_yaw_stops_drive():
    node, now_ns = harness()
    reason = StanleyNode._map_heading_consistency_problem(node, now_ns)
    assert reason.startswith("HEADING_DISAGREEMENT")


def test_current_map_heading_allows_drive():
    node, now_ns = harness(vehicle_yaw=4.0, map_yaw=0.0)
    assert StanleyNode._map_heading_consistency_problem(node, now_ns) is None


def test_lidar_heading_source_is_never_accepted_as_confirmed_yaw():
    node, now_ns = harness()
    node._heading_source = "lidar_map"
    assert StanleyNode._map_heading_consistency_problem(
        node, now_ns
    ) == "HEADING_SOURCE_INVALID source=lidar_map"


def test_rtk_course_does_not_require_lidar_map_to_be_running():
    node, now_ns = harness()
    node._heading_source = "rtk_course"
    node._map_heading_verified = False
    assert StanleyNode._map_heading_consistency_problem(node, now_ns) is None


def test_heading_status_must_be_fresh():
    node, now_ns = harness()
    node._heading_source_ns -= 3_000_000_000
    assert StanleyNode._map_heading_consistency_problem(node, now_ns) == (
        "HEADING_STATUS_STALE"
    )


def hold_harness(vehicle_yaw, map_yaw=0.0):
    from krowd_stanley_control.disagreement_hold import DisagreementHold

    node, now_ns = harness(vehicle_yaw=vehicle_yaw, map_yaw=map_yaw)
    node._map_heading_hold = DisagreementHold(1.0, math.radians(45.0))
    node._map_heading_hold_warn_ns = 0
    node.get_logger = lambda: SimpleNamespace(warning=lambda *_: None)
    return node, now_ns


def fresh(node, t_ns):
    node._map_heading_pose_ns = node._map_heading_verified_ns = t_ns
    node._heading_verified_ns = node._heading_source_ns = t_ns
    return t_ns


def test_short_disagreement_keeps_driving():
    node, now_ns = hold_harness(vehicle_yaw=20.0)
    assert StanleyNode._map_heading_consistency_problem(node, now_ns) is None
    assert StanleyNode._map_heading_consistency_problem(node, fresh(node, now_ns + 900_000_000)) is None


def test_disagreement_lasting_one_second_stops():
    node, now_ns = hold_harness(vehicle_yaw=20.0)
    StanleyNode._map_heading_consistency_problem(node, now_ns)
    reason = StanleyNode._map_heading_consistency_problem(node, fresh(node, now_ns + 1_000_000_000))
    assert reason.startswith("HEADING_DISAGREEMENT") and "for 1.0s" in reason


def test_agreement_in_between_restarts_the_hold():
    node, now_ns = hold_harness(vehicle_yaw=20.0)
    StanleyNode._map_heading_consistency_problem(node, now_ns)
    node._vehicle_state.odometry = pose(5.0)
    assert StanleyNode._map_heading_consistency_problem(node, fresh(node, now_ns + 500_000_000)) is None
    node._vehicle_state.odometry = pose(20.0)
    assert StanleyNode._map_heading_consistency_problem(node, fresh(node, now_ns + 600_000_000)) is None
    assert StanleyNode._map_heading_consistency_problem(node, fresh(node, now_ns + 1_500_000_000)) is None


def test_gross_disagreement_stops_at_once():
    node, now_ns = hold_harness(vehicle_yaw=124.0)
    assert StanleyNode._map_heading_consistency_problem(node, now_ns).startswith(
        "HEADING_DISAGREEMENT"
    )
