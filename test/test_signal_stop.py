import json
import math

import pytest

from krowd_stanley_control.signal_stop import (
    LEFT,
    STRAIGHT,
    SignalStop,
    SignalTracker,
    SignalZone,
    parse_frame,
    read_map_zones,
    zones_from_json,
    zones_on_route,
    zones_to_json,
)


def frame(state="red", go_s=False, go_l=False, conf_s=False, conf_l=False, t=0.0):
    return parse_frame(json.dumps({
        "stamp": {"sec": int(t), "nanosec": 0}, "state": state,
        "go_straight": go_s, "go_left": go_l,
        "go_straight_confirmed": conf_s, "go_left_confirmed": conf_l,
    }))


def test_parse_rejects_garbage():
    with pytest.raises(ValueError):
        parse_frame("{}")
    with pytest.raises(ValueError):
        parse_frame("not json")


def test_red_needs_consecutive_fresh_frames():
    tracker = SignalTracker(stop_frames=2)
    tracker.update(frame("red"), 10.0)
    assert not tracker.red(STRAIGHT, 10.0, 0.5)
    tracker.update(frame("red"), 10.05)
    assert tracker.red(STRAIGHT, 10.1, 0.5)
    assert not tracker.red(STRAIGHT, 10.7, 0.5)  # detector went quiet
    tracker.update(frame("unknown"), 10.8)
    assert not tracker.red(STRAIGHT, 10.8, 0.5)  # run broken


def test_direction_specific_stop_and_go():
    tracker = SignalTracker(stop_frames=1)
    # 3-lamp red + left arrow reads "green": go for left, stop for straight
    tracker.update(frame("green", go_s=False, go_l=True, conf_l=True), 1.0)
    assert tracker.red(STRAIGHT, 1.0, 0.5) and not tracker.red(LEFT, 1.0, 0.5)
    assert tracker.go(LEFT, 1.0, 0.25) and not tracker.go(STRAIGHT, 1.0, 0.25)
    assert not tracker.go(LEFT, 1.3, 0.25)  # confirmed go too old


ZONE = SignalZone("64-68", STRAIGHT, 70.0, 75.0)


def rule():
    stop = SignalStop(zone_speed_mps=7 / 3.6, approach_decel_mps2=0.5)
    stop.set_zones([ZONE, SignalZone("238-242", LEFT, 280.0, 285.0)])
    return stop


def test_unknown_light_passes_at_zone_speed():
    verdict = rule().step(72.0, lambda d: False, lambda d: False)
    assert not verdict.stop and verdict.zone == ZONE
    assert verdict.speed_cap_mps == pytest.approx(7 / 3.6)


def test_red_in_zone_stops_and_latches_until_confirmed_go():
    stop = rule()
    assert stop.step(72.0, lambda d: True, lambda d: False).stop
    # rolled past the zone end, light now unknown: still stopped
    after = stop.step(76.0, lambda d: False, lambda d: False)
    assert after.stop and after.zone == ZONE
    released = stop.step(76.0, lambda d: False, lambda d: d == STRAIGHT)
    assert not released.stop


def test_go_for_the_other_direction_does_not_release():
    stop = rule()
    stop.step(72.0, lambda d: True, lambda d: False)
    assert stop.step(72.0, lambda d: False, lambda d: d == LEFT).stop


def test_red_outside_a_zone_does_nothing():
    verdict = rule().step(40.0, lambda d: True, lambda d: False)
    assert not verdict.stop and verdict.zone is None


def test_approach_cap_reaches_zone_speed_at_the_zone():
    stop = rule()
    far = stop.step(40.0, lambda d: False, lambda d: False)   # 30 m before
    near = stop.step(69.0, lambda d: False, lambda d: False)  # 1 m before
    assert far.speed_cap_mps == pytest.approx(math.sqrt((7 / 3.6) ** 2 + 2 * 0.5 * 30))
    assert near.speed_cap_mps == pytest.approx(math.sqrt((7 / 3.6) ** 2 + 1.0))
    assert near.approaching == ZONE
    # past the last zone on an open route: no cap; on a loop: the first zone again
    assert stop.step(300.0, lambda d: False, lambda d: False).speed_cap_mps is None
    looped = stop.step(300.0, lambda d: False, lambda d: False, route_length=320.0, closed_loop=True)
    assert looped.approaching == ZONE


def test_zone_files_and_resolution_onto_a_route(tmp_path):
    (tmp_path / "traffic_detect_zones.yaml").write_text(
        'localization_monitor_node:\n  ros__parameters:\n    traffic_detect_zones: ["3-5", "8-9"]\n'
    )
    (tmp_path / "left_turn_zones.yaml").write_text(
        'mission_manager_node:\n  ros__parameters:\n    left_turn_zones: ["8-9"]\n'
    )
    specs = read_map_zones(tmp_path)
    assert specs == [(3, 5, STRAIGHT), (8, 9, LEFT)]
    waypoints = [(float(i), 0.0) for i in range(12)]
    # published route dropped waypoint 1 and starts at 2 m spacing there
    route = [waypoints[0]] + waypoints[2:]
    zones, problems = zones_on_route(specs, waypoints, route)
    assert not problems
    assert [(z.label, z.direction, z.start_s, z.end_s) for z in zones] == [
        ("3-5", STRAIGHT, 3.0, 5.0), ("8-9", LEFT, 8.0, 9.0),
    ]
    trajectory_id, back = zones_from_json(zones_to_json("run:11wp", zones))
    assert trajectory_id == "run:11wp" and back == zones


def test_zone_off_the_route_is_reported_not_enforced():
    waypoints = [(float(i), 0.0) for i in range(10)]
    zones, problems = zones_on_route([(7, 9, STRAIGHT), (2, 20, STRAIGHT)], waypoints, waypoints[:6])
    assert zones == [] and len(problems) == 2


def test_map_without_zone_files_has_no_zones(tmp_path):
    assert read_map_zones(tmp_path) == []


def test_left_turn_zone_must_be_a_traffic_zone(tmp_path):
    (tmp_path / "traffic_detect_zones.yaml").write_text(
        'localization_monitor_node:\n  ros__parameters:\n    traffic_detect_zones: ["3-5"]\n'
    )
    (tmp_path / "left_turn_zones.yaml").write_text(
        'mission_manager_node:\n  ros__parameters:\n    left_turn_zones: ["8-9"]\n'
    )
    with pytest.raises(ValueError):
        read_map_zones(tmp_path)


# ---- stop line -------------------------------------------------------------
from types import SimpleNamespace  # noqa: E402

from krowd_stanley_control.signal_stop import brake_distance, red_stop_decision  # noqa: E402
from krowd_stanley_control.stopline import StoplineEstimator  # noqa: E402

V = 7 / 3.6
BASE_TO_BUMPER = 1.34 + 1.194
KW = dict(base_to_bumper_m=BASE_TO_BUMPER, front_axle_to_bumper_m=1.194,
          stop_buffer_m=1.0, latency_s=1.0, decel_mps2=3.95)


def test_brake_distance_matches_the_final_control_model():
    assert brake_distance(V, 1.0, 3.95) == pytest.approx(V + V * V / 7.9)
    assert brake_distance(-1.0, 1.0, 3.95) == 0.0


def test_red_without_a_stop_line_brakes_where_it_is():
    decision = red_stop_decision(V, 100.0, 106.0, None, **KW)
    assert decision.brake and decision.room_m is None


def test_red_with_a_far_stop_line_rolls_on_then_brakes_before_it():
    need = brake_distance(V, 1.0, 3.95)
    far = red_stop_decision(V, 100.0, 120.0, 12.0, **KW)
    assert not far.brake and far.room_m == pytest.approx(12.0 - BASE_TO_BUMPER - 1.0)
    close = red_stop_decision(V, 105.0, 120.0, BASE_TO_BUMPER + 1.0 + need - 0.01, **KW)
    assert close.brake and "stop line" in close.reason


def test_a_line_beyond_the_zone_does_not_carry_the_red_past_the_zone_end():
    need = brake_distance(V, 1.0, 3.95)
    # the "line" is 25 m away (a crosswalk past the junction), the zone ends in 3 m
    decision = red_stop_decision(V, 100.0, 100.0 + 1.194 + need - 0.1, 25.0, **KW)
    assert decision.brake and "zone end" in decision.reason


def _line(x, y_min=-1.5, y_max=1.5):
    return SimpleNamespace(action=0, type=4, id=1,
                           points=[SimpleNamespace(x=x, y=y_min), SimpleNamespace(x=x, y=y_max)])


def test_stopline_estimate_is_carried_by_distance_travelled_until_the_ttl():
    estimator = StoplineEstimator(half_band_m=0.63, min_span_m=1.0, max_abs_slope=0.5, max_distance_m=30.0)
    assert estimator.ingest([_line(8.0)], 1_000_000_000, 50.0) is not None
    fresh = estimator.estimate(1_500_000_000, 2.0, 51.5)
    assert fresh.valid and fresh.distance_m == pytest.approx(6.5)
    assert not estimator.estimate(3_500_000_000, 2.0, 53.0).valid          # older than the TTL
    assert not estimator.estimate(1_500_000_000, None, 51.5).valid         # disabled


def test_stopline_rejects_lines_outside_the_lane_or_not_transverse():
    estimator = StoplineEstimator(half_band_m=0.63, min_span_m=1.0, max_abs_slope=0.5, max_distance_m=30.0)
    assert estimator.ingest([_line(8.0, 2.0, 4.0)], 1, None) is None       # another lane
    assert estimator.ingest([_line(40.0)], 1, None) is None                # too far
    slanted = SimpleNamespace(action=0, type=4, id=2,
                              points=[SimpleNamespace(x=6.0, y=-1.0), SimpleNamespace(x=9.0, y=1.0)])
    assert estimator.ingest([slanted], 1, None) is None
    assert estimator.ingest([_line(12.0), _line(7.0)], 1, None).distance_m == pytest.approx(7.0)


def test_a_lost_light_releases_the_red_stop_after_the_timeout():
    stop = rule()
    assert stop.step(72.0, lambda d: True, lambda d: False, lost=lambda d: False).stop
    assert stop.step(72.0, lambda d: False, lambda d: False, lost=lambda d: False).stop
    released = stop.step(72.0, lambda d: False, lambda d: False, lost=lambda d: d == STRAIGHT)
    assert not released.stop and stop.last_release == "lost"


def test_tracker_measures_time_since_the_last_stop_frame():
    from krowd_stanley_control.signal_stop import SignalTracker
    tr = SignalTracker(stop_frames=2)
    assert tr.since_stop_s(STRAIGHT, 5.0) is None
    tr.update(frame("red"), 10.0)
    tr.update(frame("red"), 10.1)
    tr.update(frame("unknown"), 10.2)
    assert tr.since_stop_s(STRAIGHT, 40.1) == pytest.approx(30.0)
    assert tr.stop_run(STRAIGHT) == 0


def test_go_read_on_two_frames_within_the_window_counts_as_a_go():
    from krowd_stanley_control.signal_stop import SignalTracker
    tr = SignalTracker(stop_frames=2)
    tr.update(frame("red"), 10.0)
    tr.update(frame("green", go_s=True), 11.0)
    tr.update(frame("unknown"), 11.2)
    assert not tr.go_seen(STRAIGHT, 11.3, 1.5, 2)          # one frame so far
    tr.update(frame("green", go_s=True), 11.6)
    assert tr.go_seen(STRAIGHT, 11.7, 1.5, 2)
    assert not tr.go_seen(STRAIGHT, 11.7, 1.5, 0)          # disabled
    assert not tr.go_seen(LEFT, 11.7, 1.5, 2)              # other direction
    # frames too far apart do not add up
    tr2 = SignalTracker(stop_frames=2)
    tr2.update(frame("green", go_s=True), 20.0)
    tr2.update(frame("green", go_s=True), 22.0)
    assert not tr2.go_seen(STRAIGHT, 22.1, 1.5, 2)
    # a red in between clears the count
    tr3 = SignalTracker(stop_frames=2)
    tr3.update(frame("green", go_s=True), 30.0)
    tr3.update(frame("red"), 30.3)
    tr3.update(frame("green", go_s=True), 30.6)
    assert not tr3.go_seen(STRAIGHT, 30.7, 1.5, 2)


def test_red_without_a_line_can_stop_at_the_zone_end_instead():
    from krowd_stanley_control.signal_stop import red_stop_decision
    v = 7 / 3.6
    need = v * 1.0 + v * v / (2 * 3.95)
    # far from the zone end: roll on at the zone speed
    d = red_stop_decision(v, 60.0, 75.0, None, 2.534, 1.194, 1.0, 1.0, 3.95, stop_at_zone_end=True)
    assert not d.brake and "zone end" in d.reason and d.room_m == pytest.approx(75.0 - 61.194)
    # one braking distance before the end: brake
    s = 75.0 - 1.194 - need + 0.05
    d = red_stop_decision(v, s, 75.0, None, 2.534, 1.194, 1.0, 1.0, 3.95, stop_at_zone_end=True)
    assert d.brake and d.reason == "at the zone end"
    # the old rule is still the default
    assert red_stop_decision(v, 60.0, 75.0, None, 2.534, 1.194, 1.0, 1.0, 3.95).brake
    # a nearer line still wins
    d = red_stop_decision(v, 60.0, 75.0, 4.0, 2.534, 1.194, 1.0, 1.0, 3.95, stop_at_zone_end=True)
    assert d.brake and d.reason == "at the stop line"
