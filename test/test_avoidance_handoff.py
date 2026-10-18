import math

import pytest

from krowd_stanley_control.avoidance_handoff import (
    APPROACH, DRIVE, HANDED_OFF, STOPPING, AvoidanceHandoff, read_avoidance_zones,
)
from krowd_stanley_control.signal_stop import SignalZone, zones_from_json, zones_to_json

ZONE = SignalZone("107-135", "avoid", 128.0, 163.0)


def rule():
    h = AvoidanceHandoff(approach_speed_mps=5 / 3.6, approach_decel_mps2=0.5,
                         stop_speed_mps=0.3, stop_settle_s=1.0,
                         heading_tolerance_rad=math.radians(5), align_timeout_s=10.0)
    h.set_zones([ZONE])
    return h


BD5 = (5 / 3.6) * 1.0 + (5 / 3.6) ** 2 / (2 * 3.95)   # braking distance at 5 km/h


def test_approach_cap_holds_the_approach_speed_until_the_brake_point():
    h = rule()
    far = h.step(0.0, 100.0, 5.0, 0.0)
    assert far.state == APPROACH and far.zone == ZONE
    assert far.speed_cap_mps == pytest.approx(math.sqrt(2 * 0.5 * (28.0 - BD5)))
    assert h.step(0.0, 50.0, 5.0, 0.0).speed_cap_mps > far.speed_cap_mps
    # close to the brake point the cap floors at the approach speed
    assert h.step(0.0, 128.0 - BD5 - 0.5, 1.4, 0.0).speed_cap_mps == pytest.approx(5 / 3.6)


def test_brakes_when_the_braking_distance_reaches_the_zone_start():
    h = rule()
    v = 5 / 3.6
    assert h.step(0.0, 128.0 - BD5 - 0.05, v, 0.0).state == APPROACH
    assert h.step(0.1, 128.0 - BD5 + 0.05, v, 0.0).state == STOPPING
    # faster -> earlier
    h = rule()
    bd = 3.0 * 1.0 + 9.0 / (2 * 3.95)
    assert h.step(0.0, 128.0 - bd + 0.05, 3.0, 0.0).state == STOPPING
    # standing before the start: no brake, keeps rolling at the approach speed
    h = rule()
    v = h.step(0.0, 127.0, 0.0, 0.0)
    assert v.state == APPROACH and v.speed_cap_mps == pytest.approx(5 / 3.6)


def test_already_inside_the_zone_stops_immediately():
    h = rule()
    assert h.step(0.0, 130.0, 2.0, 0.0).state == STOPPING


def test_stops_at_the_zone_start_then_hands_off_when_still_and_aligned():
    h = rule()
    v = h.step(10.0, 128.5, 1.4, 0.0)
    assert v.state == STOPPING and v.brake
    assert h.step(10.5, 128.9, 0.1, 0.0).state == STOPPING   # not still for 1 s yet
    assert h.step(11.6, 128.9, 0.1, math.radians(20)).state == STOPPING  # misaligned
    done = h.step(11.7, 128.9, 0.1, math.radians(2))
    assert done.state == HANDED_OFF and not done.brake and h.last_note == ""


def test_misaligned_vehicle_is_handed_off_after_the_timeout_with_a_note():
    h = rule()
    h.step(0.0, 128.5, 0.0, math.radians(20))
    assert h.step(9.0, 128.5, 0.0, math.radians(20)).state == STOPPING
    assert h.step(10.5, 128.5, 0.0, math.radians(20)).state == HANDED_OFF
    assert "handing off anyway" in h.last_note


def test_handed_off_reports_past_end_and_resume_latches_the_zone():
    h = rule()
    h.step(0.0, 128.5, 0.0, 0.0)
    assert h.step(2.0, 128.5, 0.0, 0.0).state == HANDED_OFF
    assert h.step(3.0, 150.0, 2.0, 0.0).past_end is False
    assert h.step(4.0, 164.0, 2.0, 0.0).past_end is True
    h.resume()
    # still inside / just past the zone: no second hand-off
    assert h.step(5.0, 150.0, 2.0, 0.0).state == DRIVE
    assert h.step(6.0, 170.0, 2.0, 0.0).state == DRIVE
    # next lap, well before the start again: unlatched
    v = h.step(7.0, 100.0, 2.0, 0.0, route_length=200.0, closed_loop=True)
    assert v.state == APPROACH


def test_loop_distance_wraps():
    h = rule()
    v = h.step(0.0, 190.0, 2.0, 0.0, route_length=200.0, closed_loop=True)
    assert v.state == APPROACH
    assert v.speed_cap_mps == pytest.approx(math.sqrt(2 * 0.5 * (138.0 - BD5)))


def test_avoid_zones_round_trip_json_and_file(tmp_path):
    text = zones_to_json("run", [ZONE])
    assert zones_from_json(text) == ("run", [ZONE])
    (tmp_path / "avoidance_zones.yaml").write_text(
        'localization_monitor_node:\n  ros__parameters:\n    avoidance_zones: ["107-135"]\n'
    )
    assert read_avoidance_zones(tmp_path) == [(107, 135, "avoid")]
    assert read_avoidance_zones(tmp_path / "missing") == []
