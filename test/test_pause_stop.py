import math

import pytest

from krowd_stanley_control.pause_stop import (
    APPROACH, DRIVE, HOLDING, ROLLING, PauseStop, read_pause_zones,
)
from krowd_stanley_control.signal_stop import SignalZone, zones_from_json, zones_to_json

ZONE = SignalZone("110-114", "pause", 137.0, 142.0)
V7 = 7 / 3.6
BD7 = V7 * 1.0 + V7 ** 2 / (2 * 3.95)       # braking distance at 7 km/h
BUMPER = 1.194                               # front axle -> front bumper
BASE_TO_BUMPER = 1.34 + 0.0 + BUMPER


def rule(hold=3.0):
    p = PauseStop(zone_speed_mps=V7, approach_decel_mps2=0.5, stop_speed_mps=0.3, hold_s=hold)
    p.set_zones([ZONE])
    return p


def step(p, now, s, v, line=None, **kw):
    return p.step(now, s, v, line, BASE_TO_BUMPER, BUMPER, 1.0, **kw)


def test_approach_cap_reaches_zone_speed_at_the_start():
    p = rule()
    v = step(p, 0.0, 100.0, 5.0)
    assert v.state == APPROACH and v.zone == ZONE
    assert v.speed_cap_mps == pytest.approx(math.sqrt(V7 ** 2 + 2 * 0.5 * 37.0))
    assert step(p, 0.0, 136.9, 2.0).speed_cap_mps == pytest.approx(V7, abs=0.03)


def test_rolls_at_zone_speed_then_brakes_before_the_zone_end_without_a_line():
    p = rule()
    v = step(p, 0.0, 137.5, V7)
    assert v.state == ROLLING and v.speed_cap_mps == pytest.approx(V7) and not v.brake
    assert v.room_m == pytest.approx(142.0 - 137.5 - BUMPER)
    # brake when the bumper is one braking distance from the zone end
    s_brake = 142.0 - BUMPER - BD7
    assert step(p, 1.0, s_brake - 0.05, V7).state == ROLLING
    v = step(p, 1.1, s_brake + 0.05, V7)
    assert v.state == HOLDING and v.brake and "zone end" in v.reason


def test_a_detected_stop_line_nearer_than_the_zone_end_wins():
    p = rule()
    # line 3.0 m ahead of base_link -> bumper room = 3.0 - 2.534 - 1.0 < 0 -> brake now
    v = step(p, 0.0, 137.5, V7, line=3.0)
    assert v.state == HOLDING and "stop line" in v.reason
    p = rule()
    # line far away: the zone end limits
    v = step(p, 0.0, 137.5, V7, line=30.0)
    assert v.state == ROLLING and v.room_m == pytest.approx(142.0 - 137.5 - BUMPER)


def test_holds_until_still_for_the_hold_time_then_releases_and_latches():
    p = rule()
    step(p, 0.0, 141.0, V7)                       # brake point reached
    assert p.state == HOLDING
    assert step(p, 1.0, 141.5, 0.8).brake         # still slowing: timer not running
    v = step(p, 2.0, 141.6, 0.1)
    assert v.brake and v.held_s == pytest.approx(0.0)
    assert step(p, 4.9, 141.6, 0.1).brake         # 2.9 s standing
    v = step(p, 5.05, 141.6, 0.1)
    assert v.released and not v.brake and v.state == DRIVE and v.zone == ZONE
    # latched: driving on through the same zone does not stop again
    assert step(p, 6.0, 141.8, 1.0).state == DRIVE
    assert step(p, 7.0, 150.0, 2.0).state == DRIVE
    # next lap, well before the start: active again
    assert step(p, 8.0, 100.0, 2.0, route_length=200.0, closed_loop=True).state == APPROACH


def test_creeping_resets_the_still_timer():
    p = rule()
    step(p, 0.0, 141.0, V7)
    step(p, 1.0, 141.5, 0.1)
    step(p, 2.5, 141.5, 0.1)
    assert step(p, 3.0, 141.7, 0.5).held_s == 0.0   # rolled again
    assert step(p, 3.5, 141.7, 0.1).brake            # timer restarts here
    assert step(p, 6.4, 141.7, 0.1).brake
    assert step(p, 6.6, 141.7, 0.1).released


def test_pause_zones_round_trip_json_and_file(tmp_path):
    text = zones_to_json("run", [ZONE])
    assert zones_from_json(text) == ("run", [ZONE])
    assert read_pause_zones(tmp_path) == []
    (tmp_path / "pause_zones.yaml").write_text(
        "stanley_node:\n  ros__parameters:\n    pause_zones: ['169-175', '110-114']\n"
    )
    assert read_pause_zones(tmp_path) == [(110, 114, "pause"), (169, 175, "pause")]
    (tmp_path / "pause_zones.yaml").write_text(
        "stanley_node:\n  ros__parameters:\n    pause_zones: ['175-169']\n"
    )
    with pytest.raises(ValueError):
        read_pause_zones(tmp_path)
