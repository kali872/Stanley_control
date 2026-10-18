import math

import pytest

from krowd_stanley_control.recovery import Recovery, pursuit_steering, to_body_frame

S = 1_000_000_000


def _r():
    return Recovery(done_offset_m=1.5, done_heading_rad=math.radians(30), max_offset_m=8.0,
                    max_duration_s=20.0, max_jump_m=1.0)


def test_on_track_is_off_and_leaving_the_corridor_starts_recovery():
    r = _r()
    assert r.step(0, 0.2, 0.0, True, True) == Recovery.OFF
    # Drifting out over a few cycles, as a vehicle does (a jump is a pose glitch).
    for i, offset in enumerate((-0.6, -1.4, -2.2, -2.9)):
        assert r.step((i + 1) * S, offset, -0.2, True, True) == Recovery.OFF
    assert r.step(5 * S, -3.1, -0.2, False, True) == Recovery.RECOVERING
    assert r.active


def test_done_needs_both_offset_and_heading_and_resumes_tracking():
    r = _r()
    r.step(0, -2.5, 0.0, True, True)
    r.step(0, -3.1, 0.0, False, True)
    # Back inside the corridor but still crossing at 50 deg: keep recovering.
    assert r.step(2 * S, -1.0, math.radians(50), True, True) == Recovery.RECOVERING
    assert r.step(3 * S, -1.0, math.radians(20), True, True) == Recovery.DONE
    assert r.step(4 * S, -0.9, 0.1, True, True) == Recovery.OFF


def test_failure_on_too_far_too_long_or_target_behind_and_it_latches():
    for kwargs in ({"signed_offset_m": -8.5}, {"now_ns": 25 * S}, {"target_ahead": False}):
        r = _r()
        r.step(0, -2.5, 0.0, True, True)
        r.step(0, -3.1, 0.0, False, True)
        args = dict(now_ns=1 * S, signed_offset_m=-3.2, heading_error_rad=0.0, on_track=False, target_ahead=True)
        args.update(kwargs)
        assert r.step(**args) == Recovery.FAILED
        assert r.step(2 * S, -3.2, 0.0, False, True) == Recovery.FAILED  # stays failed off track
        assert r.step(3 * S, -0.2, 0.0, True, True) == Recovery.OFF  # cleared once back on track


def test_an_offset_that_jumps_is_not_recovered_from():
    r = _r()
    r.step(0, 0.1, 0.0, True, True)
    assert r.step(50_000_000, 3.4, 0.0, False, True) == Recovery.FAILED


def test_pursuit_steering_turns_toward_the_target():
    assert pursuit_steering(5.0, 2.0, 1.34) > 0.0
    assert pursuit_steering(5.0, -2.0, 1.34) < 0.0
    assert pursuit_steering(5.0, 0.0, 1.34) == 0.0
    assert pursuit_steering(0.0, 0.0, 1.34) == 0.0


def test_to_body_frame_puts_a_point_ahead_on_positive_x():
    bx, by = to_body_frame(10.0, 5.0, math.pi / 2, 10.0, 8.0)
    assert bx == pytest.approx(3.0) and by == pytest.approx(0.0)
    bx, by = to_body_frame(0.0, 0.0, 0.0, 2.0, 1.0)
    assert (bx, by) == pytest.approx((2.0, 1.0))


def test_forget_offset_lets_recovery_start_after_a_hand_off():
    from krowd_stanley_control.recovery import Recovery

    r = Recovery(done_offset_m=1.5, done_heading_rad=0.5, max_offset_m=8.0, max_duration_s=20.0, max_jump_m=1.0)
    assert r.step(0, 0.1, 0.0, True, True) == Recovery.OFF
    # without forgetting, a 5 m change counts as a jump -> FAILED
    r.forget_offset()
    assert r.step(1_000_000_000, -5.2, 0.4, False, True) == Recovery.RECOVERING


def test_done_needs_on_track_and_before_start_holds_the_clock():
    from krowd_stanley_control.recovery import Recovery
    r = Recovery(done_offset_m=1.5, done_heading_rad=0.52, max_offset_m=8.0, max_duration_s=20.0, max_jump_m=1.0)
    # before the route start: offset/heading fine but not on track -> stays RECOVERING, no flip-flop
    assert r.step(0, -0.7, -0.1, False, True, hold_timer=True) == Recovery.RECOVERING
    for i in range(1, 60):
        assert r.step(i * 10**9, -0.7, -0.1, False, True, hold_timer=True) == Recovery.RECOVERING
    assert r.elapsed_s(59 * 10**9) == 0.0          # clock held while before the start
    # now on the route and within the done thresholds -> DONE
    assert r.step(60 * 10**9, -0.7, -0.1, True, True) == Recovery.DONE
