from krowd_stanley_control.external_motion import ExternalMotion


def test_moving_against_the_brake_for_the_hold_time_is_external():
    e = ExternalMotion(speed_mps=1.0, hold_s=1.0)
    assert e.step(0.0, True, 0.0) is False
    assert e.step(1.0, True, 3.0) is False      # just started moving
    assert e.step(1.9, True, 3.0) is False
    assert e.step(2.0, True, 3.0) is True
    assert e.step(2.5, True, 0.2) is False      # stood still again
    assert e.step(3.0, False, 5.0) is False     # we command the motion ourselves
    assert e.step(4.5, False, 5.0) is False
    assert e.step(5.0, None, 5.0) is False      # nothing commanded yet
