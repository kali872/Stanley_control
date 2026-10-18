import pytest

from krowd_stanley_control.speed_trim import SpeedTrim

KMH = 1.0 / 3.6


def _trim():
    return SpeedTrim(gain_per_s=0.3, max_trim_mps=5.0 * KMH, min_speed_mps=1.0 * KMH)


def test_a_vehicle_at_its_target_gets_the_target():
    trim = _trim()
    for _ in range(200):
        assert trim.command(5.0 * KMH, 5.0 * KMH, 0.05) == pytest.approx(5.0 * KMH)


def test_a_slow_vehicle_gets_a_growing_but_bounded_command():
    trim = _trim()
    first = trim.command(5.0 * KMH, 3.8 * KMH, 0.05)
    for _ in range(100):
        later = trim.command(5.0 * KMH, 3.8 * KMH, 0.05)
    assert 5.0 * KMH < first < later
    for _ in range(100000):
        last = trim.command(5.0 * KMH, 3.8 * KMH, 0.05)
    assert last == pytest.approx(10.0 * KMH)


def test_waiting_at_a_standstill_does_not_wind_up():
    trim = _trim()
    for _ in range(2000):  # 100 s waiting for the ASMS
        command = trim.command(5.0 * KMH, 0.0, 0.05)
    assert command == pytest.approx(5.0 * KMH) and trim.trim_mps == 0.0


def test_overspeed_takes_the_trim_back_but_never_below_the_target():
    trim = _trim()
    for _ in range(400):
        trim.command(5.0 * KMH, 3.0 * KMH, 0.05)
    assert trim.trim_mps > 0.0
    for _ in range(4000):
        command = trim.command(5.0 * KMH, 8.0 * KMH, 0.05)
    assert command == pytest.approx(5.0 * KMH)


def test_a_zero_target_and_a_reset_clear_the_trim():
    trim = _trim()
    for _ in range(400):
        trim.command(5.0 * KMH, 3.0 * KMH, 0.05)
    assert trim.command(0.0, 3.0 * KMH, 0.05) == 0.0 and trim.trim_mps == 0.0
    for _ in range(400):
        trim.command(5.0 * KMH, 3.0 * KMH, 0.05)
    trim.reset()
    assert trim.trim_mps == 0.0


def test_disabled_passes_the_target_through():
    trim = SpeedTrim(0.0, 0.0, 0.0)
    assert not trim.enabled
    assert trim.command(5.0 * KMH, 1.0 * KMH, 0.05) == pytest.approx(5.0 * KMH)


def test_negative_settings_are_refused():
    with pytest.raises(ValueError):
        SpeedTrim(-0.1, 1.0, 0.0)


def test_hold_applies_no_trim_and_keeps_it():
    trim = SpeedTrim(1.0, 2.0, 0.1)
    trim.command(3.0, 2.0, 1.0)
    assert trim.trim_mps == 1.0
    assert trim.command(3.0, 2.0, 1.0, hold=True) == 3.0
    assert trim.trim_mps == 1.0
    assert trim.command(3.0, 3.0, 1.0) == 4.0
