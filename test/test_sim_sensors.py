import math
import statistics

import pytest

from krowd_stanley_control.sim_sensors import (
    EstimationNoise,
    GaussMarkov,
    RtkCourseVerifier,
    SimulatedEncoder,
    prior_heading_pose,
)


def test_zero_noise_is_exact():
    noise = EstimationNoise()
    for _ in range(10):
        s = noise.sample(0.02)
        assert (s.dx, s.dy, s.dyaw, s.dyaw_rate) == (0.0, 0.0, 0.0, 0.0)


def test_white_noise_has_the_requested_sigma():
    noise = EstimationNoise(position_sigma_m=0.006, yaw_sigma_rad=0.0025, seed=3)
    samples = [noise.sample(0.02) for _ in range(20000)]
    assert statistics.pstdev(s.dx for s in samples) == pytest.approx(0.006, rel=0.05)
    assert statistics.pstdev(s.dyaw for s in samples) == pytest.approx(0.0025, rel=0.05)


def test_noise_is_reproducible_by_seed():
    a = [EstimationNoise(0.01, seed=7).sample(0.02).dx for _ in range(1)]
    b = [EstimationNoise(0.01, seed=7).sample(0.02).dx for _ in range(1)]
    assert a == b


def test_gauss_markov_is_slow_and_stationary():
    import random

    drift = GaussMarkov(0.18, 10.0, random.Random(1))
    values = [drift.step(0.02) for _ in range(200000)]
    assert statistics.pstdev(values) == pytest.approx(0.18, rel=0.2)
    # correlated: a 20 ms step moves sigma * sqrt(2 dt / tau) = 1.1 cm, not 18
    steps = [b - a for a, b in zip(values, values[1:])]
    assert statistics.pstdev(steps) == pytest.approx(0.18 * math.sqrt(0.004), rel=0.05)


def test_encoder_quantises():
    encoder = SimulatedEncoder(resolution_rad=0.01)
    assert encoder.read(0.1234) == pytest.approx(0.12)
    assert encoder.read(-0.2151) == pytest.approx(-0.22)
    assert SimulatedEncoder().read(0.1234) == 0.1234


def test_course_verifies_after_three_straight_fixes():
    course = RtkCourseVerifier()
    assert not course.update(0.0, 1.5, 0.0)
    assert not course.update(0.2, 1.5, 0.0)
    assert course.update(0.4, 1.5, 0.0)
    assert course.update(0.6, 0.0, 1.0)  # latched


def test_course_needs_speed_straightness_and_no_gap():
    course = RtkCourseVerifier()
    course.update(0.0, 1.5, 0.0)
    course.update(0.2, 1.5, 0.0)
    assert not course.update(0.4, 0.9, 0.0)  # too slow: restart
    course.update(0.6, 1.5, 0.0)
    course.update(0.8, 1.5, 0.0)
    assert not course.update(1.0, 1.5, 0.2)  # turning: restart
    course.update(1.2, 1.5, 0.0)
    course.update(1.4, 1.5, 0.0)
    assert not course.update(2.6, 1.5, 0.0)  # 1.2 s gap: restart
    assert not course.update(2.8, 1.5, 0.0)
    assert course.update(3.0, 1.5, 0.0)


def test_prior_heading_keeps_the_antenna_and_moves_base_link():
    x, y, yaw = prior_heading_pose(10.0, 5.0, 0.0, math.radians(5.0), 2.5038)
    assert yaw == pytest.approx(math.radians(5.0))
    # antenna (12.5038, 5.0) is where it was
    assert x + 2.5038 * math.cos(yaw) == pytest.approx(12.5038)
    assert y + 2.5038 * math.sin(yaw) == pytest.approx(5.0)
    # base_link moved 2 L sin(err/2) = 0.22 m, mostly sideways (right for +err)
    assert math.hypot(x - 10.0, y - 5.0) == pytest.approx(0.2185, abs=1e-3)
    assert y < 5.0


def test_zero_prior_error_is_the_truth():
    assert prior_heading_pose(1.0, 2.0, 0.3, 0.0, 2.5) == pytest.approx((1.0, 2.0, 0.3))
