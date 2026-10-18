import math

import pytest

from krowd_stanley_control.tracking_common import clamp_steering, rate_limited
from krowd_stanley_control.vehicle import (
    SteeringActuator,
    Vehicle,
    implausible_steering_limit_reason,
    speed_command_kmh,
    steering_limit_from_turning_radius,
)


WHEELBASE = 1.340


def test_speed_command_is_km_h_for_the_drive_mcu():
    # ros_can_bridge sends /target_speed x 100 on CAN 0x110 and the drive MCU
    # reads it as km/h x 100: a 15 km/h route speed (4.1667 m/s) must go out
    # as 15.0, not 4.17 (which the MCU would drive as 4.17 km/h).
    assert speed_command_kmh(15.0 / 3.6) == pytest.approx(15.0)
    assert speed_command_kmh(0.0) == 0.0
    assert speed_command_kmh(-1.0) == 0.0


def test_rate_limit_caps_a_step_command():
    limited = rate_limited(0.0, 1.0, math.radians(60.0), 0.05)
    assert limited == pytest.approx(math.radians(3.0))


def test_rate_limit_passes_a_small_change_through():
    assert rate_limited(0.1, 0.11, math.radians(60.0), 0.05) == pytest.approx(0.11)


def test_rate_limit_is_symmetric():
    assert rate_limited(0.0, -1.0, math.radians(60.0), 0.05) == pytest.approx(
        -math.radians(3.0)
    )


def test_clamp_steering_bounds_both_directions():
    assert clamp_steering(1.0, 0.4) == pytest.approx(0.4)
    assert clamp_steering(-1.0, 0.4) == pytest.approx(-0.4)
    assert clamp_steering(0.2, 0.4) == pytest.approx(0.2)


def test_steering_limit_matches_the_measured_turning_circle():
    # wheelbase 1.340 m, 5.15 m turning-circle diameter by hand (2026-09-19).
    limit = steering_limit_from_turning_radius(1.340, 2.575)
    assert math.degrees(limit) == pytest.approx(27.5, abs=0.1)


def test_steering_command_is_the_road_wheel_angle_times_the_measured_gain():
    actuator = SteeringActuator(gain=8.0, offset_rad=0.0, max_command_rad=math.radians(99.0))
    actuator.validate()
    assert actuator.command(math.radians(5.0)) == pytest.approx(math.radians(40.0))
    assert actuator.command(math.radians(5.0), sign=-1.0) == pytest.approx(-math.radians(40.0))
    # Beyond the MCU's range the command is clamped, never sent as-is.
    assert actuator.command(math.radians(20.0)) == pytest.approx(math.radians(99.0))
    assert math.degrees(actuator.road_wheel_limit_rad) == pytest.approx(12.375)
    offset = SteeringActuator(gain=8.0, offset_rad=0.08, max_command_rad=math.radians(99.0))
    assert offset.command(0.0) == pytest.approx(0.08)
    with pytest.raises(ValueError):
        SteeringActuator(gain=0.0, offset_rad=0.0, max_command_rad=1.7).validate()


def test_steering_limit_rejects_nonsense_geometry():
    with pytest.raises(ValueError):
        steering_limit_from_turning_radius(0.0, 3.4)
    with pytest.raises(ValueError):
        steering_limit_from_turning_radius(1.34, 0.0)


def _vehicle(max_steering_deg: float) -> Vehicle:
    return Vehicle(
        wheelbase_m=1.340,
        track_width_m=1.080,
        min_turning_radius_m=3.4,
        max_steering_angle_rad=math.radians(max_steering_deg),
        max_steering_rate_rad_s=math.radians(60.0),
        rear_axle_offset_m=0.0,
    )


def test_spec_sheet_steering_limit_is_not_flagged():
    assert implausible_steering_limit_reason(_vehicle(21.5)) is None


def test_steering_wheel_angle_mistaken_for_road_wheel_is_flagged():
    # 75 deg of road-wheel angle implies a 0.36 m radius, inside the vehicle's
    # own half track width - the signature of a steering-wheel figure.
    reason = implausible_steering_limit_reason(_vehicle(75.0))
    assert reason is not None
    assert "road-WHEEL" in reason


def test_max_curvature_is_the_inverse_of_the_implied_radius():
    vehicle = _vehicle(21.5)
    assert vehicle.max_curvature == pytest.approx(
        1.0 / vehicle.implied_turning_radius_m
    )


def test_curvature_ahead_sees_a_corner_within_the_window_only():
    from krowd_stanley_control.route_geometry import max_abs_value_ahead

    points = [(float(x), 0.0) for x in range(20)]
    curvatures = [0.0] * 20
    curvatures[10] = -0.2  # a right-hand corner at x = 10

    # From x = 4.5: 6 m ahead reaches x = 10.5, 4 m only x = 8.5.
    assert max_abs_value_ahead(points, curvatures, 4, 0.5, 6.0) == pytest.approx(0.2)
    assert max_abs_value_ahead(points, curvatures, 4, 0.5, 4.0) == pytest.approx(0.0)
    # Already in the corner.
    assert max_abs_value_ahead(points, curvatures, 10, 0.0, 1.0) == pytest.approx(0.2)
