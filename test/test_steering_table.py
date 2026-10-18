import math

import pytest

from krowd_stanley_control.vehicle import SteeringActuator, bicycle_angle_rad, steering_table

ENCODER = [-2.71, -2.00, -1.00, 0.0, 1.00, 2.00, 2.75]
RIGHT = [-22.0, -16.2, -8.1, 0.0, 8.0, 16.0, 22.0]
LEFT = [-21.0, -15.5, -7.7, 0.0, 10.5, 21.1, 29.0]


def _actuator(max_command_deg=99.0, offset=0.0):
    command, wheel = steering_table(ENCODER, LEFT, RIGHT)
    actuator = SteeringActuator(8.0, offset, math.radians(max_command_deg), command, wheel)
    actuator.validate()
    return actuator


def test_the_bicycle_angle_lies_between_the_two_tyres():
    angle = math.degrees(bicycle_angle_rad(29.0, 22.0))
    assert 22.0 < angle < 29.0
    assert bicycle_angle_rad(0.0, 0.0) == 0.0


def test_command_and_road_wheel_are_inverses_inside_the_range():
    actuator = _actuator()
    for degrees in (-12.0, -5.0, 0.0, 3.0, 10.0):
        wheel = math.radians(degrees)
        assert actuator.road_wheel(actuator.command(wheel)) == pytest.approx(wheel)


def test_the_table_asks_for_less_command_than_gain_8():
    table, gain = _actuator(), SteeringActuator(8.0, 0.0, math.radians(99.0))
    for degrees in (-10.0, 5.0, 10.0):
        wheel = math.radians(degrees)
        assert abs(table.command(wheel)) < abs(gain.command(wheel))


def test_left_needs_less_command_than_right_for_the_same_angle():
    actuator = _actuator()
    wheel = math.radians(10.0)
    assert actuator.command(wheel) < -actuator.command(-wheel)


def test_the_limit_is_the_smaller_side_and_commands_stay_inside_it():
    actuator = _actuator()
    limit = actuator.road_wheel_limit_rad
    reach = math.radians(99.0)
    assert limit == pytest.approx(min(abs(actuator.road_wheel(-reach)), actuator.road_wheel(reach)))
    assert abs(actuator.command(math.radians(40.0))) == pytest.approx(reach)
    assert abs(actuator.command(-math.radians(40.0))) == pytest.approx(reach)


def test_no_table_falls_back_to_the_gain():
    assert steering_table([0.0], [0.0], [0.0]) == ((), ())
    actuator = SteeringActuator(8.0, 0.0, math.radians(99.0))
    assert not actuator.has_table
    assert actuator.command(0.1) == pytest.approx(0.8)
    assert actuator.road_wheel(0.8) == pytest.approx(0.1)


def test_a_table_that_does_not_cover_the_command_range_is_refused():
    command, wheel = steering_table([-1.0, 0.0, 1.0], [-8.0, 0.0, 10.0], [-8.0, 0.0, 8.0])
    with pytest.raises(ValueError, match="covers"):
        SteeringActuator(8.0, 0.0, math.radians(99.0), command, wheel).validate()


def test_each_side_gets_its_own_limit():
    actuator = _actuator()
    right, left = actuator.road_wheel_limits_rad()
    assert math.degrees(right) == pytest.approx(-13.7, abs=0.1)
    assert math.degrees(left) == pytest.approx(+15.7, abs=0.1)
    # The symmetric limit is still the smaller side...
    assert actuator.road_wheel_limit_rad == pytest.approx(-right)
    # ...and both ends are exactly what the command range reaches.
    reach = math.radians(99.0)
    assert actuator.command(left) == pytest.approx(reach)
    assert actuator.command(right) == pytest.approx(-reach)


def test_a_reversed_steering_sign_swaps_the_sides():
    right, left = _actuator().road_wheel_limits_rad(sign=-1.0)
    assert math.degrees(right) == pytest.approx(-15.7, abs=0.1)
    assert math.degrees(left) == pytest.approx(+13.7, abs=0.1)


def test_without_a_table_both_sides_are_the_same():
    right, left = SteeringActuator(8.0, 0.0, math.radians(99.0)).road_wheel_limits_rad()
    assert -right == pytest.approx(left) == pytest.approx(math.radians(99.0) / 8.0)


def test_an_offset_shifts_the_two_limits_instead_of_shrinking_both():
    # Straight ahead at -0.21 rad of encoder: the right side has 1.52 rad of
    # travel to the -99 deg limit, the left 1.94 rad to +99 deg.
    actuator = _actuator(offset=-0.21)
    right, left = actuator.road_wheel_limits_rad()
    assert actuator.road_wheel(-math.radians(99.0)) == pytest.approx(right)
    assert actuator.road_wheel(+math.radians(99.0)) == pytest.approx(left)
    assert -right < left  # less to the right, more to the left
    assert actuator.command(0.0) == pytest.approx(-0.21)
    assert actuator.road_wheel(-0.21) == pytest.approx(0.0)
    # ...and the commands for those limits are exactly the raw +/-99 deg.
    assert actuator.command(left) == pytest.approx(math.radians(99.0))
    assert actuator.command(right) == pytest.approx(-math.radians(99.0))
