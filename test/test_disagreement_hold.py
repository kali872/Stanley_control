import math

import pytest

from krowd_stanley_control.disagreement_hold import DisagreementHold

LIMIT = math.radians(15.0)


def test_under_limit_never_stops():
    hold = DisagreementHold(1.0, math.radians(45.0))
    assert not any(hold.update(t * 0.05, math.radians(14.9), LIMIT) for t in range(100))


def test_stops_after_hold_and_resets_on_agreement():
    hold = DisagreementHold(1.0, math.radians(45.0))
    assert not hold.update(0.0, math.radians(20.0), LIMIT)
    assert not hold.update(0.95, math.radians(20.0), LIMIT)
    assert hold.update(1.0, math.radians(-20.0), LIMIT)
    assert not hold.update(1.05, math.radians(3.0), LIMIT)
    assert hold.held_s(1.05) == 0.0
    assert not hold.update(1.1, math.radians(20.0), LIMIT)


def test_immediate_threshold():
    hold = DisagreementHold(1.0, math.radians(45.0))
    assert hold.update(0.0, math.radians(50.0), LIMIT)


def test_zero_hold_is_the_old_behaviour():
    hold = DisagreementHold(0.0, math.radians(45.0))
    assert hold.update(0.0, math.radians(16.0), LIMIT)


def test_rejects_bad_parameters():
    with pytest.raises(ValueError):
        DisagreementHold(-1.0, 1.0)
