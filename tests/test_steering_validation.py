import math

import pytest

from codesign.steering_validation import _angle_delta, _first_crossing


def test_angle_delta_unwraps_across_pi_boundary() -> None:
    assert _angle_delta(-math.pi + 0.1, math.pi - 0.1) == pytest.approx(0.2)
    assert _angle_delta(math.pi - 0.1, -math.pi + 0.1) == pytest.approx(-0.2)


def test_first_crossing_returns_first_time_and_nan_if_absent() -> None:
    assert _first_crossing([0.0, 0.2, 0.4], [0.0, 0.4, 1.0], 0.3) == pytest.approx(0.2)
    assert math.isnan(_first_crossing([0.0, 0.2], [0.0, 0.1], 0.3))
