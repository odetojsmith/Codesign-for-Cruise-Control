import numpy as np
import pytest

from codesign.efficiency import EfficiencyMap, default_motoring_map


def test_bilinear_interpolation() -> None:
    efficiency = EfficiencyMap(
        np.array([0.0, 1.0]),
        np.array([0.0, 1.0]),
        np.array([[0.8, 0.9], [0.9, 1.0]]),
    )
    assert efficiency.interpolate(0.5, 0.5) == pytest.approx(0.9)


def test_default_map_clamps_out_of_range_inputs() -> None:
    efficiency = default_motoring_map()
    assert efficiency.interpolate(10.0, 10.0) == pytest.approx(0.82)
    assert 0 < efficiency.interpolate(-1.0, -1.0) <= 1

