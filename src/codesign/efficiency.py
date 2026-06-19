"""Motor efficiency-map interpolation without a SciPy runtime dependency."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray


@dataclass(frozen=True, slots=True)
class EfficiencyMap:
    """Bilinear map indexed by normalized speed and absolute normalized torque."""

    speed_axis: NDArray[np.float64]
    torque_axis: NDArray[np.float64]
    values: NDArray[np.float64]

    def __post_init__(self) -> None:
        if self.values.shape != (len(self.speed_axis), len(self.torque_axis)):
            raise ValueError("efficiency-map shape must match its axes")
        if np.any(np.diff(self.speed_axis) <= 0) or np.any(np.diff(self.torque_axis) <= 0):
            raise ValueError("efficiency-map axes must be strictly increasing")
        if np.any((self.values <= 0) | (self.values > 1)):
            raise ValueError("efficiencies must be in (0, 1]")

    def interpolate(self, normalized_speed: float, normalized_torque: float) -> float:
        speed = float(np.clip(normalized_speed, self.speed_axis[0], self.speed_axis[-1]))
        torque = float(np.clip(abs(normalized_torque), self.torque_axis[0], self.torque_axis[-1]))

        speed_hi = min(int(np.searchsorted(self.speed_axis, speed, side="right")), len(self.speed_axis) - 1)
        torque_hi = min(
            int(np.searchsorted(self.torque_axis, torque, side="right")),
            len(self.torque_axis) - 1,
        )
        speed_lo = max(speed_hi - 1, 0)
        torque_lo = max(torque_hi - 1, 0)

        s0, s1 = self.speed_axis[speed_lo], self.speed_axis[speed_hi]
        t0, t1 = self.torque_axis[torque_lo], self.torque_axis[torque_hi]
        ws = 0.0 if s1 == s0 else (speed - s0) / (s1 - s0)
        wt = 0.0 if t1 == t0 else (torque - t0) / (t1 - t0)

        v00 = self.values[speed_lo, torque_lo]
        v01 = self.values[speed_lo, torque_hi]
        v10 = self.values[speed_hi, torque_lo]
        v11 = self.values[speed_hi, torque_hi]
        value = (1 - ws) * ((1 - wt) * v00 + wt * v01) + ws * (
            (1 - wt) * v10 + wt * v11
        )
        return float(value)


def default_motoring_map() -> EfficiencyMap:
    """Return a smooth illustrative map to be replaced by sourced motor data later."""

    axis = np.array([0.0, 0.25, 0.50, 0.75, 1.0], dtype=float)
    values = np.array(
        [
            [0.78, 0.84, 0.87, 0.86, 0.82],
            [0.82, 0.89, 0.93, 0.92, 0.88],
            [0.84, 0.91, 0.95, 0.94, 0.90],
            [0.82, 0.90, 0.94, 0.92, 0.87],
            [0.78, 0.86, 0.90, 0.87, 0.82],
        ],
        dtype=float,
    )
    return EfficiencyMap(axis, axis.copy(), values)

