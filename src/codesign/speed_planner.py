"""Curvature-aware speed-reference preview shared by PID and MPC experiments."""

from __future__ import annotations

from dataclasses import dataclass
from math import sqrt
from typing import Callable


@dataclass(frozen=True, slots=True)
class SpeedPlan:
    reference_mps: tuple[float, ...]
    curvature_per_m: tuple[float, ...]
    distances_m: tuple[float, ...]


def curvature_aware_speed_plan(
    mission_reference_mps: tuple[float, ...],
    curvature_per_m: tuple[float, ...],
    dt_s: float,
    maximum_lateral_acceleration_mps2: float = 2.0,
    maximum_acceleration_mps2: float = 3.0,
    maximum_braking_mps2: float = 3.0,
) -> tuple[float, ...]:
    """Apply curve limits and forward/backward longitudinal feasibility passes."""

    if len(mission_reference_mps) != len(curvature_per_m) or not mission_reference_mps:
        raise ValueError("mission reference and curvature must have equal nonzero length")
    if dt_s <= 0:
        raise ValueError("dt_s must be positive")
    limited = [
        min(
            mission,
            sqrt(maximum_lateral_acceleration_mps2 / max(abs(curvature), 1e-9)),
        )
        for mission, curvature in zip(mission_reference_mps, curvature_per_m)
    ]
    for index in range(len(limited) - 2, -1, -1):
        limited[index] = min(
            limited[index], limited[index + 1] + maximum_braking_mps2 * dt_s
        )
    for index in range(1, len(limited)):
        limited[index] = min(
            limited[index], limited[index - 1] + maximum_acceleration_mps2 * dt_s
        )
    return tuple(max(0.0, speed) for speed in limited)


def build_speed_plan(
    mission_reference_mps: tuple[float, ...],
    curvature_query: Callable[[tuple[float, ...]], tuple[float, ...]],
    dt_s: float,
) -> SpeedPlan:
    distances = [0.0]
    for speed in mission_reference_mps[:-1]:
        distances.append(distances[-1] + max(0.0, speed) * dt_s)
    curvature = tuple(float(value) for value in curvature_query(tuple(distances)))
    reference = curvature_aware_speed_plan(mission_reference_mps, curvature, dt_s)
    return SpeedPlan(reference, curvature, tuple(distances))
