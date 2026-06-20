"""Convex longitudinal MPC with hardware, comfort, curvature, and gap constraints."""

from __future__ import annotations

from dataclasses import dataclass, field
from math import sqrt
from typing import Any

import cvxpy as cp
import numpy as np

from .powertrain import EVPowertrain
from .scenarios import ControlObservation


@dataclass(frozen=True, slots=True)
class MPCDiagnostics:
    status: str
    objective: float | None
    solve_time_s: float | None
    iterations: int | None
    safety_slack_m: float
    predicted_speed_mps: tuple[float, ...]
    predicted_gap_m: tuple[float, ...]
    predicted_force_n: tuple[float, ...]
    used_fallback: bool


@dataclass(slots=True)
class LongitudinalMPCController:
    powertrain: EVPowertrain
    dt_s: float = 0.2
    horizon_steps: int = 20
    lambda_energy: float = 1.0
    lambda_force_slew: float = 0.1
    standstill_gap_m: float = 5.0
    time_headway_s: float = 1.5
    maximum_acceleration_mps2: float = 3.0
    maximum_jerk_mps3: float = 3.5
    combined_acceleration_limit_mps2: float = 3.5
    lead_prediction_braking_mps2: float = 3.0
    nominal_efficiency: float = 0.90
    _problem: cp.Problem = field(init=False, repr=False)
    _parameters: dict[str, cp.Parameter] = field(init=False, repr=False)
    _variables: dict[str, Any] = field(init=False, repr=False)
    last_diagnostics: MPCDiagnostics | None = field(init=False, default=None)
    solve_count: int = field(init=False, default=0)
    fallback_count: int = field(init=False, default=0)
    maximum_safety_slack_m: float = field(init=False, default=0.0)

    def __post_init__(self) -> None:
        if self.horizon_steps < 2:
            raise ValueError("horizon_steps must be at least two")
        if self.dt_s <= 0 or self.lambda_energy < 0 or self.lambda_force_slew < 0:
            raise ValueError("MPC timing and weights must be nonnegative, with positive dt")
        self._build_problem()

    @property
    def mass_kg(self) -> float:
        return self.powertrain.total_vehicle_mass_kg

    def _build_problem(self) -> None:
        horizon = self.horizon_steps
        force_scale = 6000.0
        speed = cp.Variable(horizon + 1, name="speed")
        gap = cp.Variable(horizon + 1, name="gap")
        normalized_force = cp.Variable(horizon, name="normalized_wheel_force")
        normalized_force_positive = cp.Variable(
            horizon, nonneg=True, name="normalized_force_positive"
        )
        normalized_force_negative = cp.Variable(
            horizon, nonneg=True, name="normalized_force_negative"
        )
        force = force_scale * normalized_force
        safety_slack = cp.Variable(horizon + 1, nonneg=True, name="safety_slack")

        parameters = {
            "initial_speed": cp.Parameter(nonneg=True),
            "initial_gap": cp.Parameter(nonneg=True),
            "previous_force": cp.Parameter(),
            "reference_speed": cp.Parameter(horizon + 1, nonneg=True),
            "lead_speed": cp.Parameter(horizon, nonneg=True),
            "minimum_force": cp.Parameter(horizon),
            "maximum_force": cp.Parameter(horizon),
            "drive_energy_coefficient": cp.Parameter(horizon, nonneg=True),
            "regen_energy_coefficient": cp.Parameter(horizon, nonneg=True),
            "grade_force": cp.Parameter(horizon),
        }
        constraints = [
            speed[0] == parameters["initial_speed"],
            gap[0] == parameters["initial_gap"],
            speed[1:]
            == speed[:-1]
            + self.dt_s * (force - parameters["grade_force"]) / self.mass_kg,
            gap[1:]
            == gap[:-1] + self.dt_s * (parameters["lead_speed"] - speed[:-1]),
            normalized_force == normalized_force_positive - normalized_force_negative,
            speed >= 0.0,
            force >= parameters["minimum_force"],
            force <= parameters["maximum_force"],
            gap
            >= self.standstill_gap_m + self.time_headway_s * speed - safety_slack,
        ]
        maximum_force_change = self.mass_kg * self.maximum_jerk_mps3 * self.dt_s
        constraints.extend(
            [
                force[0] - parameters["previous_force"] <= maximum_force_change,
                parameters["previous_force"] - force[0] <= maximum_force_change,
                force[1:] - force[:-1] <= maximum_force_change,
                force[:-1] - force[1:] <= maximum_force_change,
            ]
        )

        speed_scale = 10.0
        energy_scale_j = 1_000_000.0
        tracking_cost = cp.sum_squares(
            (speed - parameters["reference_speed"]) / speed_scale
        )
        slew = cp.hstack([force[0] - parameters["previous_force"], force[1:] - force[:-1]])
        slew_cost = cp.sum_squares(slew / force_scale)
        energy_cost = cp.sum(
            force_scale
            * (
                cp.multiply(
                    parameters["drive_energy_coefficient"], normalized_force_positive
                )
                - cp.multiply(
                    parameters["regen_energy_coefficient"], normalized_force_negative
                )
            )
        ) / energy_scale_j
        safety_cost = 10_000.0 * cp.sum_squares(safety_slack / 5.0)
        objective = cp.Minimize(
            tracking_cost
            + self.lambda_energy * energy_cost
            + self.lambda_force_slew * slew_cost
            + safety_cost
        )
        self._problem = cp.Problem(objective, constraints)
        self._parameters = parameters
        self._variables = {
            "speed": speed,
            "gap": gap,
            "force": force,
            "normalized_force": normalized_force,
            "safety_slack": safety_slack,
        }

    @staticmethod
    def _pad(values: tuple[float, ...], length: int, fallback: float) -> np.ndarray:
        source = list(values) if values else [fallback]
        source.extend([source[-1]] * max(0, length - len(source)))
        return np.asarray(source[:length], dtype=float)

    def _force_bounds(
        self,
        reference_speed: np.ndarray,
        curvature: np.ndarray,
        grade_force: np.ndarray | None = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        if grade_force is None:
            grade_force = np.zeros(len(reference_speed) - 1)
        minimum = []
        maximum = []
        for speed, curve, disturbance in zip(
            reference_speed[:-1], curvature[:-1], grade_force
        ):
            powertrain_minimum, powertrain_maximum = self.powertrain.force_limits(float(speed))
            lateral_acceleration = float(speed) ** 2 * abs(float(curve))
            combined_available = sqrt(
                max(
                    0.0,
                    self.combined_acceleration_limit_mps2**2 - lateral_acceleration**2,
                )
            )
            longitudinal_limit = self.mass_kg * min(
                self.maximum_acceleration_mps2, combined_available
            )
            minimum.append(max(powertrain_minimum, float(disturbance) - longitudinal_limit))
            maximum.append(min(powertrain_maximum, float(disturbance) + longitudinal_limit))
        return np.asarray(minimum), np.asarray(maximum)

    def reset(self) -> None:
        self.last_diagnostics = None
        self.solve_count = 0
        self.fallback_count = 0
        self.maximum_safety_slack_m = 0.0
        for variable in self._variables.values():
            if isinstance(variable, cp.Variable):
                variable.value = None

    def _fallback(self, observation: ControlObservation, minimum: float, maximum: float) -> float:
        safe_gap = self.standstill_gap_m + self.time_headway_s * observation.speed_mps
        if observation.lead_gap_m is not None and observation.lead_gap_m < safe_gap:
            desired = max(minimum, -self.mass_kg * self.maximum_acceleration_mps2)
        else:
            desired = 1000.0 * (
                observation.reference_speed_mps - observation.speed_mps
            )
            if observation.grade_preview_fraction:
                grade = observation.grade_preview_fraction[0]
                desired += self.mass_kg * 9.80665 * grade / sqrt(1.0 + grade**2)
        maximum_change = self.mass_kg * self.maximum_jerk_mps3 * self.dt_s
        desired = max(
            observation.previous_force_n - maximum_change,
            min(observation.previous_force_n + maximum_change, desired),
        )
        return max(minimum, min(maximum, desired))

    def command(self, observation: ControlObservation) -> float:
        horizon = self.horizon_steps
        reference = self._pad(
            observation.reference_preview_mps,
            horizon + 1,
            observation.reference_speed_mps,
        )
        curvature = self._pad(observation.curvature_preview_per_m, horizon + 1, 0.0)
        grade = self._pad(observation.grade_preview_fraction, horizon + 1, 0.0)
        grade_force = self.mass_kg * 9.80665 * grade[:-1] / np.sqrt(1.0 + grade[:-1] ** 2)
        minimum_force, maximum_force = self._force_bounds(reference, curvature, grade_force)
        has_lead = observation.lead_gap_m is not None and observation.lead_speed_mps is not None
        initial_gap = float(observation.lead_gap_m) if has_lead else 200.0
        if has_lead:
            lead_speed = np.maximum(
                0.0,
                float(observation.lead_speed_mps)
                - self.lead_prediction_braking_mps2
                * self.dt_s
                * np.arange(horizon, dtype=float),
            )
        else:
            lead_speed = np.full(horizon, float(reference[-1]))
        inverter = self.powertrain.battery.inverter_efficiency
        drive_coefficient = self.dt_s * reference[:-1] / (
            self.nominal_efficiency * inverter
        )
        regen_coefficient = self.dt_s * reference[:-1] * self.nominal_efficiency * inverter

        values: dict[str, Any] = {
            "initial_speed": max(0.0, observation.speed_mps),
            "initial_gap": max(0.0, initial_gap),
            "previous_force": observation.previous_force_n,
            "reference_speed": np.maximum(reference, 0.0),
            "lead_speed": np.maximum(lead_speed, 0.0),
            "minimum_force": minimum_force,
            "maximum_force": maximum_force,
            "drive_energy_coefficient": drive_coefficient,
            "regen_energy_coefficient": regen_coefficient,
            "grade_force": grade_force,
        }
        for name, value in values.items():
            self._parameters[name].value = value

        try:
            objective = self._problem.solve(
                solver=cp.OSQP,
                warm_start=True,
                eps_abs=1e-4,
                eps_rel=1e-4,
                max_iter=100_000,
                polishing=True,
                verbose=False,
            )
        except cp.error.SolverError:
            objective = None

        solved = self._problem.status in {cp.OPTIMAL, cp.OPTIMAL_INACCURATE}
        force_value = self._variables["force"].value if solved else None
        if force_value is None:
            command = self._fallback(observation, minimum_force[0], maximum_force[0])
            predicted_speed: tuple[float, ...] = ()
            predicted_gap: tuple[float, ...] = ()
            predicted_force: tuple[float, ...] = ()
            safety_slack = float("inf")
        else:
            command = float(force_value[0])
            predicted_speed = tuple(float(value) for value in self._variables["speed"].value)
            predicted_gap = tuple(float(value) for value in self._variables["gap"].value)
            predicted_force = tuple(float(value) for value in force_value)
            safety_slack = float(np.max(self._variables["safety_slack"].value))
        fallback_used = force_value is None or (has_lead and safety_slack > 1e-3)
        if fallback_used and force_value is not None:
            command = self._fallback(observation, minimum_force[0], maximum_force[0])
        stats = self._problem.solver_stats
        self.last_diagnostics = MPCDiagnostics(
            status=str(self._problem.status),
            objective=None if objective is None else float(objective),
            solve_time_s=None if stats is None else stats.solve_time,
            iterations=None if stats is None else stats.num_iters,
            safety_slack_m=safety_slack,
            predicted_speed_mps=predicted_speed,
            predicted_gap_m=predicted_gap,
            predicted_force_n=predicted_force,
            used_fallback=fallback_used,
        )
        self.solve_count += 1
        self.fallback_count += int(fallback_used)
        self.maximum_safety_slack_m = max(self.maximum_safety_slack_m, safety_slack)
        return max(float(minimum_force[0]), min(float(maximum_force[0]), command))
