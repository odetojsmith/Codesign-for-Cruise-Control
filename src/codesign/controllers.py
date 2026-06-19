"""Heuristic PID controllers used to validate the pre-MPC simulation loop."""

from __future__ import annotations

from dataclasses import dataclass, field

from .metadrive_env import LaneState
from .scenarios import ControlObservation


@dataclass(slots=True)
class PID:
    kp: float
    ki: float
    kd: float
    dt_s: float
    output_min: float
    output_max: float
    integral_min: float = -float("inf")
    integral_max: float = float("inf")
    _integral: float = field(init=False, default=0.0)
    _previous_error: float | None = field(init=False, default=None)

    def reset(self) -> None:
        self._integral = 0.0
        self._previous_error = None

    def update(self, error: float) -> float:
        derivative = (
            0.0
            if self._previous_error is None
            else (error - self._previous_error) / self.dt_s
        )
        candidate_integral = max(
            self.integral_min,
            min(self.integral_max, self._integral + error * self.dt_s),
        )
        raw = self.kp * error + self.ki * candidate_integral + self.kd * derivative
        output = max(self.output_min, min(self.output_max, raw))
        # Conditional integration prevents wind-up when saturation pushes in the error direction.
        if output == raw or (output == self.output_max and error < 0) or (
            output == self.output_min and error > 0
        ):
            self._integral = candidate_integral
        self._previous_error = error
        return output


@dataclass(slots=True)
class LongitudinalPIDController:
    dt_s: float = 0.2
    kp_n_per_mps: float = 1200.0
    ki_n_per_m: float = 120.0
    kd_n_s_per_m: float = 80.0
    minimum_force_n: float = -6000.0
    maximum_force_n: float = 6000.0
    _pid: PID = field(init=False)

    def __post_init__(self) -> None:
        self._pid = PID(
            self.kp_n_per_mps,
            self.ki_n_per_m,
            self.kd_n_s_per_m,
            self.dt_s,
            self.minimum_force_n,
            self.maximum_force_n,
            integral_min=-20.0,
            integral_max=20.0,
        )

    def reset(self) -> None:
        self._pid.reset()

    def command(self, observation: ControlObservation) -> float:
        return self._pid.update(observation.reference_speed_mps - observation.speed_mps)


@dataclass(slots=True)
class CenterlinePIDController:
    """Two-loop centerline controller following MetaDrive's lateral/heading convention."""

    dt_s: float = 0.2
    lateral_kp: float = 0.30
    lateral_ki: float = 0.01
    lateral_kd: float = 0.01
    heading_kp: float = 1.70
    heading_ki: float = 0.05
    heading_kd: float = 0.70
    _lateral_pid: PID = field(init=False)
    _heading_pid: PID = field(init=False)

    def __post_init__(self) -> None:
        self._lateral_pid = PID(
            self.lateral_kp,
            self.lateral_ki,
            self.lateral_kd,
            self.dt_s,
            -1.0,
            1.0,
            integral_min=-2.0,
            integral_max=2.0,
        )
        self._heading_pid = PID(
            self.heading_kp,
            self.heading_ki,
            self.heading_kd,
            self.dt_s,
            -1.0,
            1.0,
            integral_min=-1.0,
            integral_max=1.0,
        )

    def reset(self) -> None:
        self._lateral_pid.reset()
        self._heading_pid.reset()

    def command(self, lane: LaneState) -> float:
        # MetaDrive's positive steering action corrects positive lane-lateral and lane-heading
        # errors. Its built-in controller hides this sign inside a negating PID implementation;
        # this project uses a conventional positive-gain PID and therefore passes errors directly.
        steering = self._heading_pid.update(lane.heading_error_rad)
        steering += self._lateral_pid.update(lane.lateral_error_m)
        return max(-1.0, min(1.0, steering))
