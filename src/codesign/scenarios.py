"""Deterministic reference profiles and closed-loop episode logging."""

from __future__ import annotations

import csv
from dataclasses import asdict, dataclass
from math import sqrt
from pathlib import Path
from typing import Callable, Protocol, Sequence

import numpy as np

from .metadrive_env import LaneState, LeadVehicleState
from .powertrain import EnergyState, PowertrainStep
from .speed_planner import build_speed_plan


@dataclass(frozen=True, slots=True)
class SpeedProfile:
    name: str
    time_s: tuple[float, ...]
    speed_mps: tuple[float, ...]

    def __post_init__(self) -> None:
        if len(self.time_s) != len(self.speed_mps) or len(self.time_s) < 2:
            raise ValueError("profile time and speed arrays must have equal length >= 2")
        if self.time_s[0] != 0 or any(b <= a for a, b in zip(self.time_s, self.time_s[1:])):
            raise ValueError("profile time must start at zero and increase strictly")
        if any(speed < 0 for speed in self.speed_mps):
            raise ValueError("profile speeds cannot be negative")

    @property
    def duration_s(self) -> float:
        return self.time_s[-1]

    def reference_at(self, time_s: float) -> float:
        return float(np.interp(time_s, self.time_s, self.speed_mps))


@dataclass(frozen=True, slots=True)
class RoadGradeProfile:
    name: str
    distance_m: tuple[float, ...]
    grade_fraction: tuple[float, ...]

    def __post_init__(self) -> None:
        if len(self.distance_m) != len(self.grade_fraction) or len(self.distance_m) < 2:
            raise ValueError("grade distance and values must have equal length >= 2")
        if self.distance_m[0] != 0 or any(
            b <= a for a, b in zip(self.distance_m, self.distance_m[1:])
        ):
            raise ValueError("grade distance must start at zero and increase strictly")
        if any(abs(value) > 0.25 for value in self.grade_fraction):
            raise ValueError("grade fractions must remain within +/-25%")

    def grade_at(self, distance_m: float) -> float:
        return float(np.interp(distance_m, self.distance_m, self.grade_fraction))


URBAN_PROFILE = SpeedProfile(
    name="urban_stop_go",
    time_s=(0.0, 4.0, 12.0, 18.0, 24.0, 28.0, 36.0, 44.0),
    speed_mps=(0.0, 8.0, 12.0, 12.0, 0.0, 0.0, 10.0, 10.0),
)

HIGHWAY_PROFILE = SpeedProfile(
    name="highway_changes",
    time_s=(0.0, 8.0, 20.0, 28.0, 40.0, 48.0),
    speed_mps=(16.0, 24.0, 24.0, 30.0, 30.0, 22.0),
)

HIGHWAY_TRAINING_PROFILE = SpeedProfile(
    name="highway_training",
    time_s=(0.0, 8.0, 11.0),
    speed_mps=(0.0, 16.0, 20.0),
)

CENTERLINE_PROFILE = SpeedProfile(
    name="curved_centerline",
    time_s=(0.0, 5.0, 30.0),
    speed_mps=(0.0, 12.0, 12.0),
)

MIXED_GRADE_SPEED_PROFILE = SpeedProfile(
    name="mixed_grade",
    time_s=(0.0, 3.0, 8.0, 12.0, 16.0),
    speed_mps=(0.0, 8.0, 14.0, 14.0, 8.0),
)

MIXED_GRADE_PROFILE = RoadGradeProfile(
    name="mixed_grade_route",
    distance_m=(0.0, 25.0, 55.0, 85.0, 120.0, 180.0),
    grade_fraction=(0.0, 0.06, -0.04, 0.03, -0.06, 0.0),
)


@dataclass(frozen=True, slots=True)
class ControlObservation:
    time_s: float
    speed_mps: float
    reference_speed_mps: float
    previous_force_n: float
    lead_gap_m: float | None = None
    lead_speed_mps: float | None = None
    reference_preview_mps: tuple[float, ...] = ()
    curvature_preview_per_m: tuple[float, ...] = ()
    grade_preview_fraction: tuple[float, ...] = ()


class LongitudinalController(Protocol):
    def reset(self) -> None: ...

    def command(self, observation: ControlObservation) -> float: ...


class LateralController(Protocol):
    def reset(self) -> None: ...

    def command(self, lane: LaneState) -> float: ...


@dataclass(slots=True)
class ProportionalForceController:
    """Temporary baseline used to validate scenarios before MPC design."""

    gain_n_per_mps: float = 1200.0
    minimum_force_n: float = -6000.0
    maximum_force_n: float = 6000.0

    def reset(self) -> None:
        return None

    def command(self, observation: ControlObservation) -> float:
        force = self.gain_n_per_mps * (
            observation.reference_speed_mps - observation.speed_mps
        )
        return max(self.minimum_force_n, min(self.maximum_force_n, force))


class LongitudinalEnvironment(Protocol):
    control_interval_s: float
    energy: EnergyState
    last_powertrain_step: PowertrainStep | None

    @property
    def speed_mps(self) -> float: ...

    @property
    def position_xy_m(self) -> tuple[float, float]: ...

    def reset(self) -> tuple[object, dict[str, object]]: ...

    def step(
        self, action: tuple[float, float]
    ) -> tuple[object, float, bool, bool, dict[str, object]]: ...

    def lead_vehicle_state(self, lateral_tolerance_m: float = 2.0) -> LeadVehicleState | None: ...

    def lane_state(self) -> LaneState: ...

    def road_curvature_preview(self, distances_m: tuple[float, ...]) -> tuple[float, ...]: ...

    def road_grade_preview(self, distances_m: tuple[float, ...]) -> tuple[float, ...]: ...


@dataclass(frozen=True, slots=True)
class TrajectoryPoint:
    time_s: float
    reference_speed_mps: float
    speed_mps: float
    requested_force_n: float
    applied_force_n: float
    acceleration_mps2: float
    jerk_mps3: float
    battery_power_w: float
    cumulative_battery_wh: float
    distance_m: float
    lead_gap_m: float | None
    lead_speed_mps: float | None
    saturated: bool
    steering_command: float
    lateral_error_m: float
    heading_error_rad: float
    motor_mechanical_power_w: float
    motor_efficiency: float
    actuator_speed_mps: float
    regenerative_force_n: float
    friction_brake_force_n: float
    road_grade_fraction: float
    grade_resistance_force_n: float
    motor_temperature_c: float
    thermal_derating_factor: float
    battery_power_limited: bool


@dataclass(frozen=True, slots=True)
class EpisodeMetrics:
    rmse_mps: float
    net_battery_wh: float
    wh_per_km: float
    distance_m: float
    peak_acceleration_mps2: float
    peak_jerk_mps3: float
    minimum_gap_m: float | None
    saturation_fraction: float
    lateral_rmse_m: float
    maximum_abs_lateral_error_m: float
    completed: bool


@dataclass(frozen=True, slots=True)
class EpisodeResult:
    profile_name: str
    metrics: EpisodeMetrics
    trajectory: tuple[TrajectoryPoint, ...]

    def write_csv(self, path: str | Path) -> None:
        output = Path(path)
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(asdict(self.trajectory[0]).keys()))
            writer.writeheader()
            writer.writerows(asdict(point) for point in self.trajectory)


def _metrics(points: Sequence[TrajectoryPoint], net_battery_wh: float, completed: bool) -> EpisodeMetrics:
    if not points:
        raise ValueError("cannot calculate metrics for an empty trajectory")
    rmse = sqrt(sum((p.speed_mps - p.reference_speed_mps) ** 2 for p in points) / len(points))
    distance = points[-1].distance_m
    gaps = [p.lead_gap_m for p in points if p.lead_gap_m is not None]
    return EpisodeMetrics(
        rmse_mps=rmse,
        net_battery_wh=net_battery_wh,
        wh_per_km=float("inf") if distance <= 0 else net_battery_wh / (distance / 1000.0),
        distance_m=distance,
        peak_acceleration_mps2=max(abs(p.acceleration_mps2) for p in points),
        peak_jerk_mps3=max(abs(p.jerk_mps3) for p in points),
        minimum_gap_m=min(gaps) if gaps else None,
        saturation_fraction=sum(p.saturated for p in points) / len(points),
        lateral_rmse_m=sqrt(sum(p.lateral_error_m**2 for p in points) / len(points)),
        maximum_abs_lateral_error_m=max(abs(p.lateral_error_m) for p in points),
        completed=completed,
    )


def run_speed_profile(
    env: LongitudinalEnvironment,
    profile: SpeedProfile,
    controller: LongitudinalController,
    lateral_controller: LateralController | None = None,
    step_callback: Callable[[int, LongitudinalEnvironment], None] | None = None,
    preview_steps: int = 20,
) -> EpisodeResult:
    """Run one deterministic profile and collect controller-independent system metrics."""

    env.reset()
    controller.reset()
    if lateral_controller is not None:
        lateral_controller.reset()
    dt = env.control_interval_s
    previous_speed = env.speed_mps
    previous_acceleration = 0.0
    previous_force = 0.0
    previous_position = env.position_xy_m
    distance_m = 0.0
    points: list[TrajectoryPoint] = []
    completed = True

    for index in range(int(round(profile.duration_s / dt)) + 1):
        time_s = index * dt
        lead = env.lead_vehicle_state()
        mission_preview = tuple(
            profile.reference_at(time_s + preview_index * dt)
            for preview_index in range(preview_steps + 1)
        )
        speed_plan = build_speed_plan(mission_preview, env.road_curvature_preview, dt)
        grade_preview = env.road_grade_preview(speed_plan.distances_m)
        observation = ControlObservation(
            time_s=time_s,
            speed_mps=env.speed_mps,
            reference_speed_mps=speed_plan.reference_mps[0],
            previous_force_n=previous_force,
            lead_gap_m=None if lead is None else lead.gap_m,
            lead_speed_mps=None if lead is None else lead.speed_mps,
            reference_preview_mps=speed_plan.reference_mps,
            curvature_preview_per_m=speed_plan.curvature_per_m,
            grade_preview_fraction=grade_preview,
        )
        requested_force = controller.command(observation)
        lane = env.lane_state()
        steering = 0.0 if lateral_controller is None else lateral_controller.command(lane)
        _, _, terminated, truncated, _ = env.step((steering, requested_force))
        powertrain = env.last_powertrain_step
        if powertrain is None:
            raise RuntimeError("environment did not expose its powertrain step")

        speed = env.speed_mps
        acceleration = (speed - previous_speed) / dt
        jerk = (acceleration - previous_acceleration) / dt
        position = env.position_xy_m
        distance_m += sqrt(
            (position[0] - previous_position[0]) ** 2
            + (position[1] - previous_position[1]) ** 2
        )
        points.append(
            TrajectoryPoint(
                time_s=time_s,
                reference_speed_mps=observation.reference_speed_mps,
                speed_mps=speed,
                requested_force_n=requested_force,
                applied_force_n=powertrain.applied_wheel_force_n,
                acceleration_mps2=acceleration,
                jerk_mps3=jerk,
                battery_power_w=powertrain.battery_power_w,
                cumulative_battery_wh=env.energy.net_battery_wh,
                distance_m=distance_m,
                lead_gap_m=observation.lead_gap_m,
                lead_speed_mps=observation.lead_speed_mps,
                saturated=powertrain.saturated,
                steering_command=steering,
                lateral_error_m=lane.lateral_error_m,
                heading_error_rad=lane.heading_error_rad,
                motor_mechanical_power_w=(
                    powertrain.motor_torque_nm * powertrain.motor_speed_rad_s
                ),
                motor_efficiency=powertrain.motor_efficiency,
                actuator_speed_mps=powertrain.vehicle_speed_mps,
                regenerative_force_n=powertrain.regenerative_wheel_force_n,
                friction_brake_force_n=powertrain.friction_brake_force_n,
                road_grade_fraction=grade_preview[0],
                grade_resistance_force_n=(
                    powertrain.applied_wheel_force_n
                    - float(getattr(env, "last_net_chassis_force_n", powertrain.applied_wheel_force_n))
                ),
                motor_temperature_c=powertrain.motor_temperature_c,
                thermal_derating_factor=powertrain.thermal_derating_factor,
                battery_power_limited=powertrain.battery_power_limited,
            )
        )
        if step_callback is not None:
            step_callback(index, env)
        previous_speed = speed
        previous_acceleration = acceleration
        previous_force = requested_force
        previous_position = position
        if terminated or truncated:
            completed = time_s + dt >= profile.duration_s
            break

    return EpisodeResult(
        profile_name=profile.name,
        metrics=_metrics(points, env.energy.net_battery_wh, completed),
        trajectory=tuple(points),
    )
