"""Open-loop steering handoff, steady-turn, symmetry, and step-response validation."""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass
from math import atan, degrees
from pathlib import Path

import numpy as np

from .config import ProjectConfig
from .metadrive_env import MetaDriveEVEnv
from .powertrain import EVPowertrain


@dataclass(frozen=True, slots=True)
class SteeringSweepSample:
    requested_command: float
    reported_command: float
    reported_wheel_angle_deg: float
    mean_speed_mps: float
    yaw_rate_rad_s: float
    measured_curvature_per_m: float
    bicycle_curvature_per_m: float
    effective_steering_angle_deg: float


@dataclass(frozen=True, slots=True)
class SteeringStepPoint:
    time_s: float
    requested_command: float
    reported_command: float
    wheel_angle_deg: float
    speed_mps: float
    yaw_rate_rad_s: float


@dataclass(frozen=True, slots=True)
class SteeringStepResult:
    command: float
    steady_yaw_rate_rad_s: float
    response_delay_s: float
    rise_time_s: float
    points: tuple[SteeringStepPoint, ...]


@dataclass(frozen=True, slots=True)
class SteeringValidationResult:
    sweep: tuple[SteeringSweepSample, ...]
    step: SteeringStepResult
    maximum_command_handoff_error: float
    maximum_left_right_curvature_asymmetry: float
    maximum_bicycle_curvature_relative_error: float
    curvature_is_monotonic: bool
    signs_are_consistent: bool


def _angle_delta(current: float, previous: float) -> float:
    """Shortest signed angular change in radians."""

    return (current - previous + np.pi) % (2.0 * np.pi) - np.pi


def _speed_hold_force(env: MetaDriveEVEnv, target_speed_mps: float) -> float:
    requested = 3500.0 * (target_speed_mps - env.speed_mps)
    return max(-env.maximum_regenerative_force_n, min(env.maximum_traction_force_n, requested))


def _make_env(config: ProjectConfig) -> MetaDriveEVEnv:
    return MetaDriveEVEnv(
        EVPowertrain(config.hardware, config.vehicle, config.motor, config.battery),
        control_interval_s=config.control_interval_s,
        use_render=False,
        seed=config.seed,
        map_sequence="SSSSSSSSSSSS",
        traffic_density=0.0,
        terminate_on_out_of_road=False,
    )


def _settle_at_speed(env: MetaDriveEVEnv, target_speed_mps: float, timeout_s: float = 12.0) -> None:
    env.reset()
    for _ in range(round(timeout_s / env.control_interval_s)):
        env.step((0.0, _speed_hold_force(env, target_speed_mps)))
        if abs(env.speed_mps - target_speed_mps) < 0.05:
            return
    raise RuntimeError("steering validation could not settle at its target speed")


def _constant_steer_sample(
    env: MetaDriveEVEnv,
    command: float,
    target_speed_mps: float,
    duration_s: float = 3.0,
    discard_s: float = 1.0,
) -> SteeringSweepSample:
    _settle_at_speed(env, target_speed_mps)
    dt = env.control_interval_s
    previous_heading = env.heading_rad
    records: list[tuple[float, float, float, float]] = []
    for index in range(round(duration_s / dt)):
        env.step((command, _speed_hold_force(env, target_speed_mps)))
        heading = env.heading_rad
        yaw_rate = _angle_delta(heading, previous_heading) / dt
        previous_heading = heading
        if (index + 1) * dt >= discard_s:
            records.append(
                (
                    env.speed_mps,
                    yaw_rate,
                    env.applied_steering_command,
                    env.applied_steering_angle_deg,
                )
            )

    speed = float(np.mean([row[0] for row in records]))
    yaw_rate = float(np.mean([row[1] for row in records]))
    reported_command = float(np.mean([row[2] for row in records]))
    wheel_angle_deg = float(np.mean([row[3] for row in records]))
    curvature = yaw_rate / speed
    bicycle_curvature = np.tan(np.deg2rad(wheel_angle_deg)) / env.wheelbase_m
    effective_angle = degrees(atan(env.wheelbase_m * curvature))
    return SteeringSweepSample(
        requested_command=command,
        reported_command=reported_command,
        reported_wheel_angle_deg=wheel_angle_deg,
        mean_speed_mps=speed,
        yaw_rate_rad_s=yaw_rate,
        measured_curvature_per_m=curvature,
        bicycle_curvature_per_m=float(bicycle_curvature),
        effective_steering_angle_deg=effective_angle,
    )


def _first_crossing(times: list[float], values: list[float], threshold: float) -> float:
    for time_s, value in zip(times, values):
        if value >= threshold:
            return time_s
    return float("nan")


def _step_response(
    env: MetaDriveEVEnv,
    command: float,
    target_speed_mps: float,
    pre_step_s: float = 1.0,
    post_step_s: float = 4.0,
) -> SteeringStepResult:
    _settle_at_speed(env, target_speed_mps)
    dt = env.control_interval_s
    previous_heading = env.heading_rad
    points: list[SteeringStepPoint] = []
    total_steps = round((pre_step_s + post_step_s) / dt)
    for index in range(total_steps + 1):
        time_s = index * dt - pre_step_s
        requested = 0.0 if time_s < 0.0 else command
        env.step((requested, _speed_hold_force(env, target_speed_mps)))
        heading = env.heading_rad
        yaw_rate = _angle_delta(heading, previous_heading) / dt
        previous_heading = heading
        points.append(
            SteeringStepPoint(
                time_s=time_s,
                requested_command=requested,
                reported_command=env.applied_steering_command,
                wheel_angle_deg=env.applied_steering_angle_deg,
                speed_mps=env.speed_mps,
                yaw_rate_rad_s=yaw_rate,
            )
        )

    post = [point for point in points if point.time_s >= post_step_s - 1.0]
    steady = float(np.mean([point.yaw_rate_rad_s for point in post]))
    times = [point.time_s for point in points if point.time_s >= 0.0]
    normalized_yaw = [
        point.yaw_rate_rad_s * np.sign(steady)
        for point in points
        if point.time_s >= 0.0
    ]
    magnitude = abs(steady)
    t10 = _first_crossing(times, normalized_yaw, 0.1 * magnitude)
    t90 = _first_crossing(times, normalized_yaw, 0.9 * magnitude)
    return SteeringStepResult(
        command=command,
        steady_yaw_rate_rad_s=steady,
        response_delay_s=t10,
        rise_time_s=t90 - t10,
        points=tuple(points),
    )


def run_steering_validation(
    config: ProjectConfig,
    commands: tuple[float, ...] = (-0.20, -0.10, -0.05, 0.05, 0.10, 0.20),
    target_speed_mps: float = 8.0,
    step_command: float = 0.10,
) -> SteeringValidationResult:
    env = _make_env(config)
    try:
        sweep = tuple(_constant_steer_sample(env, command, target_speed_mps) for command in commands)
        step = _step_response(env, step_command, target_speed_mps)
    finally:
        env.close()

    by_magnitude: dict[float, list[SteeringSweepSample]] = {}
    for sample in sweep:
        by_magnitude.setdefault(abs(sample.requested_command), []).append(sample)
    asymmetries = []
    for pair in by_magnitude.values():
        if len(pair) == 2:
            magnitudes = [abs(sample.measured_curvature_per_m) for sample in pair]
            asymmetries.append(abs(magnitudes[0] - magnitudes[1]) / np.mean(magnitudes))

    positive = sorted(
        (sample for sample in sweep if sample.requested_command > 0),
        key=lambda sample: sample.requested_command,
    )
    monotonic = all(
        right.measured_curvature_per_m > left.measured_curvature_per_m
        for left, right in zip(positive, positive[1:])
    )
    signs = all(
        np.sign(sample.requested_command) == np.sign(sample.measured_curvature_per_m)
        for sample in sweep
    )
    return SteeringValidationResult(
        sweep=sweep,
        step=step,
        maximum_command_handoff_error=max(
            abs(sample.requested_command - sample.reported_command) for sample in sweep
        ),
        maximum_left_right_curvature_asymmetry=float(
            max(asymmetries, default=float("nan"))
        ),
        maximum_bicycle_curvature_relative_error=max(
            abs(sample.measured_curvature_per_m - sample.bicycle_curvature_per_m)
            / abs(sample.bicycle_curvature_per_m)
            for sample in sweep
        ),
        curvature_is_monotonic=monotonic,
        signs_are_consistent=signs,
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=Path("configs/default.yaml"))
    parser.add_argument("--output", type=Path, default=Path("artifacts/steering_validation.json"))
    args = parser.parse_args()
    result = run_steering_validation(ProjectConfig.from_yaml(args.config))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(asdict(result), indent=2) + "\n"
    args.output.write_text(payload, encoding="utf-8")
    print(payload, end="")


if __name__ == "__main__":
    main()
