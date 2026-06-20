"""Live blended-braking calibration and deterministic MPC lead-braking validation."""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass
from math import hypot
from pathlib import Path
from typing import Any

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from .config import ProjectConfig  # noqa: E402
from .controllers import CenterlinePIDController  # noqa: E402
from .metadrive_env import LeadVehicleState, MetaDriveEVEnv  # noqa: E402
from .mpc import LongitudinalMPCController  # noqa: E402
from .powertrain import EVPowertrain  # noqa: E402
from .scenarios import EpisodeResult, SpeedProfile, run_speed_profile  # noqa: E402


@dataclass(frozen=True, slots=True)
class BrakeCalibrationSample:
    requested_deceleration_mps2: float
    requested_force_n: float
    measured_deceleration_mps2: float
    equivalent_force_n: float
    relative_force_error: float
    regenerative_force_n: float
    friction_force_n: float
    mean_battery_power_w: float


@dataclass(frozen=True, slots=True)
class LeadBrakingMetrics:
    minimum_gap_m: float
    minimum_safe_gap_margin_m: float
    minimum_requested_force_n: float
    peak_deceleration_mps2: float
    peak_jerk_mps3: float
    fallback_count: int
    completed: bool


class VirtualBrakingLeadEnvironment:
    """Use live MetaDrive ego physics with a deterministic kinematic lead vehicle."""

    def __init__(
        self,
        base: MetaDriveEVEnv,
        initial_gap_m: float = 30.0,
        initial_speed_mps: float = 10.0,
        braking_start_s: float = 8.0,
        braking_mps2: float = 3.0,
    ) -> None:
        self.base = base
        self.initial_gap_m = initial_gap_m
        self.initial_speed_mps = initial_speed_mps
        self.braking_start_s = braking_start_s
        self.braking_mps2 = braking_mps2
        self.time_s = 0.0
        self.lead_speed_mps = initial_speed_mps
        self.lead_distance_m = initial_gap_m
        self.ego_distance_m = 0.0
        self._previous_position = (0.0, 0.0)

    def __getattr__(self, name: str) -> Any:
        return getattr(self.base, name)

    def reset(self) -> tuple[Any, dict[str, Any]]:
        output = self.base.reset()
        self.time_s = 0.0
        self.lead_speed_mps = self.initial_speed_mps
        self.lead_distance_m = self.initial_gap_m
        self.ego_distance_m = 0.0
        self._previous_position = self.base.position_xy_m
        return output

    def lead_vehicle_state(self, lateral_tolerance_m: float = 2.0) -> LeadVehicleState:
        del lateral_tolerance_m
        return LeadVehicleState(
            gap_m=self.lead_distance_m - self.ego_distance_m,
            speed_mps=self.lead_speed_mps,
        )

    def step(self, action: tuple[float, float]) -> tuple[Any, float, bool, bool, dict[str, Any]]:
        output = self.base.step(action)
        position = self.base.position_xy_m
        self.ego_distance_m += hypot(
            position[0] - self._previous_position[0],
            position[1] - self._previous_position[1],
        )
        self._previous_position = position
        old_lead_speed = self.lead_speed_mps
        if self.time_s >= self.braking_start_s:
            self.lead_speed_mps = max(
                0.0,
                self.lead_speed_mps - self.braking_mps2 * self.control_interval_s,
            )
        self.lead_distance_m += (
            0.5 * (old_lead_speed + self.lead_speed_mps) * self.control_interval_s
        )
        self.time_s += self.control_interval_s
        return output


def _brake_sample(
    env: MetaDriveEVEnv,
    deceleration_mps2: float,
    target_speed_mps: float = 15.0,
    duration_s: float = 1.2,
) -> BrakeCalibrationSample:
    env.reset()
    launch_force = 0.7 * env.maximum_traction_force_n
    for _ in range(round(15.0 / env.control_interval_s)):
        env.step((0.0, launch_force))
        if env.speed_mps >= target_speed_mps:
            break
    else:
        raise RuntimeError("vehicle did not reach blended-braking calibration speed")

    requested_force = -env.powertrain.total_vehicle_mass_kg * deceleration_mps2
    times = [0.0]
    speeds = [env.speed_mps]
    regenerative = []
    friction = []
    battery_power = []
    for index in range(1, round(duration_s / env.control_interval_s) + 1):
        env.step((0.0, requested_force))
        times.append(index * env.control_interval_s)
        speeds.append(env.speed_mps)
        step = env.last_powertrain_step
        if step is None:
            raise RuntimeError("powertrain step missing during braking calibration")
        regenerative.append(step.regenerative_wheel_force_n)
        friction.append(step.friction_brake_force_n)
        battery_power.append(step.battery_power_w)
    acceleration = float(np.polyfit(times, speeds, 1)[0])
    equivalent_force = acceleration * env.powertrain.total_vehicle_mass_kg
    return BrakeCalibrationSample(
        requested_deceleration_mps2=deceleration_mps2,
        requested_force_n=requested_force,
        measured_deceleration_mps2=-acceleration,
        equivalent_force_n=equivalent_force,
        relative_force_error=abs(equivalent_force - requested_force) / abs(requested_force),
        regenerative_force_n=float(np.mean(regenerative)),
        friction_force_n=float(np.mean(friction)),
        mean_battery_power_w=float(np.mean(battery_power)),
    )


def run_braking_validation(
    config: ProjectConfig,
) -> tuple[tuple[BrakeCalibrationSample, ...], EpisodeResult, LeadBrakingMetrics]:
    powertrain = EVPowertrain(config.hardware, config.vehicle, config.motor, config.battery)
    calibration_env = MetaDriveEVEnv(
        powertrain,
        control_interval_s=config.control_interval_s,
        seed=config.seed,
        map_sequence="SSSSSSSS",
        traffic_density=0.0,
    )
    try:
        samples = tuple(_brake_sample(calibration_env, value) for value in (2.0, 4.0, 5.5))
    finally:
        calibration_env.close()

    following_powertrain = EVPowertrain(
        config.hardware, config.vehicle, config.motor, config.battery
    )
    base = MetaDriveEVEnv(
        following_powertrain,
        control_interval_s=config.control_interval_s,
        seed=config.seed,
        map_sequence="SSSSSSSS",
        traffic_density=0.0,
    )
    env = VirtualBrakingLeadEnvironment(base)
    controller = LongitudinalMPCController(
        following_powertrain,
        dt_s=config.control_interval_s,
        lead_prediction_braking_mps2=3.0,
    )
    profile = SpeedProfile("virtual_lead_braking", (0.0, 5.0, 25.0), (0.0, 12.0, 12.0))
    try:
        result = run_speed_profile(
            env,
            profile,
            controller,
            CenterlinePIDController(dt_s=config.control_interval_s),
        )
    finally:
        base.close()
    gaps = [point.lead_gap_m for point in result.trajectory if point.lead_gap_m is not None]
    margins = [
        point.lead_gap_m - (controller.standstill_gap_m + controller.time_headway_s * point.speed_mps)
        for point in result.trajectory
        if point.lead_gap_m is not None
    ]
    metrics = LeadBrakingMetrics(
        minimum_gap_m=min(gaps),
        minimum_safe_gap_margin_m=min(margins),
        minimum_requested_force_n=min(point.requested_force_n for point in result.trajectory),
        peak_deceleration_mps2=max(-point.acceleration_mps2 for point in result.trajectory),
        peak_jerk_mps3=result.metrics.peak_jerk_mps3,
        fallback_count=controller.fallback_count,
        completed=result.metrics.completed,
    )
    return samples, result, metrics


def _plot(
    samples: tuple[BrakeCalibrationSample, ...],
    result: EpisodeResult,
    output: Path,
) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(13, 9), constrained_layout=True)
    requested = [sample.requested_force_n for sample in samples]
    measured = [sample.equivalent_force_n for sample in samples]
    axes[0, 0].scatter(requested, measured, s=65)
    limit = max(abs(value) for value in requested) * 1.05
    axes[0, 0].plot([-limit, 0], [-limit, 0], "k--", label="Ideal")
    axes[0, 0].set(title="Total braking-force delivery", xlabel="Requested force [N]", ylabel="Measured $ma$ [N]")
    axes[0, 0].legend()
    axes[0, 0].grid(alpha=0.3)

    labels = [f"{sample.requested_deceleration_mps2:g} m/s²" for sample in samples]
    regen = [abs(sample.regenerative_force_n) for sample in samples]
    friction = [abs(sample.friction_force_n) for sample in samples]
    axes[0, 1].bar(labels, regen, label="Regenerative")
    axes[0, 1].bar(labels, friction, bottom=regen, label="Friction")
    axes[0, 1].set(title="Blended-brake split", ylabel="Braking force magnitude [N]")
    axes[0, 1].legend()

    time = [point.time_s for point in result.trajectory]
    axes[1, 0].plot(time, [point.speed_mps for point in result.trajectory], label="Ego")
    axes[1, 0].plot(
        time,
        [point.lead_speed_mps for point in result.trajectory],
        label="Braking lead",
    )
    axes[1, 0].set(title="Deterministic lead-braking response", xlabel="Time [s]", ylabel="Speed [m/s]")
    axes[1, 0].legend()
    axes[1, 0].grid(alpha=0.3)

    gap = [float(point.lead_gap_m) for point in result.trajectory]
    safe = [5.0 + 1.5 * point.speed_mps for point in result.trajectory]
    axes[1, 1].plot(time, gap, label="Measured virtual gap")
    axes[1, 1].plot(time, safe, "r--", label="$5+1.5v$ requirement")
    axes[1, 1].set(title="Safety-gap constraint", xlabel="Time [s]", ylabel="Gap [m]")
    axes[1, 1].legend()
    axes[1, 1].grid(alpha=0.3)
    fig.suptitle("Blended braking and MPC lead-braking validation", fontsize=16)
    fig.savefig(output, dpi=180)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=Path("configs/default.yaml"))
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/braking_validation"))
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    samples, result, metrics = run_braking_validation(ProjectConfig.from_yaml(args.config))
    result.write_csv(args.output_dir / "lead_braking_trajectory.csv")
    _plot(samples, result, args.output_dir / "braking_validation.png")
    checks = {
        "total_braking_force_error_below_0_1_percent": max(
            sample.relative_force_error for sample in samples
        )
        < 0.001,
        "low_deceleration_is_regen_only": abs(samples[0].friction_force_n) < 1e-6,
        "high_deceleration_uses_friction": abs(samples[-1].friction_force_n) > 100.0,
        "lead_episode_completed": metrics.completed,
        "lead_gap_positive": metrics.minimum_gap_m > 0.0,
        "safe_gap_maintained": metrics.minimum_safe_gap_margin_m >= -0.25,
        "mpc_braked": metrics.minimum_requested_force_n < -100.0,
        "no_mpc_fallback": metrics.fallback_count == 0,
    }
    report = {
        "overall_passed": all(checks.values()),
        "checks": checks,
        "brake_calibration": [asdict(sample) for sample in samples],
        "lead_braking": asdict(metrics),
    }
    (args.output_dir / "braking_validation_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2))
    if not report["overall_passed"]:
        raise SystemExit("braking validation failed")


if __name__ == "__main__":
    main()
