"""Generate visual evidence for actuator, energy, speed, and centerline behavior."""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import matplotlib
import numpy as np
from PIL import Image

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from .calibration import CalibrationSample, run_actuator_calibration  # noqa: E402
from .config import ProjectConfig  # noqa: E402
from .controllers import CenterlinePIDController, LongitudinalPIDController  # noqa: E402
from .metadrive_env import MetaDriveEVEnv  # noqa: E402
from .powertrain import EVPowertrain  # noqa: E402
from .scenarios import (  # noqa: E402
    CENTERLINE_PROFILE,
    URBAN_PROFILE,
    EpisodeResult,
    run_speed_profile,
)


@dataclass(frozen=True, slots=True)
class EnergyConsistency:
    integrated_battery_wh: float
    recorded_battery_wh: float
    energy_residual_wh: float
    maximum_power_residual_w: float
    maximum_driveline_residual_w: float


def _energy_consistency(result: EpisodeResult, config: ProjectConfig) -> EnergyConsistency:
    expected_power: list[float] = []
    driveline_residual: list[float] = []
    for point in result.trajectory:
        mechanical = point.motor_mechanical_power_w
        if mechanical >= 0:
            traction = mechanical / (point.motor_efficiency * config.battery.inverter_efficiency)
            expected_motor = (
                point.applied_force_n
                * point.actuator_speed_mps
                / config.vehicle.final_drive_efficiency
            )
        else:
            traction = mechanical * point.motor_efficiency * config.battery.inverter_efficiency
            expected_motor = (
                point.applied_force_n
                * point.actuator_speed_mps
                * config.vehicle.final_drive_efficiency
            )
        expected_power.append(traction + config.battery.auxiliary_power_w)
        driveline_residual.append(mechanical - expected_motor)

    integrated = sum(expected_power) * config.control_interval_s / 3600.0
    recorded = result.metrics.net_battery_wh
    power_residual = [
        point.battery_power_w - expected
        for point, expected in zip(result.trajectory, expected_power)
    ]
    return EnergyConsistency(
        integrated_battery_wh=integrated,
        recorded_battery_wh=recorded,
        energy_residual_wh=recorded - integrated,
        maximum_power_residual_w=max(abs(value) for value in power_residual),
        maximum_driveline_residual_w=max(abs(value) for value in driveline_residual),
    )


def _save_gif(frames: list[np.ndarray[Any, Any]], output: Path) -> None:
    if not frames:
        raise RuntimeError("MetaDrive produced no top-down frames")
    images = [Image.fromarray(np.asarray(frame, dtype=np.uint8)) for frame in frames]
    images[0].save(
        output,
        save_all=True,
        append_images=images[1:],
        duration=120,
        loop=0,
        optimize=True,
    )
    images[-1].save(output.with_name("centerline_topdown.png"))


def _run_episode(
    config: ProjectConfig,
    profile: Any,
    map_sequence: str,
    spawn_lateral_m: float,
    capture_frames: bool,
) -> tuple[EpisodeResult, list[np.ndarray[Any, Any]]]:
    env = MetaDriveEVEnv(
        EVPowertrain(config.hardware, config.vehicle, config.motor, config.battery),
        control_interval_s=config.control_interval_s,
        use_render=False,
        seed=config.seed,
        map_sequence=map_sequence,
        traffic_density=0.0,
        spawn_lateral_m=spawn_lateral_m,
    )
    frames: list[np.ndarray[Any, Any]] = []

    def capture(index: int, active_env: Any) -> None:
        if capture_frames and index % 5 == 0:
            frame = np.asarray(active_env.render_topdown())
            frames.append(frame.swapaxes(0, 1))

    try:
        result = run_speed_profile(
            env,
            profile,
            LongitudinalPIDController(dt_s=config.control_interval_s),
            CenterlinePIDController(dt_s=config.control_interval_s),
            step_callback=capture,
        )
        return result, frames
    finally:
        env.close()


def _plot_dashboard(
    calibration: tuple[CalibrationSample, ...],
    centerline: EpisodeResult,
    urban: EpisodeResult,
    output: Path,
) -> None:
    fig, axes = plt.subplots(3, 2, figsize=(13, 12), constrained_layout=True)
    traction = [sample for sample in calibration if sample.mode == "traction"]
    regeneration = [sample for sample in calibration if sample.mode == "regeneration"]
    axis = axes[0, 0]
    axis.scatter(
        [sample.requested_force_n for sample in traction],
        [sample.equivalent_force_n for sample in traction],
        label="Traction",
        s=55,
    )
    axis.scatter(
        [sample.requested_force_n for sample in regeneration],
        [sample.equivalent_force_n for sample in regeneration],
        label="Regeneration",
        s=55,
    )
    force_limit = max(abs(sample.requested_force_n) for sample in calibration) * 1.05
    axis.plot([-force_limit, force_limit], [-force_limit, force_limit], "k--", label="Ideal $ma=F$")
    axis.set(title="Actuator validation", xlabel="Requested wheel force [N]", ylabel="Measured $ma$ [N]")
    axis.legend()
    axis.grid(alpha=0.3)

    for result, axis, title in (
        (centerline, axes[0, 1], "Curved-track speed PID"),
        (urban, axes[1, 0], "Urban stop-go speed PID"),
    ):
        time = [point.time_s for point in result.trajectory]
        axis.plot(time, [point.reference_speed_mps for point in result.trajectory], "k--", label="Reference")
        axis.plot(time, [point.speed_mps for point in result.trajectory], label="Vehicle")
        axis.set(title=title, xlabel="Time [s]", ylabel="Speed [m/s]")
        axis.legend()
        axis.grid(alpha=0.3)

    axis = axes[1, 1]
    time = [point.time_s for point in centerline.trajectory]
    axis.plot(time, [point.lateral_error_m for point in centerline.trajectory], label="Lateral error")
    axis.axhline(0.0, color="k", linewidth=1)
    axis.axhline(1.75, color="r", linestyle="--", linewidth=1, label="Lane boundary")
    axis.axhline(-1.75, color="r", linestyle="--", linewidth=1)
    axis.set(title="Centerline tracking", xlabel="Time [s]", ylabel="Lateral error [m]")
    axis.legend()
    axis.grid(alpha=0.3)

    axis = axes[2, 0]
    time = [point.time_s for point in urban.trajectory]
    axis.plot(time, [point.battery_power_w / 1000 for point in urban.trajectory], label="Battery power")
    axis.axhline(0.0, color="k", linewidth=1)
    axis.set(title="Urban energy flow", xlabel="Time [s]", ylabel="Battery power [kW]")
    axis.grid(alpha=0.3)
    energy_axis = axis.twinx()
    energy_axis.plot(
        time,
        [point.cumulative_battery_wh for point in urban.trajectory],
        color="tab:orange",
        label="Cumulative energy",
    )
    energy_axis.set_ylabel("Cumulative battery energy [Wh]", color="tab:orange")

    axis = axes[2, 1]
    positions_x = np.array([point.distance_m for point in centerline.trajectory])
    lateral = np.array([point.lateral_error_m for point in centerline.trajectory])
    centerline_time = [point.time_s for point in centerline.trajectory]
    scatter = axis.scatter(positions_x, lateral, c=centerline_time, cmap="viridis", s=16)
    axis.axhline(0.0, color="k", linewidth=1)
    axis.set(title="Centerline error along route", xlabel="Distance traveled [m]", ylabel="Lateral error [m]")
    axis.grid(alpha=0.3)
    fig.colorbar(scatter, ax=axis, label="Time [s]")
    fig.suptitle("EV actuator, energy, speed, and centerline validation", fontsize=16)
    fig.savefig(output, dpi=170)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=Path("configs/default.yaml"))
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/validation"))
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    config = ProjectConfig.from_yaml(args.config)

    calibration = run_actuator_calibration(config)
    centerline, frames = _run_episode(config, CENTERLINE_PROFILE, "SCSCSC", 1.0, True)
    urban, _ = _run_episode(config, URBAN_PROFILE, "SSSSSSSSSSSS", 0.5, False)
    centerline.write_csv(args.output_dir / "centerline_trajectory.csv")
    urban.write_csv(args.output_dir / "urban_trajectory.csv")
    _save_gif(frames, args.output_dir / "centerline_topdown.gif")
    _plot_dashboard(calibration, centerline, urban, args.output_dir / "validation_dashboard.png")

    actuator_error = max(abs(sample.force_gain - 1.0) for sample in calibration)
    centerline_energy = _energy_consistency(centerline, config)
    urban_energy = _energy_consistency(urban, config)
    checks = {
        "actuator_relative_error_below_0_1_percent": actuator_error < 0.001,
        "centerline_episode_completed": centerline.metrics.completed,
        "centerline_stayed_inside_lane": centerline.metrics.maximum_abs_lateral_error_m < 1.75,
        "urban_episode_completed": urban.metrics.completed,
        "centerline_energy_balance_closed": abs(centerline_energy.energy_residual_wh) < 1e-9,
        "urban_energy_balance_closed": abs(urban_energy.energy_residual_wh) < 1e-9,
        "driveline_power_balance_closed": max(
            centerline_energy.maximum_driveline_residual_w,
            urban_energy.maximum_driveline_residual_w,
        )
        < 1e-6,
    }
    report = {
        "overall_passed": all(checks.values()),
        "checks": checks,
        "maximum_actuator_relative_error": actuator_error,
        "centerline": asdict(centerline.metrics),
        "urban": asdict(urban.metrics),
        "centerline_energy_consistency": asdict(centerline_energy),
        "urban_energy_consistency": asdict(urban_energy),
        "energy_model_scope": (
            "Structural and numerical consistency only; absolute real-world accuracy still "
            "depends on replacing the illustrative efficiency map with sourced motor data."
        ),
    }
    (args.output_dir / "validation_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    (args.output_dir / "actuator_calibration.json").write_text(
        json.dumps([asdict(sample) for sample in calibration], indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(report, indent=2))
    if not report["overall_passed"]:
        raise SystemExit("validation failed; inspect validation_report.json")


if __name__ == "__main__":
    main()
