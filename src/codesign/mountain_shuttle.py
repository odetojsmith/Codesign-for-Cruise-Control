"""Closed-loop mountain-shuttle experiment exposing hardware/controller coupling."""

from __future__ import annotations

import argparse
import csv
import json
from dataclasses import asdict, dataclass, replace
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from .config import HardwareDesign, ProjectConfig  # noqa: E402
from .controllers import CenterlinePIDController  # noqa: E402
from .hardware_sizing import size_hardware  # noqa: E402
from .metadrive_env import MetaDriveEVEnv  # noqa: E402
from .mpc import LongitudinalMPCController  # noqa: E402
from .optimization import ControllerDesign  # noqa: E402
from .powertrain import EVPowertrain  # noqa: E402
from .scenarios import EpisodeResult, RoadGradeProfile, SpeedProfile, run_speed_profile  # noqa: E402


def _mountain_profile(cycles: int = 4) -> SpeedProfile:
    times: list[float] = []
    speeds: list[float] = []
    for cycle in range(cycles):
        start = cycle * 25.0
        segment = (
            (start, 0.0),
            (start + 3.0, 0.0),
            (start + 10.0, 15.0),
            (start + 16.0, 15.0),
            (start + 22.0, 0.0),
            (start + 25.0, 0.0),
        )
        if cycle:
            segment = segment[1:]
        times.extend(point[0] for point in segment)
        speeds.extend(point[1] for point in segment)
    return SpeedProfile("autonomous_mountain_shuttle", tuple(times), tuple(speeds))


def _mountain_grade(cycles: int = 4) -> RoadGradeProfile:
    distances: list[float] = []
    grades: list[float] = []
    cycle_distance = 187.5
    for cycle in range(cycles):
        start = cycle * cycle_distance
        segment = (
            (start, 0.0),
            (start + 10.0, 0.0),
            (start + 20.0, 0.10),
            (start + 105.0, 0.10),
            (start + 125.0, 0.0),
            (start + 135.0, -0.10),
            (start + 177.5, -0.10),
            (start + cycle_distance, 0.0),
        )
        if cycle:
            segment = segment[1:]
        distances.extend(point[0] for point in segment)
        grades.extend(point[1] for point in segment)
    return RoadGradeProfile("repeated_ten_percent_hills", tuple(distances), tuple(grades))


MOUNTAIN_SHUTTLE_PROFILE = _mountain_profile()
MOUNTAIN_SHUTTLE_GRADE = _mountain_grade()
REFERENCE_DISTANCE_M = 4 * 187.5
STATION_TIMES_S = (22.0, 47.0, 72.0, 97.0, 100.0)


@dataclass(frozen=True, slots=True)
class MountainResult:
    final_drive_ratio: float
    motor_scale: float
    log10_lambda_energy: float
    log10_lambda_force_slew: float
    rmse_mps: float
    net_battery_wh: float
    wh_per_km: float
    distance_m: float
    terminal_progress_fraction: float
    maximum_station_distance_error_m: float
    maximum_station_speed_mps: float
    peak_motor_temperature_c: float
    thermal_derating_fraction: float
    saturation_fraction: float
    battery_limit_fraction: float
    friction_brake_wh: float
    regenerated_wh: float
    completed: bool
    feasible: bool
    violations: tuple[str, ...]


def mountain_config(config: ProjectConfig) -> ProjectConfig:
    """Enable constraints that are important for a repeated hill duty cycle."""

    return replace(
        config,
        battery=replace(
            config.battery,
            maximum_discharge_power_kw=90.0,
            maximum_charge_power_kw=45.0,
        ),
        thermal=replace(
            config.thermal,
            enabled=True,
            ambient_temperature_c=30.0,
            initial_temperature_c=55.0,
            base_thermal_capacity_j_per_k=28_000.0,
            base_thermal_conductance_w_per_k=85.0,
            derating_start_temperature_c=82.0,
            maximum_temperature_c=112.0,
            minimum_torque_fraction=0.30,
        ),
    )


def _station_metrics(result: EpisodeResult) -> tuple[float, float]:
    expected = (187.5, 375.0, 562.5, 750.0, 750.0)
    distance_errors: list[float] = []
    speeds: list[float] = []
    for time_s, expected_distance in zip(STATION_TIMES_S, expected):
        point = min(result.trajectory, key=lambda item: abs(item.time_s - time_s))
        distance_errors.append(abs(point.distance_m - expected_distance))
        speeds.append(point.speed_mps)
    return max(distance_errors), max(speeds)


def evaluate_mountain_design(
    config: ProjectConfig,
    hardware: HardwareDesign,
    controller_design: ControllerDesign,
    *,
    keep_trajectory: bool = False,
) -> tuple[MountainResult, EpisodeResult | None]:
    run_config = mountain_config(replace(config, hardware=hardware))
    powertrain = EVPowertrain(
        hardware,
        run_config.vehicle,
        run_config.motor,
        run_config.battery,
        run_config.thermal,
    )
    env = MetaDriveEVEnv(
        powertrain,
        control_interval_s=run_config.control_interval_s,
        seed=run_config.seed,
        map_sequence="S" * 45,
        spawn_lateral_m=0.5,
        road_grade_profile=MOUNTAIN_SHUTTLE_GRADE.grade_at,
        maximum_abs_grade_fraction=0.10,
    )
    controller = LongitudinalMPCController(
        powertrain,
        dt_s=run_config.control_interval_s,
        horizon_steps=25,
        lambda_energy=10.0**controller_design.log10_lambda_energy,
        lambda_force_slew=10.0**controller_design.log10_lambda_force_slew,
    )
    try:
        episode = run_speed_profile(
            env,
            MOUNTAIN_SHUTTLE_PROFILE,
            controller,
            CenterlinePIDController(dt_s=run_config.control_interval_s),
            preview_steps=25,
        )
    finally:
        env.close()

    dt = run_config.control_interval_s
    friction_wh = sum(
        max(0.0, -point.friction_brake_force_n * point.speed_mps) * dt / 3600.0
        for point in episode.trajectory
    )
    station_error, station_speed = _station_metrics(episode)
    progress = episode.metrics.distance_m / REFERENCE_DISTANCE_M
    peak_temperature = max(point.motor_temperature_c for point in episode.trajectory)
    minimum_derating = min(point.thermal_derating_factor for point in episode.trajectory)
    battery_limit_fraction = sum(point.battery_power_limited for point in episode.trajectory) / len(
        episode.trajectory
    )
    violations: list[str] = []
    if not episode.metrics.completed:
        violations.append("incomplete_episode")
    if episode.metrics.rmse_mps > 0.75:
        violations.append("tracking_rmse")
    if progress < 0.985:
        violations.append("terminal_progress")
    if station_error > 12.0:
        violations.append("station_distance")
    if station_speed > 1.25:
        violations.append("station_stop_speed")
    if peak_temperature > run_config.thermal.maximum_temperature_c:
        violations.append("motor_temperature")
    if controller.fallback_count:
        violations.append("mpc_fallback")

    summary = MountainResult(
        final_drive_ratio=hardware.final_drive_ratio,
        motor_scale=hardware.motor_scale,
        log10_lambda_energy=controller_design.log10_lambda_energy,
        log10_lambda_force_slew=controller_design.log10_lambda_force_slew,
        rmse_mps=episode.metrics.rmse_mps,
        net_battery_wh=episode.metrics.net_battery_wh,
        wh_per_km=episode.metrics.wh_per_km,
        distance_m=episode.metrics.distance_m,
        terminal_progress_fraction=progress,
        maximum_station_distance_error_m=station_error,
        maximum_station_speed_mps=station_speed,
        peak_motor_temperature_c=peak_temperature,
        thermal_derating_fraction=1.0 - minimum_derating,
        saturation_fraction=episode.metrics.saturation_fraction,
        battery_limit_fraction=battery_limit_fraction,
        friction_brake_wh=friction_wh,
        regenerated_wh=sum(
            max(0.0, -point.battery_power_w) * dt / 3600.0
            for point in episode.trajectory
        ),
        completed=episode.metrics.completed,
        feasible=not violations,
        violations=tuple(violations),
    )
    return summary, episode if keep_trajectory else None


def _best(results: list[MountainResult]) -> MountainResult | None:
    feasible = [result for result in results if result.feasible]
    return min(feasible, key=lambda item: item.net_battery_wh) if feasible else None


def _write_results(results: list[MountainResult], path: Path) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(asdict(results[0]).keys()))
        writer.writeheader()
        for result in results:
            row = asdict(result)
            row["violations"] = ";".join(result.violations)
            writer.writerow(row)


def _plot_hardware_map(results: list[MountainResult], conventional: HardwareDesign, output: Path) -> None:
    ratios = sorted({result.final_drive_ratio for result in results})
    scales = sorted({result.motor_scale for result in results})
    values = np.full((len(scales), len(ratios)), np.nan)
    for scale_index, scale in enumerate(scales):
        for ratio_index, ratio in enumerate(ratios):
            best = _best(
                [
                    result
                    for result in results
                    if result.final_drive_ratio == ratio and result.motor_scale == scale
                ]
            )
            if best is not None:
                values[scale_index, ratio_index] = best.net_battery_wh
    fig, axis = plt.subplots(figsize=(10.5, 6.2), constrained_layout=True)
    image = axis.imshow(values, origin="lower", aspect="auto", cmap="viridis")
    axis.set_xticks(range(len(ratios)), [f"{value:g}" for value in ratios])
    axis.set_yticks(range(len(scales)), [f"{value:g}" for value in scales])
    axis.set(
        xlabel="Final-drive ratio",
        ylabel="Motor scale",
        title="Mountain shuttle: minimum feasible closed-loop energy",
    )
    if conventional.final_drive_ratio in ratios and conventional.motor_scale in scales:
        axis.scatter(
            ratios.index(conventional.final_drive_ratio),
            scales.index(conventional.motor_scale),
            marker="x",
            s=180,
            linewidths=3,
            color="white",
            label="Traditional sizing",
        )
    best = _best(results)
    if best is not None:
        axis.scatter(
            ratios.index(best.final_drive_ratio),
            scales.index(best.motor_scale),
            marker="*",
            s=260,
            color="red",
            edgecolor="white",
            label="Closed-loop co-design",
        )
    axis.legend(loc="upper right")
    colorbar = fig.colorbar(image, ax=axis)
    colorbar.set_label("Net battery energy [Wh]")
    fig.savefig(output, dpi=180)
    plt.close(fig)


def _plot_comparison(
    separate: tuple[MountainResult, EpisodeResult],
    codesign: tuple[MountainResult, EpisodeResult],
    output: Path,
) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(11.5, 10.0), sharex=True, constrained_layout=True)
    for (summary, episode), color, label in (
        (separate, "tab:blue", "Traditional hardware + tuned MPC"),
        (codesign, "tab:orange", "Hardware-controller co-design"),
    ):
        time = [point.time_s for point in episode.trajectory]
        axes[0].plot(time, [point.speed_mps for point in episode.trajectory], color=color, label=label)
        axes[1].plot(
            time,
            [point.motor_temperature_c for point in episode.trajectory],
            color=color,
            label=f"{label} ({summary.peak_motor_temperature_c:.1f} °C peak)",
        )
        axes[2].plot(
            time,
            [point.cumulative_battery_wh for point in episode.trajectory],
            color=color,
            label=f"{label} ({summary.net_battery_wh:.1f} Wh)",
        )
    reference_episode = separate[1]
    axes[0].plot(
        [point.time_s for point in reference_episode.trajectory],
        [point.reference_speed_mps for point in reference_episode.trajectory],
        "k--",
        linewidth=1.3,
        label="Reference",
    )
    axes[0].set_ylabel("Speed [m/s]")
    axes[1].axhline(82.0, color="tab:red", linestyle="--", linewidth=1.0, label="Derating starts")
    axes[1].set_ylabel("Motor temperature [°C]")
    axes[2].set(xlabel="Time [s]", ylabel="Net battery energy [Wh]")
    for axis in axes:
        axis.grid(alpha=0.25)
        axis.legend(loc="best")
    fig.suptitle("Autonomous mountain shuttle: separate design versus co-design")
    fig.savefig(output, dpi=180)
    plt.close(fig)


def run_mountain_experiment(config: ProjectConfig, output_dir: Path, quick: bool = False) -> dict[str, object]:
    output_dir.mkdir(parents=True, exist_ok=True)
    _, conventional_sizing = size_hardware(config)
    conventional = HardwareDesign(
        conventional_sizing.final_drive_ratio, conventional_sizing.motor_scale
    )
    if quick:
        ratios = (7.0, 8.5, 10.0, 10.5, 11.5)
        scales = (0.6, 0.75, 0.9, 1.05)
        controllers = (
            ControllerDesign(-1.0, -1.0),
            ControllerDesign(0.5, -1.0),
            ControllerDesign(1.5, -0.5),
        )
    else:
        ratios = tuple(np.arange(7.0, 12.01, 0.5))
        scales = tuple(np.arange(0.6, 1.051, 0.075))
        controllers = tuple(
            ControllerDesign(energy, slew)
            for energy in (-1.5, -0.5, 0.5, 1.5)
            for slew in (-1.5, -0.5, 0.5)
        )
    hardware = tuple(HardwareDesign(ratio, scale) for ratio in ratios for scale in scales)
    if conventional not in hardware:
        hardware += (conventional,)

    results: list[MountainResult] = []
    total = len(hardware) * len(controllers)
    for index, design in enumerate(hardware, start=1):
        for controller in controllers:
            summary, _ = evaluate_mountain_design(config, design, controller)
            results.append(summary)
        print(f"[{index * len(controllers):3d}/{total}] g={design.final_drive_ratio:g}, s={design.motor_scale:g}")

    separate_candidates = [result for result in results if result.final_drive_ratio == conventional.final_drive_ratio and result.motor_scale == conventional.motor_scale]
    separate_best = _best(separate_candidates)
    codesign_best = _best(results)
    if separate_best is None:
        separate_best = min(separate_candidates, key=lambda item: item.rmse_mps)
    if codesign_best is None:
        codesign_best = min(results, key=lambda item: item.rmse_mps)

    separate_summary, separate_episode = evaluate_mountain_design(
        config,
        conventional,
        ControllerDesign(
            separate_best.log10_lambda_energy, separate_best.log10_lambda_force_slew
        ),
        keep_trajectory=True,
    )
    codesign_hardware = HardwareDesign(codesign_best.final_drive_ratio, codesign_best.motor_scale)
    codesign_summary, codesign_episode = evaluate_mountain_design(
        config,
        codesign_hardware,
        ControllerDesign(
            codesign_best.log10_lambda_energy, codesign_best.log10_lambda_force_slew
        ),
        keep_trajectory=True,
    )
    assert separate_episode is not None and codesign_episode is not None

    _write_results(results, output_dir / "mountain_shuttle_results.csv")
    separate_episode.write_csv(output_dir / "traditional_trajectory.csv")
    codesign_episode.write_csv(output_dir / "codesign_trajectory.csv")
    _plot_hardware_map(results, conventional, output_dir / "mountain_hardware_map.png")
    _plot_comparison(
        (separate_summary, separate_episode),
        (codesign_summary, codesign_episode),
        output_dir / "mountain_separate_vs_codesign.png",
    )
    improvement = (
        (separate_summary.net_battery_wh - codesign_summary.net_battery_wh)
        / separate_summary.net_battery_wh
        * 100.0
    )
    report: dict[str, object] = {
        "mode": "quick" if quick else "full",
        "scenario": {
            "cycles": 4,
            "maximum_grade_fraction": 0.10,
            "reference_distance_m": REFERENCE_DISTANCE_M,
            "duration_s": MOUNTAIN_SHUTTLE_PROFILE.duration_s,
            "rmse_limit_mps": 0.75,
            "minimum_terminal_progress_fraction": 0.985,
            "maximum_station_distance_error_m": 12.0,
            "maximum_station_speed_mps": 1.25,
        },
        "traditional_hardware": asdict(conventional),
        "separate_design": asdict(separate_summary),
        "codesign": asdict(codesign_summary),
        "energy_improvement_percent": improvement,
        "sample_count": len(results),
        "feasible_count": sum(result.feasible for result in results),
    }
    (output_dir / "mountain_shuttle_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    return report


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=Path("configs/default.yaml"))
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/mountain_shuttle"))
    parser.add_argument("--quick", action="store_true")
    args = parser.parse_args()
    report = run_mountain_experiment(
        ProjectConfig.from_yaml(args.config), args.output_dir, quick=args.quick
    )
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
