"""Conventional backward-facing EV hardware sizing without feedback-control metrics."""

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
from .powertrain import EVPowertrain, EnergyState  # noqa: E402
from .scenarios import (  # noqa: E402
    MIXED_GRADE_PROFILE,
    MIXED_GRADE_SPEED_PROFILE,
    RoadGradeProfile,
    SpeedProfile,
)


@dataclass(frozen=True, slots=True)
class SizingRequirements:
    top_speed_kph: float = 120.0
    maximum_zero_to_100_s: float = 10.0
    gradeability_fraction: float = 0.20
    gradeability_speed_kph: float = 30.0


@dataclass(frozen=True, slots=True)
class HardwareSizingResult:
    final_drive_ratio: float
    motor_scale: float
    motor_mass_kg: float
    cycle_wh_per_km: float
    cycle_net_battery_wh: float
    cycle_distance_m: float
    zero_to_100_s: float
    top_speed_feasible: bool
    acceleration_feasible: bool
    gradeability_feasible: bool
    cycle_feasible: bool
    maximum_motor_speed_rpm: float
    maximum_abs_motor_torque_nm: float
    feasible: bool
    selected: bool = False


def _sample_cycle(
    speed_profile: SpeedProfile,
    grade_profile: RoadGradeProfile,
    dt_s: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    times = np.arange(0.0, speed_profile.duration_s + 0.5 * dt_s, dt_s)
    speeds = np.asarray([speed_profile.reference_at(float(time)) for time in times])
    distance = np.cumsum(speeds * dt_s)
    grades = np.asarray([grade_profile.grade_at(float(value)) for value in distance])
    acceleration = np.gradient(speeds, dt_s)
    return times, speeds, grades, acceleration


def _backward_cycle(
    powertrain: EVPowertrain,
    speed_profile: SpeedProfile,
    grade_profile: RoadGradeProfile,
    dt_s: float,
) -> tuple[float, float, float, bool, float, float]:
    _, speeds, grades, acceleration = _sample_cycle(speed_profile, grade_profile, dt_s)
    energy = EnergyState()
    feasible = True
    maximum_speed_rpm = 0.0
    maximum_abs_torque_nm = 0.0
    for speed, grade, accel in zip(speeds, grades, acceleration):
        required_force = (
            powertrain.total_vehicle_mass_kg * float(accel)
            + powertrain.road_load_force(float(speed), float(grade))
        )
        step = powertrain.evaluate(required_force, float(speed))
        if required_force > 0.0 and step.applied_wheel_force_n + 1e-6 < required_force:
            feasible = False
        if step.speed_limited and required_force > 0.0:
            feasible = False
        energy.update(step, dt_s)
        maximum_speed_rpm = max(maximum_speed_rpm, step.motor_speed_rad_s * 60.0 / (2.0 * np.pi))
        maximum_abs_torque_nm = max(maximum_abs_torque_nm, abs(step.motor_torque_nm))
    distance_m = float(np.sum(speeds) * dt_s)
    wh_per_km = energy.net_battery_wh / (distance_m / 1000.0)
    return (
        energy.net_battery_wh,
        wh_per_km,
        distance_m,
        feasible,
        maximum_speed_rpm,
        maximum_abs_torque_nm,
    )


def _zero_to_speed_time(
    powertrain: EVPowertrain,
    target_speed_mps: float,
    maximum_time_s: float,
    dt_s: float = 0.02,
) -> float:
    speed = 0.0
    elapsed = 0.0
    while speed < target_speed_mps and elapsed <= maximum_time_s + dt_s:
        _, traction_limit = powertrain.force_limits(speed)
        net_force = traction_limit - powertrain.road_load_force(speed)
        speed = max(0.0, speed + max(0.0, net_force) / powertrain.total_vehicle_mass_kg * dt_s)
        elapsed += dt_s
        if traction_limit <= 0.0:
            return float("inf")
    return elapsed if speed >= target_speed_mps else float("inf")


def evaluate_hardware(
    config: ProjectConfig,
    hardware: HardwareDesign,
    requirements: SizingRequirements = SizingRequirements(),
    speed_profile: SpeedProfile = MIXED_GRADE_SPEED_PROFILE,
    grade_profile: RoadGradeProfile = MIXED_GRADE_PROFILE,
) -> HardwareSizingResult:
    powertrain = EVPowertrain(hardware, config.vehicle, config.motor, config.battery)
    required_top_speed = requirements.top_speed_kph / 3.6
    _, top_speed_traction = powertrain.force_limits(required_top_speed)
    top_speed_feasible = (
        powertrain.motor_speed(required_top_speed) <= powertrain.max_speed_rad_s
        and top_speed_traction >= powertrain.road_load_force(required_top_speed)
    )
    acceleration_time = _zero_to_speed_time(
        powertrain,
        100.0 / 3.6,
        requirements.maximum_zero_to_100_s,
    )
    acceleration_feasible = acceleration_time <= requirements.maximum_zero_to_100_s
    grade_speed = requirements.gradeability_speed_kph / 3.6
    _, grade_traction = powertrain.force_limits(grade_speed)
    gradeability_feasible = grade_traction >= powertrain.road_load_force(
        grade_speed, requirements.gradeability_fraction
    )
    (
        cycle_energy,
        wh_per_km,
        cycle_distance,
        cycle_feasible,
        maximum_speed_rpm,
        maximum_torque,
    ) = _backward_cycle(
        powertrain,
        speed_profile,
        grade_profile,
        config.control_interval_s,
    )
    feasible = (
        top_speed_feasible
        and acceleration_feasible
        and gradeability_feasible
        and cycle_feasible
    )
    return HardwareSizingResult(
        final_drive_ratio=hardware.final_drive_ratio,
        motor_scale=hardware.motor_scale,
        motor_mass_kg=powertrain.motor_mass_kg,
        cycle_wh_per_km=wh_per_km,
        cycle_net_battery_wh=cycle_energy,
        cycle_distance_m=cycle_distance,
        zero_to_100_s=acceleration_time,
        top_speed_feasible=top_speed_feasible,
        acceleration_feasible=acceleration_feasible,
        gradeability_feasible=gradeability_feasible,
        cycle_feasible=cycle_feasible,
        maximum_motor_speed_rpm=maximum_speed_rpm,
        maximum_abs_motor_torque_nm=maximum_torque,
        feasible=feasible,
    )


def size_hardware(
    config: ProjectConfig,
    final_drive_ratios: tuple[float, ...] = tuple(np.arange(6.0, 12.01, 0.5)),
    motor_scales: tuple[float, ...] = tuple(np.arange(0.6, 1.401, 0.1)),
) -> tuple[list[HardwareSizingResult], HardwareSizingResult]:
    results = [
        evaluate_hardware(config, HardwareDesign(float(ratio), float(scale)))
        for ratio in final_drive_ratios
        for scale in motor_scales
    ]
    feasible = [result for result in results if result.feasible]
    if not feasible:
        raise RuntimeError("hardware grid contains no feasible conventional design")
    minimum_energy = min(result.cycle_wh_per_km for result in feasible)
    near_minimum = [
        result for result in feasible if result.cycle_wh_per_km <= minimum_energy * 1.005
    ]
    selected = min(
        near_minimum,
        key=lambda result: (result.motor_mass_kg, result.cycle_wh_per_km, result.final_drive_ratio),
    )
    marked = [replace(result, selected=result == selected) for result in results]
    selected_marked = next(result for result in marked if result.selected)
    return marked, selected_marked


def _plot(results: list[HardwareSizingResult], output: Path) -> None:
    ratios = sorted({result.final_drive_ratio for result in results})
    scales = sorted({result.motor_scale for result in results})
    values = np.full((len(scales), len(ratios)), np.nan)
    for result in results:
        row = scales.index(result.motor_scale)
        column = ratios.index(result.final_drive_ratio)
        if result.feasible:
            values[row, column] = result.cycle_wh_per_km
    fig, axis = plt.subplots(figsize=(10.5, 6.5), constrained_layout=True)
    image = axis.imshow(values, origin="lower", aspect="auto", cmap="viridis")
    axis.set_xticks(range(len(ratios)), [f"{value:g}" for value in ratios])
    axis.set_yticks(range(len(scales)), [f"{value:.1f}" for value in scales])
    axis.set(xlabel="Final-drive ratio", ylabel="Motor scale", title="Conventional hardware sizing")
    selected = next(result for result in results if result.selected)
    axis.scatter(
        [ratios.index(selected.final_drive_ratio)],
        [scales.index(selected.motor_scale)],
        marker="*",
        s=260,
        color="red",
        edgecolor="white",
        label="Selected",
    )
    axis.legend(loc="upper right")
    colorbar = fig.colorbar(image, ax=axis)
    colorbar.set_label("Backward-cycle net energy [Wh/km]")
    fig.savefig(output, dpi=180)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=Path("configs/default.yaml"))
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/hardware_sizing"))
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    config = ProjectConfig.from_yaml(args.config)
    results, selected = size_hardware(config)
    with (args.output_dir / "hardware_sizing.csv").open(
        "w", encoding="utf-8", newline=""
    ) as handle:
        writer = csv.DictWriter(handle, fieldnames=list(asdict(results[0]).keys()))
        writer.writeheader()
        writer.writerows(asdict(result) for result in results)
    report = {
        "requirements": asdict(SizingRequirements()),
        "selected": asdict(selected),
        "candidate_count": len(results),
        "feasible_count": sum(result.feasible for result in results),
    }
    (args.output_dir / "hardware_sizing_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    _plot(results, args.output_dir / "hardware_sizing_map.png")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
