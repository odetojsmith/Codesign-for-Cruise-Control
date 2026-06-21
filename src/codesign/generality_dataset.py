"""Train/test scenario dataset for hardware-controller co-design generalization."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from dataclasses import asdict, dataclass, replace
from math import ceil
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from .config import HardwareDesign, ProjectConfig  # noqa: E402
from .controllers import CenterlinePIDController  # noqa: E402
from .hardware_sizing import evaluate_hardware, size_hardware  # noqa: E402
from .metadrive_env import MetaDriveEVEnv  # noqa: E402
from .mpc import LongitudinalMPCController  # noqa: E402
from .optimization import ControllerDesign  # noqa: E402
from .powertrain import EVPowertrain  # noqa: E402
from .scenarios import RoadGradeProfile, SpeedProfile, run_speed_profile  # noqa: E402


@dataclass(frozen=True, slots=True)
class ScenarioDefinition:
    name: str
    split: str
    seed: int
    cycles: int
    cruise_speed_mps: float
    dwell_s: float
    launch_s: float
    cruise_s: float
    braking_s: float
    uphill_grade: float
    downhill_grade: float
    payload_kg: float = 0.0
    drag_multiplier: float = 1.0
    initial_motor_temperature_c: float = 55.0
    discharge_power_kw: float = 90.0
    charge_power_kw: float = 45.0
    rmse_limit_mps: float = 0.40

    def __post_init__(self) -> None:
        if self.split not in {"train", "test"}:
            raise ValueError("scenario split must be train or test")
        if self.cycles < 1 or self.cruise_speed_mps <= 0:
            raise ValueError("scenario cycles and speed must be positive")
        if self.downhill_grade >= 0 or self.uphill_grade <= 0:
            raise ValueError("scenario must contain positive uphill and negative downhill grades")

    @property
    def cycle_duration_s(self) -> float:
        return 2.0 * self.dwell_s + self.launch_s + self.cruise_s + self.braking_s

    @property
    def duration_s(self) -> float:
        return self.cycles * self.cycle_duration_s

    @property
    def cycle_distance_m(self) -> float:
        return self.cruise_speed_mps * (
            0.5 * self.launch_s + self.cruise_s + 0.5 * self.braking_s
        )

    @property
    def reference_distance_m(self) -> float:
        return self.cycles * self.cycle_distance_m

    def speed_profile(self) -> SpeedProfile:
        times: list[float] = []
        speeds: list[float] = []
        for cycle in range(self.cycles):
            start = cycle * self.cycle_duration_s
            movement_end = (
                start + self.dwell_s + self.launch_s + self.cruise_s + self.braking_s
            )
            segment = (
                (start, 0.0),
                (start + self.dwell_s, 0.0),
                (start + self.dwell_s + self.launch_s, self.cruise_speed_mps),
                (
                    start + self.dwell_s + self.launch_s + self.cruise_s,
                    self.cruise_speed_mps,
                ),
                (movement_end, 0.0),
                (start + self.cycle_duration_s, 0.0),
            )
            if cycle:
                segment = segment[1:]
            times.extend(point[0] for point in segment)
            speeds.extend(point[1] for point in segment)
        return SpeedProfile(self.name, tuple(times), tuple(speeds))

    def grade_profile(self) -> RoadGradeProfile:
        distances: list[float] = []
        grades: list[float] = []
        for cycle in range(self.cycles):
            start = cycle * self.cycle_distance_m
            distance = self.cycle_distance_m
            segment = (
                (start, 0.0),
                (start + 0.05 * distance, 0.0),
                (start + 0.12 * distance, self.uphill_grade),
                (start + 0.55 * distance, self.uphill_grade),
                (start + 0.65 * distance, 0.0),
                (start + 0.72 * distance, self.downhill_grade),
                (start + 0.95 * distance, self.downhill_grade),
                (start + distance, 0.0),
            )
            if cycle:
                segment = segment[1:]
            distances.extend(point[0] for point in segment)
            grades.extend(point[1] for point in segment)
        return RoadGradeProfile(f"{self.name}_grade", tuple(distances), tuple(grades))

    def station_checks(self) -> tuple[tuple[float, float], ...]:
        checks = []
        for cycle in range(self.cycles):
            movement_end = (
                cycle * self.cycle_duration_s
                + self.dwell_s
                + self.launch_s
                + self.cruise_s
                + self.braking_s
            )
            checks.append((movement_end, (cycle + 1) * self.cycle_distance_m))
        checks.append((self.duration_s, self.reference_distance_m))
        return tuple(checks)


SCENARIO_DATASET: tuple[ScenarioDefinition, ...] = (
    ScenarioDefinition("train_alpine_commuter", "train", 11, 2, 15.0, 3.0, 7.0, 6.0, 6.0, 0.10, -0.10),
    ScenarioDefinition(
        "train_loaded_delivery", "train", 12, 2, 13.0, 4.0, 7.0, 8.0, 6.0, 0.12, -0.08,
        payload_kg=150.0, charge_power_kw=40.0,
    ),
    ScenarioDefinition(
        "train_fast_foothills", "train", 13, 2, 18.0, 2.0, 8.0, 5.0, 7.0, 0.07, -0.09,
        payload_kg=50.0, drag_multiplier=1.08, charge_power_kw=50.0,
    ),
    ScenarioDefinition(
        "train_hot_resort", "train", 14, 3, 14.0, 3.0, 7.0, 5.0, 7.0, 0.09, -0.12,
        payload_kg=100.0, initial_motor_temperature_c=65.0, charge_power_kw=35.0,
    ),
    ScenarioDefinition(
        "test_unseen_steep", "test", 101, 2, 16.0, 3.0, 8.0, 6.0, 7.0, 0.13, -0.11,
        payload_kg=75.0, drag_multiplier=1.05, charge_power_kw=42.0,
    ),
    ScenarioDefinition(
        "test_heavy_descent", "test", 102, 2, 14.5, 3.5, 7.0, 7.0, 7.0, 0.10, -0.13,
        payload_kg=225.0, charge_power_kw=35.0,
    ),
    ScenarioDefinition(
        "test_long_fast_shift", "test", 103, 3, 17.0, 2.5, 7.5, 7.0, 6.5, 0.08, -0.10,
        payload_kg=25.0, drag_multiplier=1.10, initial_motor_temperature_c=60.0,
        discharge_power_kw=85.0, charge_power_kw=48.0,
    ),
)


@dataclass(frozen=True, slots=True)
class ScenarioEvaluation:
    scenario: str
    split: str
    final_drive_ratio: float
    motor_scale: float
    log10_lambda_energy: float
    log10_lambda_force_slew: float
    rmse_mps: float
    net_battery_wh: float
    wh_per_km: float
    distance_m: float
    progress_fraction: float
    maximum_station_distance_error_m: float
    maximum_station_speed_mps: float
    peak_motor_temperature_c: float
    friction_brake_wh: float
    recovered_battery_wh: float
    fallback_count: int
    feasible: bool
    violations: tuple[str, ...]

    @classmethod
    def from_dict(cls, values: dict[str, object]) -> "ScenarioEvaluation":
        data = dict(values)
        data["violations"] = tuple(data["violations"])
        return cls(**data)  # type: ignore[arg-type]


class EvaluationStore:
    def __init__(self, path: Path, config: ProjectConfig):
        self.path = path
        self.config = config
        self.values: dict[str, dict[str, object]] = {}
        if path.exists():
            self.values = json.loads(path.read_text(encoding="utf-8"))

    def key(
        self,
        scenario: ScenarioDefinition,
        hardware: HardwareDesign,
        controller: ControllerDesign,
    ) -> str:
        payload = json.dumps(
            {
                "schema": 2,
                "project_config": asdict(self.config),
                "scenario": asdict(scenario),
                "hardware": asdict(hardware),
                "controller": asdict(controller),
            },
            sort_keys=True,
        )
        return hashlib.sha256(payload.encode("utf-8")).hexdigest()

    def get(
        self,
        scenario: ScenarioDefinition,
        hardware: HardwareDesign,
        controller: ControllerDesign,
    ) -> ScenarioEvaluation | None:
        value = self.values.get(self.key(scenario, hardware, controller))
        return None if value is None else ScenarioEvaluation.from_dict(value)

    def put(
        self,
        scenario: ScenarioDefinition,
        hardware: HardwareDesign,
        controller: ControllerDesign,
        result: ScenarioEvaluation,
        *,
        persist: bool = True,
    ) -> None:
        self.values[self.key(scenario, hardware, controller)] = asdict(result)
        if persist:
            self.flush()

    def flush(self) -> None:
        self.path.write_text(json.dumps(self.values, indent=2) + "\n", encoding="utf-8")


def _scenario_config(config: ProjectConfig, scenario: ScenarioDefinition) -> ProjectConfig:
    return replace(
        config,
        vehicle=replace(
            config.vehicle,
            base_mass_kg=config.vehicle.base_mass_kg + scenario.payload_kg,
            drag_coefficient=config.vehicle.drag_coefficient * scenario.drag_multiplier,
        ),
        battery=replace(
            config.battery,
            maximum_discharge_power_kw=scenario.discharge_power_kw,
            maximum_charge_power_kw=scenario.charge_power_kw,
        ),
        thermal=replace(
            config.thermal,
            enabled=True,
            ambient_temperature_c=30.0,
            initial_temperature_c=scenario.initial_motor_temperature_c,
            base_thermal_capacity_j_per_k=28_000.0,
            base_thermal_conductance_w_per_k=85.0,
            derating_start_temperature_c=82.0,
            maximum_temperature_c=112.0,
            minimum_torque_fraction=0.30,
        ),
        seed=scenario.seed,
    )


def evaluate_scenario(
    config: ProjectConfig,
    scenario: ScenarioDefinition,
    hardware: HardwareDesign,
    controller_design: ControllerDesign,
) -> ScenarioEvaluation:
    run_config = _scenario_config(replace(config, hardware=hardware), scenario)
    profile = scenario.speed_profile()
    grade = scenario.grade_profile()
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
        seed=scenario.seed,
        map_sequence="S" * max(20, ceil(scenario.reference_distance_m / 18.0)),
        spawn_lateral_m=0.5,
        road_grade_profile=grade.grade_at,
        maximum_abs_grade_fraction=max(abs(scenario.uphill_grade), abs(scenario.downhill_grade)),
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
            profile,
            controller,
            CenterlinePIDController(dt_s=run_config.control_interval_s),
            preview_steps=25,
        )
    finally:
        env.close()

    station_errors: list[float] = []
    station_speeds: list[float] = []
    for time_s, expected_distance in scenario.station_checks():
        point = min(episode.trajectory, key=lambda item: abs(item.time_s - time_s))
        station_errors.append(abs(point.distance_m - expected_distance))
        station_speeds.append(point.speed_mps)
    progress = episode.metrics.distance_m / scenario.reference_distance_m
    peak_temperature = max(point.motor_temperature_c for point in episode.trajectory)
    dt = run_config.control_interval_s
    friction_wh = sum(
        max(0.0, -point.friction_brake_force_n * point.speed_mps) * dt / 3600.0
        for point in episode.trajectory
    )
    recovered_wh = sum(
        max(0.0, -point.battery_power_w) * dt / 3600.0 for point in episode.trajectory
    )
    violations: list[str] = []
    if not episode.metrics.completed:
        violations.append("incomplete_episode")
    if episode.metrics.rmse_mps > scenario.rmse_limit_mps:
        violations.append("tracking_rmse")
    if progress < 0.985:
        violations.append("terminal_progress")
    if max(station_errors) > 12.0:
        violations.append("station_distance")
    if max(station_speeds) > 1.5:
        violations.append("station_stop_speed")
    if peak_temperature > run_config.thermal.maximum_temperature_c:
        violations.append("motor_temperature")
    if controller.fallback_count:
        violations.append("mpc_fallback")
    return ScenarioEvaluation(
        scenario=scenario.name,
        split=scenario.split,
        final_drive_ratio=hardware.final_drive_ratio,
        motor_scale=hardware.motor_scale,
        log10_lambda_energy=controller_design.log10_lambda_energy,
        log10_lambda_force_slew=controller_design.log10_lambda_force_slew,
        rmse_mps=episode.metrics.rmse_mps,
        net_battery_wh=episode.metrics.net_battery_wh,
        wh_per_km=episode.metrics.wh_per_km,
        distance_m=episode.metrics.distance_m,
        progress_fraction=progress,
        maximum_station_distance_error_m=max(station_errors),
        maximum_station_speed_mps=max(station_speeds),
        peak_motor_temperature_c=peak_temperature,
        friction_brake_wh=friction_wh,
        recovered_battery_wh=recovered_wh,
        fallback_count=controller.fallback_count,
        feasible=not violations,
        violations=tuple(violations),
    )


def select_controller(results: list[ScenarioEvaluation]) -> ScenarioEvaluation | None:
    feasible = [result for result in results if result.feasible]
    return min(feasible, key=lambda result: result.wh_per_km) if feasible else None


def select_training_hardware(
    selections: dict[tuple[float, float, str], ScenarioEvaluation],
    hardware_candidates: tuple[HardwareDesign, ...],
    training_scenarios: tuple[ScenarioDefinition, ...],
) -> tuple[HardwareDesign, list[dict[str, object]]]:
    summaries: list[dict[str, object]] = []
    for hardware in hardware_candidates:
        selected = [
            selections.get((hardware.final_drive_ratio, hardware.motor_scale, scenario.name))
            for scenario in training_scenarios
        ]
        feasible = all(result is not None for result in selected)
        valid = [result for result in selected if result is not None]
        summaries.append(
            {
                "final_drive_ratio": hardware.final_drive_ratio,
                "motor_scale": hardware.motor_scale,
                "feasible_scenario_count": len(valid),
                "training_scenario_count": len(training_scenarios),
                "feasible": feasible,
                "mean_wh_per_km": (
                    float(np.mean([result.wh_per_km for result in valid]))
                    if feasible
                    else float("inf")
                ),
                "mean_rmse_mps": (
                    float(np.mean([result.rmse_mps for result in valid]))
                    if feasible
                    else float("inf")
                ),
                "maximum_rmse_mps": (
                    max(result.rmse_mps for result in valid) if valid else float("inf")
                ),
            }
        )
    feasible_summaries = [summary for summary in summaries if summary["feasible"]]
    if not feasible_summaries:
        raise RuntimeError("no hardware design is feasible on every training scenario")
    best = min(feasible_summaries, key=lambda item: float(item["mean_wh_per_km"]))
    return HardwareDesign(float(best["final_drive_ratio"]), float(best["motor_scale"])), summaries


def training_hardware_pareto_flags(summaries: list[dict[str, object]]) -> list[bool]:
    """Mark feasible hardware summaries nondominated in mean RMSE and energy."""
    flags: list[bool] = []
    for candidate in summaries:
        if not bool(candidate["feasible"]):
            flags.append(False)
            continue
        rmse = float(candidate["mean_rmse_mps"])
        energy = float(candidate["mean_wh_per_km"])
        dominated = any(
            bool(other["feasible"])
            and other is not candidate
            and float(other["mean_rmse_mps"]) <= rmse
            and float(other["mean_wh_per_km"]) <= energy
            and (
                float(other["mean_rmse_mps"]) < rmse
                or float(other["mean_wh_per_km"]) < energy
            )
            for other in summaries
        )
        flags.append(not dominated)
    return flags


def _write_csv(rows: list[dict[str, object]], path: Path) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def _plot_dataset(scenarios: tuple[ScenarioDefinition, ...], output: Path) -> None:
    fig, axes = plt.subplots(2, 1, figsize=(11.5, 8.0), constrained_layout=True)
    for scenario in scenarios:
        profile = scenario.speed_profile()
        grade = scenario.grade_profile()
        style = "-" if scenario.split == "train" else "--"
        axes[0].plot(profile.time_s, profile.speed_mps, style, label=scenario.name)
        normalized_distance = np.asarray(grade.distance_m) / scenario.reference_distance_m
        axes[1].plot(normalized_distance, np.asarray(grade.grade_fraction) * 100.0, style)
    axes[0].set(xlabel="Time [s]", ylabel="Reference speed [m/s]")
    axes[1].set(xlabel="Normalized route distance", ylabel="Road grade [%]")
    axes[0].legend(ncol=2, fontsize=8)
    for axis in axes:
        axis.grid(alpha=0.25)
    fig.suptitle("Generalization dataset: solid=train, dashed=test")
    fig.savefig(output, dpi=180)
    plt.close(fig)


def _plot_training_map(
    summaries: list[dict[str, object]], selected: HardwareDesign, output: Path
) -> None:
    ratios = sorted({float(row["final_drive_ratio"]) for row in summaries})
    scales = sorted({float(row["motor_scale"]) for row in summaries})
    values = np.full((len(scales), len(ratios)), np.nan)
    for row in summaries:
        if row["feasible"]:
            values[scales.index(float(row["motor_scale"])), ratios.index(float(row["final_drive_ratio"]))] = float(row["mean_wh_per_km"])
    fig, axis = plt.subplots(figsize=(10, 5.8), constrained_layout=True)
    image = axis.imshow(values, origin="lower", aspect="auto", cmap="viridis")
    axis.set_xticks(range(len(ratios)), [f"{value:g}" for value in ratios])
    axis.set_yticks(range(len(scales)), [f"{value:g}" for value in scales])
    axis.scatter(
        ratios.index(selected.final_drive_ratio), scales.index(selected.motor_scale),
        marker="*", s=260, color="red", edgecolor="white", label="Training-selected hardware",
    )
    axis.set(xlabel="Final-drive ratio", ylabel="Motor scale", title="Mean training energy after per-scenario MPC tuning")
    axis.legend()
    fig.colorbar(image, ax=axis, label="Mean Wh/km")
    fig.savefig(output, dpi=180)
    plt.close(fig)


def _plot_training_pareto(
    summaries: list[dict[str, object]], traditional: HardwareDesign, output: Path
) -> None:
    feasible = [row for row in summaries if bool(row["feasible"])]
    frontier = sorted(
        [row for row, flag in zip(summaries, training_hardware_pareto_flags(summaries)) if flag],
        key=lambda row: float(row["mean_rmse_mps"]),
    )
    traditional_row = next(
        row for row in feasible
        if float(row["final_drive_ratio"]) == traditional.final_drive_ratio
        and float(row["motor_scale"]) == traditional.motor_scale
    )
    fig, axis = plt.subplots(figsize=(10.5, 6.8), constrained_layout=True)
    scatter = axis.scatter(
        [float(row["mean_rmse_mps"]) for row in feasible],
        [float(row["mean_wh_per_km"]) for row in feasible],
        c=[float(row["motor_scale"]) for row in feasible], cmap="viridis", s=75,
        alpha=0.72, edgecolor="white", linewidth=0.8,
        label="Hardware with its tuned MPC", zorder=2,
    )
    axis.scatter(
        [float(row["mean_rmse_mps"]) for row in frontier],
        [float(row["mean_wh_per_km"]) for row in frontier],
        marker="D", color="tab:orange", edgecolor="black", linewidth=0.8, s=105,
        label="Nondominated hardware samples", zorder=3,
    )
    for row in frontier:
        axis.annotate(
            f"g={float(row['final_drive_ratio']):g}, $s_m$={float(row['motor_scale']):.2f}",
            xy=(float(row["mean_rmse_mps"]), float(row["mean_wh_per_km"])),
            xytext=(5, -17), textcoords="offset points", fontsize=8.5, color="saddlebrown",
        )
    traditional_rmse = float(traditional_row["mean_rmse_mps"])
    traditional_energy = float(traditional_row["mean_wh_per_km"])
    axis.scatter(
        [traditional_rmse], [traditional_energy], marker="*", s=430, color="crimson",
        edgecolor="white", linewidth=1.5,
        label=f"Traditional hardware (g={traditional.final_drive_ratio:g}, $s_m$={traditional.motor_scale:.2f})",
        zorder=5,
    )
    dominating = [
        row for row in frontier
        if float(row["mean_rmse_mps"]) <= traditional_rmse
        and float(row["mean_wh_per_km"]) < traditional_energy
    ]
    if dominating:
        comparison = min(dominating, key=lambda row: float(row["mean_wh_per_km"]))
        comparison_rmse = float(comparison["mean_rmse_mps"])
        comparison_energy = float(comparison["mean_wh_per_km"])
        saving = (traditional_energy - comparison_energy) / traditional_energy * 100.0
        axis.annotate(
            f"Strictly dominates traditional\n{saving:.1f}% less energy and lower RMSE",
            xy=(comparison_rmse, comparison_energy),
            xytext=(comparison_rmse + 0.004, comparison_energy + 12.0),
            arrowprops={"arrowstyle": "->", "color": "tab:orange", "lw": 1.8},
            fontsize=10, fontweight="bold", color="darkorange",
        )
    axis.annotate(
        f"{traditional_energy:.1f} Wh/km\n{traditional_rmse:.4f} m/s",
        xy=(traditional_rmse, traditional_energy),
        xytext=(traditional_rmse + 0.003, traditional_energy + 9.0),
        arrowprops={"arrowstyle": "->", "color": "crimson", "lw": 1.4},
        fontsize=9, color="crimson",
    )
    axis.set(
        xlabel="Mean training speed RMSE [m/s] (lower is better)",
        ylabel="Mean training net battery energy [Wh/km] (lower is better)",
        title="Hardware Pareto frontier after independent MPC tuning",
    )
    axis.grid(alpha=0.25)
    axis.legend(loc="upper right")
    fig.colorbar(scatter, ax=axis, label="Motor scale $s_m$")
    fig.savefig(output, dpi=200)
    plt.close(fig)


def _plot_test_results(rows: list[dict[str, object]], output: Path) -> None:
    scenarios = list(dict.fromkeys(str(row["scenario"]) for row in rows))
    roles = ("traditional", "training_selected")
    colors = {"traditional": "tab:blue", "training_selected": "tab:orange"}
    x = np.arange(len(scenarios))
    width = 0.36
    fig, axes = plt.subplots(2, 1, figsize=(11.5, 8.5), sharex=True, constrained_layout=True)
    for offset, role in enumerate(roles):
        role_rows = {str(row["scenario"]): row for row in rows if row["hardware_role"] == role}
        energy = [float(role_rows[name]["wh_per_km"]) for name in scenarios]
        rmse = [float(role_rows[name]["rmse_mps"]) for name in scenarios]
        position = x + (offset - 0.5) * width
        axes[0].bar(position, energy, width, color=colors[role], label=role.replace("_", " "))
        axes[1].bar(position, rmse, width, color=colors[role], label=role.replace("_", " "))
    axes[0].set_ylabel("Net battery energy [Wh/km]")
    axes[1].set_ylabel("Speed RMSE [m/s]")
    axes[1].set_xticks(x, [name.replace("test_", "") for name in scenarios], rotation=15)
    for axis in axes:
        axis.grid(axis="y", alpha=0.25)
        axis.legend()
    fig.suptitle("Held-out scenarios with MPC re-tuned for each fixed hardware design")
    fig.savefig(output, dpi=180)
    plt.close(fig)


def run_generality_experiment(
    config: ProjectConfig, output_dir: Path, quick: bool = False
) -> dict[str, object]:
    output_dir.mkdir(parents=True, exist_ok=True)
    training = tuple(scenario for scenario in SCENARIO_DATASET if scenario.split == "train")
    testing = tuple(scenario for scenario in SCENARIO_DATASET if scenario.split == "test")
    if quick:
        ratios = (8.5, 10.0, 10.5, 11.5, 12.0)
        scales = (0.6, 0.75, 0.9)
        controllers = tuple(
            ControllerDesign(energy, slew)
            for energy in (-1.5, -1.0, -0.5, 0.0, 0.5)
            for slew in (-1.5, -1.0, -0.5)
        )
    else:
        ratios = tuple(np.arange(7.0, 12.01, 0.5))
        scales = tuple(np.arange(0.6, 1.051, 0.075))
        controllers = tuple(
            ControllerDesign(energy, slew)
            for energy in (-2.0, -1.5, -1.0, -0.5, 0.0, 0.5, 1.0)
            for slew in (-2.0, -1.5, -1.0, -0.5, 0.0)
        )
    sampled_hardware = tuple(
        HardwareDesign(ratio, scale) for ratio in ratios for scale in scales
    )
    hardware_candidates = tuple(
        hardware
        for hardware in sampled_hardware
        if evaluate_hardware(config, hardware).top_speed_feasible
    )
    store = EvaluationStore(output_dir / "evaluation_cache.json", config)
    all_evaluations: list[ScenarioEvaluation] = []
    selections: dict[tuple[float, float, str], ScenarioEvaluation] = {}
    total = len(training) * len(hardware_candidates) * len(controllers)
    completed = 0
    for scenario in training:
        for hardware in hardware_candidates:
            candidate_results = []
            for controller in controllers:
                result = store.get(scenario, hardware, controller)
                if result is None:
                    result = evaluate_scenario(config, scenario, hardware, controller)
                    store.put(scenario, hardware, controller, result)
                candidate_results.append(result)
                all_evaluations.append(result)
                completed += 1
            selected = select_controller(candidate_results)
            if selected is not None:
                selections[(hardware.final_drive_ratio, hardware.motor_scale, scenario.name)] = selected
        print(f"[{completed:3d}/{total}] completed training scenario {scenario.name}")

    selected_hardware, training_summaries = select_training_hardware(
        selections, hardware_candidates, training
    )
    _, traditional_sizing = size_hardware(config)
    traditional_hardware = HardwareDesign(
        traditional_sizing.final_drive_ratio, traditional_sizing.motor_scale
    )
    test_rows: list[dict[str, object]] = []
    test_selections: dict[str, dict[str, object]] = {}
    for scenario in testing:
        test_selections[scenario.name] = {}
        for role, hardware in (
            ("traditional", traditional_hardware),
            ("training_selected", selected_hardware),
        ):
            candidate_results = []
            for controller in controllers:
                result = store.get(scenario, hardware, controller)
                if result is None:
                    result = evaluate_scenario(config, scenario, hardware, controller)
                    store.put(scenario, hardware, controller, result)
                candidate_results.append(result)
                all_evaluations.append(result)
            selected = select_controller(candidate_results)
            if selected is None:
                selected = min(candidate_results, key=lambda result: result.rmse_mps)
            row = asdict(selected)
            row["hardware_role"] = role
            row["violations"] = ";".join(selected.violations)
            test_rows.append(row)
            test_selections[scenario.name][role] = row

    training_selection_rows = []
    for key, result in selections.items():
        row = asdict(result)
        row["violations"] = ";".join(result.violations)
        training_selection_rows.append(row)
    manifest_rows = [asdict(scenario) for scenario in SCENARIO_DATASET]
    evaluation_rows = []
    for result in all_evaluations:
        row = asdict(result)
        row["violations"] = ";".join(result.violations)
        evaluation_rows.append(row)
    _write_csv(manifest_rows, output_dir / "scenario_manifest.csv")
    _write_csv(evaluation_rows, output_dir / "evaluations.csv")
    _write_csv(training_selection_rows, output_dir / "training_controller_selections.csv")
    _write_csv(training_summaries, output_dir / "training_hardware_summary.csv")
    _write_csv(test_rows, output_dir / "test_results.csv")
    (output_dir / "scenario_manifest.json").write_text(
        json.dumps(manifest_rows, indent=2) + "\n", encoding="utf-8"
    )
    _plot_dataset(SCENARIO_DATASET, output_dir / "dataset_profiles.png")
    _plot_training_map(
        training_summaries, selected_hardware, output_dir / "training_hardware_map.png"
    )
    _plot_training_pareto(
        training_summaries, traditional_hardware,
        output_dir / "training_hardware_pareto.png",
    )
    _plot_test_results(test_rows, output_dir / "test_generalization.png")

    test_by_role = {
        role: [row for row in test_rows if row["hardware_role"] == role]
        for role in ("traditional", "training_selected")
    }
    mean_test_energy = {
        role: float(np.mean([float(row["wh_per_km"]) for row in rows]))
        for role, rows in test_by_role.items()
    }
    improvement = (
        (mean_test_energy["traditional"] - mean_test_energy["training_selected"])
        / mean_test_energy["traditional"]
        * 100.0
    )
    training_pareto = [
        row for row, flag in zip(
            training_summaries, training_hardware_pareto_flags(training_summaries)
        ) if flag
    ]
    report: dict[str, object] = {
        "mode": "quick" if quick else "full",
        "protocol": {
            "hardware_selected_using": "training scenarios only",
            "controller_policy": "re-optimized independently for every scenario after hardware is fixed",
            "objective": "minimum mean Wh/km subject to each scenario's tracking and mission constraints",
            "rmse_threshold_mps": 0.40,
            "controller_candidate_count": len(controllers),
            "controller_candidates": [asdict(controller) for controller in controllers],
        },
        "training_scenarios": [scenario.name for scenario in training],
        "test_scenarios": [scenario.name for scenario in testing],
        "traditional_hardware": asdict(traditional_hardware),
        "training_selected_hardware": asdict(selected_hardware),
        "training_hardware_pareto_frontier": training_pareto,
        "mean_test_wh_per_km": mean_test_energy,
        "test_energy_improvement_percent": improvement,
        "test_results": test_selections,
        "training_evaluation_count": total,
        "sampled_hardware_count": len(sampled_hardware),
        "top_speed_feasible_hardware_count": len(hardware_candidates),
        "cache_entry_count": len(store.values),
    }
    (output_dir / "generality_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    return report


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=Path("configs/default.yaml"))
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/generality_dataset"))
    parser.add_argument("--quick", action="store_true")
    args = parser.parse_args()
    report = run_generality_experiment(
        ProjectConfig.from_yaml(args.config), args.output_dir, quick=args.quick
    )
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
