"""Thirty-training/ten-test benchmark for hardware-controller generalization."""

from __future__ import annotations

import argparse
import csv
import json
import multiprocessing
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from .config import HardwareDesign, ProjectConfig  # noqa: E402
from .generality_dataset import (  # noqa: E402
    EvaluationStore,
    ScenarioDefinition,
    ScenarioEvaluation,
    evaluate_scenario,
    select_controller,
    select_training_hardware,
)
from .hardware_sizing import evaluate_hardware, size_hardware  # noqa: E402
from .optimization import ControllerDesign  # noqa: E402


TRAINING_CASE_COUNT = 30
TEST_CASE_COUNT = 10


def _latin_hypercube(count: int, dimensions: int, seed: int) -> np.ndarray:
    """Deterministic Latin-hypercube points in the open unit cube."""
    rng = np.random.default_rng(seed)
    result = np.empty((count, dimensions), dtype=float)
    for column in range(dimensions):
        result[:, column] = (rng.permutation(count) + rng.random(count)) / count
    return result


def _scale(values: np.ndarray, lower: float, upper: float) -> np.ndarray:
    return lower + values * (upper - lower)


def _build_cases(
    *,
    count: int,
    split: str,
    family: str,
    lhs_seed: int,
    simulator_seed_start: int,
    bounds: dict[str, tuple[float, float]],
) -> tuple[ScenarioDefinition, ...]:
    fields = (
        "speed", "dwell", "launch", "cruise", "braking", "uphill",
        "downhill_magnitude", "payload", "drag", "temperature", "discharge",
        "charge", "cycles",
    )
    samples = _latin_hypercube(count, len(fields), lhs_seed)
    scaled = {
        name: _scale(samples[:, index], *bounds[name])
        for index, name in enumerate(fields)
    }
    cases = []
    for index in range(count):
        cycles = 2 if scaled["cycles"][index] < 2.55 else 3
        cases.append(
            ScenarioDefinition(
                name=f"{split}_{family}_{index + 1:02d}",
                split=split,
                seed=simulator_seed_start + index,
                cycles=cycles,
                cruise_speed_mps=round(float(scaled["speed"][index]), 2),
                dwell_s=round(float(scaled["dwell"][index]), 2),
                launch_s=round(float(scaled["launch"][index]), 2),
                cruise_s=round(float(scaled["cruise"][index]), 2),
                braking_s=round(float(scaled["braking"][index]), 2),
                uphill_grade=round(float(scaled["uphill"][index]), 4),
                downhill_grade=-round(float(scaled["downhill_magnitude"][index]), 4),
                payload_kg=round(float(scaled["payload"][index]), 1),
                drag_multiplier=round(float(scaled["drag"][index]), 3),
                initial_motor_temperature_c=round(float(scaled["temperature"][index]), 1),
                discharge_power_kw=round(float(scaled["discharge"][index]), 1),
                charge_power_kw=round(float(scaled["charge"][index]), 1),
            )
        )
    return tuple(cases)


TRAIN_BOUNDS = {
    "speed": (12.5, 18.0), "dwell": (2.0, 4.0), "launch": (6.5, 8.5),
    "cruise": (5.0, 8.0), "braking": (5.5, 7.5), "uphill": (0.07, 0.12),
    "downhill_magnitude": (0.07, 0.13), "payload": (0.0, 180.0),
    "drag": (0.98, 1.10), "temperature": (50.0, 68.0),
    "discharge": (85.0, 100.0), "charge": (32.0, 52.0), "cycles": (2.0, 3.0),
}

STRESS_BOUNDS = {
    "speed": (16.0, 19.0), "dwell": (2.0, 3.5), "launch": (7.0, 9.0),
    "cruise": (6.0, 8.5), "braking": (5.5, 7.0), "uphill": (0.12, 0.15),
    "downhill_magnitude": (0.12, 0.15), "payload": (180.0, 260.0),
    "drag": (1.08, 1.15), "temperature": (65.0, 75.0),
    "discharge": (75.0, 90.0), "charge": (25.0, 40.0), "cycles": (2.4, 3.0),
}


EXPANDED_SCENARIO_DATASET = (
    *_build_cases(
        count=30, split="train", family="coverage", lhs_seed=20260621,
        simulator_seed_start=1001, bounds=TRAIN_BOUNDS,
    ),
    *_build_cases(
        count=5, split="test", family="interpolation", lhs_seed=20260622,
        simulator_seed_start=2001, bounds=TRAIN_BOUNDS,
    ),
    *_build_cases(
        count=5, split="test", family="stress", lhs_seed=20260623,
        simulator_seed_start=2101, bounds=STRESS_BOUNDS,
    ),
)


def scenario_family(scenario: ScenarioDefinition) -> str:
    if scenario.split == "train":
        return "training_coverage"
    return "stress" if "_stress_" in scenario.name else "interpolation"


def screening_controller_grid() -> tuple[ControllerDesign, ...]:
    return tuple(
        ControllerDesign(energy, slew)
        for energy in (-1.5, -1.0, -0.5, 0.0, 0.5)
        for slew in (-1.5, -1.0, -0.5)
    )


def test_controller_grid() -> tuple[ControllerDesign, ...]:
    return tuple(
        ControllerDesign(energy, slew)
        for energy in (-2.0, -1.5, -1.0, -0.5, 0.0, 0.25, 0.5, 0.75)
        for slew in (-2.0, -1.5, -1.0, -0.5, 0.0)
    )


def quick_hardware_grid(config: ProjectConfig) -> tuple[HardwareDesign, ...]:
    sampled = tuple(
        HardwareDesign(ratio, scale)
        for ratio in (8.5, 10.0, 10.5, 11.5, 12.0)
        for scale in (0.60, 0.75, 0.90)
    )
    return tuple(
        hardware for hardware in sampled
        if evaluate_hardware(config, hardware).top_speed_feasible
    )


def _write_csv(rows: list[dict[str, object]], path: Path) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def _manifest_rows() -> list[dict[str, object]]:
    rows = []
    for scenario in EXPANDED_SCENARIO_DATASET:
        row = asdict(scenario)
        row["family"] = scenario_family(scenario)
        rows.append(row)
    return rows


def _evaluate_candidates(
    config: ProjectConfig,
    store: EvaluationStore,
    scenario: ScenarioDefinition,
    hardware: HardwareDesign,
    controllers: tuple[ControllerDesign, ...],
    *,
    flush_interval: int = 20,
    executor: ProcessPoolExecutor | None = None,
) -> list[ScenarioEvaluation]:
    results_by_controller: dict[ControllerDesign, ScenarioEvaluation] = {}
    missing = []
    for controller in controllers:
        result = store.get(scenario, hardware, controller)
        if result is not None:
            results_by_controller[controller] = result
        else:
            missing.append(controller)
    if executor is None:
        computed = [
            evaluate_scenario(config, scenario, hardware, controller)
            for controller in missing
        ]
    else:
        computed = list(
            executor.map(
                _evaluate_worker,
                [(config, scenario, hardware, controller) for controller in missing],
            )
        )
    for index, (controller, result) in enumerate(zip(missing, computed), start=1):
        results_by_controller[controller] = result
        store.put(scenario, hardware, controller, result, persist=False)
        if index % flush_interval == 0:
            store.flush()
    if missing:
        store.flush()
    return [results_by_controller[controller] for controller in controllers]


def _evaluate_worker(
    payload: tuple[ProjectConfig, ScenarioDefinition, HardwareDesign, ControllerDesign],
) -> ScenarioEvaluation:
    return evaluate_scenario(*payload)


def _minimum_energy_under_rmse(
    results: list[ScenarioEvaluation], bound: float
) -> ScenarioEvaluation | None:
    feasible = [
        result for result in results
        if not [item for item in result.violations if item != "tracking_rmse"]
        and result.rmse_mps <= bound + 1e-12
    ]
    return min(feasible, key=lambda result: result.wh_per_km) if feasible else None


def _nontracking_violations(result: ScenarioEvaluation) -> tuple[str, ...]:
    return tuple(item for item in result.violations if item != "tracking_rmse")


def _best_operational_tracking(results: list[ScenarioEvaluation]) -> ScenarioEvaluation:
    operational = [result for result in results if not _nontracking_violations(result)]
    if operational:
        return min(operational, key=lambda result: (result.rmse_mps, result.wh_per_km))
    return min(
        results,
        key=lambda result: (
            len(_nontracking_violations(result)), result.rmse_mps, result.wh_per_km,
        ),
    )


def _plot_distributions(output: Path) -> None:
    train = [item for item in EXPANDED_SCENARIO_DATASET if item.split == "train"]
    interpolation = [
        item for item in EXPANDED_SCENARIO_DATASET
        if scenario_family(item) == "interpolation"
    ]
    stress = [item for item in EXPANDED_SCENARIO_DATASET if scenario_family(item) == "stress"]
    fig, axes = plt.subplots(2, 2, figsize=(11.5, 8.5), constrained_layout=True)
    groups = (
        (train, "tab:blue", "Train (30)"),
        (interpolation, "tab:green", "Unseen interpolation (5)"),
        (stress, "tab:red", "Unseen stress (5)"),
    )
    fields = (
        ("cruise_speed_mps", "Cruise speed [m/s]"),
        ("uphill_grade", "Uphill grade"),
        ("payload_kg", "Payload [kg]"),
        ("charge_power_kw", "Charge-power limit [kW]"),
    )
    for axis, (field, label) in zip(axes.flat, fields):
        for group, color, name in groups:
            axis.hist(
                [getattr(item, field) for item in group], bins=8, alpha=0.45,
                color=color, label=name,
            )
        axis.set_xlabel(label)
        axis.set_ylabel("Case count")
        axis.grid(alpha=0.2)
    axes[0, 0].legend(fontsize=8)
    fig.suptitle("Expanded 30-training/10-test scenario distribution")
    fig.savefig(output, dpi=190)
    plt.close(fig)


def _plot_test_results(rows: list[dict[str, object]], output: Path) -> None:
    scenarios = list(dict.fromkeys(str(row["scenario"]) for row in rows))
    roles = ("traditional", "training_selected")
    colors = {"traditional": "tab:blue", "training_selected": "tab:orange"}
    x = np.arange(len(scenarios))
    width = 0.38
    fig, axes = plt.subplots(2, 1, figsize=(13, 9), sharex=True, constrained_layout=True)
    for offset, role in enumerate(roles):
        by_name = {str(row["scenario"]): row for row in rows if row["hardware_role"] == role}
        position = x + (offset - 0.5) * width
        axes[0].bar(
            position, [float(by_name[name]["wh_per_km"]) for name in scenarios],
            width, color=colors[role], label=role.replace("_", " "),
        )
        axes[1].bar(
            position, [float(by_name[name]["rmse_mps"]) for name in scenarios],
            width, color=colors[role], label=role.replace("_", " "),
        )
    axes[1].axhline(0.4, color="tab:red", linestyle="--", label="RMSE limit")
    axes[0].set_ylabel("Net battery energy [Wh/km]")
    axes[1].set_ylabel("Speed RMSE [m/s]")
    axes[1].set_xticks(x, [name.replace("test_", "") for name in scenarios], rotation=28)
    for axis in axes:
        axis.grid(axis="y", alpha=0.25)
        axis.legend()
    for index, name in enumerate(scenarios):
        scenario_rows = [row for row in rows if row["scenario"] == name]
        if all(bool(row["violations"]) for row in scenario_rows):
            axes[1].text(
                index, 0.73, "both\ninfeasible", ha="center", va="top",
                color="crimson", fontsize=8, fontweight="bold",
            )
    axes[0].axvspan(4.5, 9.5, color="tab:red", alpha=0.05)
    axes[1].axvspan(4.5, 9.5, color="tab:red", alpha=0.05)
    fig.suptitle("Ten unseen cases with controller re-tuning after hardware freeze")
    fig.savefig(output, dpi=190)
    plt.close(fig)


def _plot_training_hardware(
    summaries: list[dict[str, object]],
    selected: HardwareDesign,
    traditional: HardwareDesign,
    output: Path,
) -> None:
    ratios = sorted({float(row["final_drive_ratio"]) for row in summaries})
    scales = sorted({float(row["motor_scale"]) for row in summaries})
    values = np.full((len(scales), len(ratios)), np.nan)
    for row in summaries:
        if bool(row["feasible"]):
            values[
                scales.index(float(row["motor_scale"])),
                ratios.index(float(row["final_drive_ratio"])),
            ] = float(row["mean_wh_per_km"])
    fig, axis = plt.subplots(figsize=(9.5, 5.8), constrained_layout=True)
    image = axis.imshow(values, origin="lower", aspect="auto", cmap="viridis")
    axis.set_xticks(range(len(ratios)), [f"{value:g}" for value in ratios])
    axis.set_yticks(range(len(scales)), [f"{value:g}" for value in scales])
    for scale_index, scale in enumerate(scales):
        for ratio_index, ratio in enumerate(ratios):
            value = values[scale_index, ratio_index]
            label = "infeasible" if np.isnan(value) else f"{value:.1f}"
            axis.text(ratio_index, scale_index, label, ha="center", va="center", fontsize=8)
    axis.scatter(
        ratios.index(selected.final_drive_ratio), scales.index(selected.motor_scale),
        marker="*", s=300, color="tab:orange", edgecolor="white",
        label="30-case selected hardware",
    )
    axis.scatter(
        ratios.index(traditional.final_drive_ratio), scales.index(traditional.motor_scale),
        marker="X", s=180, color="crimson", edgecolor="white",
        label="Traditional hardware",
    )
    axis.set(
        xlabel="Final-drive ratio", ylabel="Motor scale",
        title="Mean training energy after controller tuning [Wh/km]",
    )
    axis.legend()
    fig.colorbar(image, ax=axis, label="Mean Wh/km")
    fig.savefig(output, dpi=190)
    plt.close(fig)


def run_expanded_experiment(
    config: ProjectConfig,
    output_dir: Path,
    *,
    training_limit: int | None = None,
    test_limit: int | None = None,
    workers: int = 1,
) -> dict[str, object]:
    output_dir.mkdir(parents=True, exist_ok=True)
    training = tuple(item for item in EXPANDED_SCENARIO_DATASET if item.split == "train")
    testing = tuple(item for item in EXPANDED_SCENARIO_DATASET if item.split == "test")
    if training_limit is not None:
        training = training[:training_limit]
    if test_limit is not None:
        testing = testing[:test_limit]
    hardware_candidates = quick_hardware_grid(config)
    screening_controllers = screening_controller_grid()
    dense_controllers = test_controller_grid()
    store = EvaluationStore(output_dir / "evaluation_cache.json", config)
    selections: dict[tuple[float, float, str], ScenarioEvaluation] = {}
    selection_rows: list[dict[str, object]] = []
    executor_context = (
        ProcessPoolExecutor(
            max_workers=workers, mp_context=multiprocessing.get_context("spawn")
        )
        if workers > 1 else None
    )
    try:
        total_groups = len(training) * len(hardware_candidates)
        group = 0
        for scenario in training:
            for hardware in hardware_candidates:
                group += 1
                results = _evaluate_candidates(
                    config, store, scenario, hardware, screening_controllers,
                    executor=executor_context,
                )
                selected = select_controller(results)
                if selected is not None:
                    selections[(hardware.final_drive_ratio, hardware.motor_scale, scenario.name)] = selected
                    row = asdict(selected)
                    row["violations"] = ";".join(selected.violations)
                    selection_rows.append(row)
                print(
                    f"[train {group:03d}/{total_groups}] {scenario.name} "
                    f"h=({hardware.final_drive_ratio:g},{hardware.motor_scale:g}) "
                    f"{'feasible' if selected else 'infeasible'}", flush=True,
                )
        selected_hardware, hardware_summaries = select_training_hardware(
            selections, hardware_candidates, training
        )
        _, traditional_sizing = size_hardware(config)
        traditional_hardware = HardwareDesign(
            traditional_sizing.final_drive_ratio, traditional_sizing.motor_scale
        )
        test_rows: list[dict[str, object]] = []
        comparison_rows: list[dict[str, object]] = []
        for scenario_index, scenario in enumerate(testing, start=1):
            role_results: dict[str, list[ScenarioEvaluation]] = {}
            for role, hardware in (
                ("traditional", traditional_hardware),
                ("training_selected", selected_hardware),
            ):
                role_results[role] = _evaluate_candidates(
                    config, store, scenario, hardware, dense_controllers,
                    executor=executor_context,
                )
            traditional = _minimum_energy_under_rmse(role_results["traditional"], 0.40)
            if traditional is None:
                traditional = _best_operational_tracking(role_results["traditional"])
            selected = _minimum_energy_under_rmse(
                role_results["training_selected"], min(0.40, traditional.rmse_mps)
            )
            if selected is None:
                selected = _best_operational_tracking(role_results["training_selected"])
            for role, result in (("traditional", traditional), ("training_selected", selected)):
                row = asdict(result)
                row["hardware_role"] = role
                row["family"] = scenario_family(scenario)
                row["violations"] = ";".join(result.violations)
                test_rows.append(row)
            comparison_rows.append(
                {
                    "scenario": scenario.name,
                    "family": scenario_family(scenario),
                    "traditional_rmse_mps": traditional.rmse_mps,
                    "selected_rmse_mps": selected.rmse_mps,
                    "traditional_wh_per_km": traditional.wh_per_km,
                    "selected_wh_per_km": selected.wh_per_km,
                    "traditional_feasible": not traditional.violations,
                    "selected_feasible": not selected.violations,
                    "selected_matches_rmse": (
                        not _nontracking_violations(traditional)
                        and not _nontracking_violations(selected)
                        and selected.rmse_mps <= traditional.rmse_mps + 1e-12
                    ),
                    "selected_saves_energy": (
                        not traditional.violations
                        and not selected.violations
                        and selected.wh_per_km < traditional.wh_per_km
                    ),
                }
            )
            print(
                f"[test {scenario_index:02d}/{len(testing)}] {scenario.name}: "
                f"RMSE {traditional.rmse_mps:.4f}->{selected.rmse_mps:.4f}, "
                f"energy {traditional.wh_per_km:.2f}->{selected.wh_per_km:.2f}", flush=True,
            )
    finally:
        if executor_context is not None:
            executor_context.shutdown(wait=True, cancel_futures=True)
    manifest_rows = _manifest_rows()
    _write_csv(manifest_rows, output_dir / "scenario_manifest.csv")
    _write_csv(selection_rows, output_dir / "training_controller_selections.csv")
    _write_csv(hardware_summaries, output_dir / "training_hardware_summary.csv")
    _write_csv(test_rows, output_dir / "test_selected_controllers.csv")
    _write_csv(comparison_rows, output_dir / "test_comparison.csv")
    (output_dir / "scenario_manifest.json").write_text(
        json.dumps(manifest_rows, indent=2) + "\n", encoding="utf-8"
    )
    _plot_distributions(output_dir / "scenario_distributions.png")
    _plot_test_results(test_rows, output_dir / "test_results.png")
    _plot_training_hardware(
        hardware_summaries, selected_hardware, traditional_hardware,
        output_dir / "training_hardware_map.png",
    )
    valid_common = [
        row for row in comparison_rows
        if bool(row["traditional_feasible"]) and bool(row["selected_feasible"])
    ]
    traditional_mean = float(
        np.mean([float(row["traditional_wh_per_km"]) for row in valid_common])
    )
    selected_mean = float(
        np.mean([float(row["selected_wh_per_km"]) for row in valid_common])
    )
    family_summaries = {}
    for family in ("interpolation", "stress"):
        family_rows = [row for row in valid_common if row["family"] == family]
        traditional_family = float(
            np.mean([float(row["traditional_wh_per_km"]) for row in family_rows])
        )
        selected_family = float(
            np.mean([float(row["selected_wh_per_km"]) for row in family_rows])
        )
        family_summaries[family] = {
            "valid_case_count": len(family_rows),
            "traditional_mean_wh_per_km": traditional_family,
            "selected_mean_wh_per_km": selected_family,
            "energy_improvement_percent": (
                (traditional_family - selected_family) / traditional_family * 100.0
            ),
            "matched_rmse_case_count": sum(
                bool(row["selected_matches_rmse"]) for row in family_rows
            ),
        }
    summary_by_hardware = {
        (float(row["final_drive_ratio"]), float(row["motor_scale"])): row
        for row in hardware_summaries
    }
    traditional_training = float(
        summary_by_hardware[
            (traditional_hardware.final_drive_ratio, traditional_hardware.motor_scale)
        ]["mean_wh_per_km"]
    )
    selected_training = float(
        summary_by_hardware[
            (selected_hardware.final_drive_ratio, selected_hardware.motor_scale)
        ]["mean_wh_per_km"]
    )
    report: dict[str, object] = {
        "protocol": {
            "training_case_count": len(training),
            "test_case_count": len(testing),
            "test_interpolation_count": sum(
                scenario_family(item) == "interpolation" for item in testing
            ),
            "test_stress_count": sum(scenario_family(item) == "stress" for item in testing),
            "hardware_candidate_count": len(hardware_candidates),
            "training_controller_candidate_count": len(screening_controllers),
            "test_controller_candidate_count": len(dense_controllers),
            "hardware_selected_using": "training cases only",
            "test_controller_rule": "selected hardware must not exceed traditional achieved RMSE",
        },
        "traditional_hardware": asdict(traditional_hardware),
        "training_selected_hardware": asdict(selected_hardware),
        "mean_training_wh_per_km": {
            "traditional": traditional_training,
            "training_selected": selected_training,
        },
        "training_energy_improvement_percent": (
            (traditional_training - selected_training) / traditional_training * 100.0
        ),
        "mean_test_wh_per_km": {
            "traditional": traditional_mean, "training_selected": selected_mean,
        },
        "mean_energy_improvement_percent": (
            (traditional_mean - selected_mean) / traditional_mean * 100.0
        ),
        "common_threshold_valid_case_count": len(valid_common),
        "common_threshold_energy_win_count": sum(
            bool(row["selected_saves_energy"]) for row in valid_common
        ),
        "matched_rmse_valid_case_count": sum(
            bool(row["selected_matches_rmse"]) for row in valid_common
        ),
        "infeasible_case_names": [
            str(row["scenario"]) for row in comparison_rows if row not in valid_common
        ],
        "family_summaries": family_summaries,
        "all_test_cases_match_or_improve_rmse": all(
            bool(row["selected_matches_rmse"]) for row in comparison_rows
        ),
        "all_test_cases_save_energy": all(
            bool(row["selected_saves_energy"]) for row in comparison_rows
        ),
        "test_comparisons": comparison_rows,
        "cache_entry_count": len(store.values),
    }
    (output_dir / "expanded_generality_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    return report


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=Path("configs/default.yaml"))
    parser.add_argument(
        "--output-dir", type=Path, default=Path("artifacts/expanded_generality")
    )
    parser.add_argument("--pilot", action="store_true")
    parser.add_argument("--workers", type=int, default=1)
    args = parser.parse_args()
    report = run_expanded_experiment(
        ProjectConfig.from_yaml(args.config), args.output_dir,
        training_limit=3 if args.pilot else None,
        test_limit=2 if args.pilot else None,
        workers=args.workers,
    )
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
