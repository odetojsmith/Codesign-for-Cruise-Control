"""Held-out energy comparison requiring co-design RMSE to beat the traditional result."""

from __future__ import annotations

import argparse
import csv
import json
from dataclasses import asdict, dataclass
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from .config import HardwareDesign, ProjectConfig  # noqa: E402
from .generality_dataset import (  # noqa: E402
    EvaluationStore,
    SCENARIO_DATASET,
    ScenarioEvaluation,
    evaluate_scenario,
)
from .optimization import ControllerDesign  # noqa: E402


TRADITIONAL_HARDWARE = HardwareDesign(10.5, 0.60)
TRAINING_SELECTED_HARDWARE = HardwareDesign(11.5, 0.75)
GLOBAL_RMSE_LIMIT_MPS = 0.40


def dense_controller_grid() -> tuple[ControllerDesign, ...]:
    return tuple(
        ControllerDesign(energy, slew)
        for energy in (-2.0, -1.5, -1.0, -0.5, 0.0, 0.25, 0.5, 0.75)
        for slew in (-2.0, -1.5, -1.0, -0.5, 0.0)
    )


def select_minimum_energy_under_rmse(
    results: list[ScenarioEvaluation], rmse_bound_mps: float
) -> ScenarioEvaluation | None:
    feasible = [
        result
        for result in results
        if result.feasible and result.rmse_mps <= rmse_bound_mps + 1e-12
    ]
    return min(feasible, key=lambda result: result.wh_per_km) if feasible else None


@dataclass(frozen=True, slots=True)
class MatchedComparison:
    scenario: str
    traditional_rmse_mps: float
    selected_rmse_mps: float
    rmse_margin_mps: float
    traditional_wh_per_km: float
    selected_wh_per_km: float
    energy_improvement_percent: float
    traditional_log10_lambda_energy: float
    traditional_log10_lambda_force_slew: float
    selected_log10_lambda_energy: float
    selected_log10_lambda_force_slew: float
    traditional_feasible: bool
    selected_feasible: bool
    matched_rmse_pass: bool


def _evaluate_grid(
    config: ProjectConfig,
    store: EvaluationStore,
    scenario_name: str,
    hardware: HardwareDesign,
    controllers: tuple[ControllerDesign, ...],
) -> list[ScenarioEvaluation]:
    scenario = next(item for item in SCENARIO_DATASET if item.name == scenario_name)
    results = []
    for controller in controllers:
        result = store.get(scenario, hardware, controller)
        if result is None:
            result = evaluate_scenario(config, scenario, hardware, controller)
            store.put(scenario, hardware, controller, result)
        results.append(result)
    return results


def _write_csv(rows: list[dict[str, object]], path: Path) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def _plot(comparisons: list[MatchedComparison], output: Path) -> None:
    labels = [item.scenario.replace("test_", "") for item in comparisons]
    x = np.arange(len(labels))
    width = 0.36
    fig, axes = plt.subplots(2, 1, figsize=(11.5, 8.5), sharex=True, constrained_layout=True)
    axes[0].bar(
        x - width / 2,
        [item.traditional_wh_per_km for item in comparisons],
        width,
        label="Traditional hardware",
        color="tab:blue",
    )
    axes[0].bar(
        x + width / 2,
        [item.selected_wh_per_km for item in comparisons],
        width,
        label="Training-selected hardware",
        color="tab:orange",
    )
    axes[1].bar(
        x - width / 2,
        [item.traditional_rmse_mps for item in comparisons],
        width,
        label="Traditional hardware",
        color="tab:blue",
    )
    axes[1].bar(
        x + width / 2,
        [item.selected_rmse_mps for item in comparisons],
        width,
        label="Training-selected hardware",
        color="tab:orange",
    )
    axes[1].axhline(GLOBAL_RMSE_LIMIT_MPS, color="tab:red", linestyle="--", label="Global RMSE limit")
    axes[0].set_ylabel("Net battery energy [Wh/km]")
    axes[1].set_ylabel("Speed RMSE [m/s]")
    axes[1].set_xticks(x, labels, rotation=15)
    for axis in axes:
        axis.grid(axis="y", alpha=0.25)
        axis.legend()
    fig.suptitle("Held-out comparison with co-design RMSE constrained below traditional RMSE")
    fig.savefig(output, dpi=180)
    plt.close(fig)


def run_matched_rmse_test(config: ProjectConfig, output_dir: Path) -> dict[str, object]:
    output_dir.mkdir(parents=True, exist_ok=True)
    controllers = dense_controller_grid()
    store = EvaluationStore(output_dir / "evaluation_cache.json", config)
    comparisons: list[MatchedComparison] = []
    selected_rows: list[dict[str, object]] = []
    test_scenarios = tuple(item for item in SCENARIO_DATASET if item.split == "test")
    for scenario in test_scenarios:
        traditional_results = _evaluate_grid(
            config, store, scenario.name, TRADITIONAL_HARDWARE, controllers
        )
        traditional = select_minimum_energy_under_rmse(
            traditional_results, GLOBAL_RMSE_LIMIT_MPS
        )
        if traditional is None:
            raise RuntimeError(f"traditional hardware has no feasible controller for {scenario.name}")
        selected_results = _evaluate_grid(
            config, store, scenario.name, TRAINING_SELECTED_HARDWARE, controllers
        )
        selected = select_minimum_energy_under_rmse(
            selected_results, traditional.rmse_mps
        )
        if selected is None:
            raise RuntimeError(
                f"training-selected hardware cannot match traditional RMSE on {scenario.name}"
            )
        improvement = (
            (traditional.wh_per_km - selected.wh_per_km)
            / traditional.wh_per_km
            * 100.0
        )
        comparison = MatchedComparison(
            scenario=scenario.name,
            traditional_rmse_mps=traditional.rmse_mps,
            selected_rmse_mps=selected.rmse_mps,
            rmse_margin_mps=traditional.rmse_mps - selected.rmse_mps,
            traditional_wh_per_km=traditional.wh_per_km,
            selected_wh_per_km=selected.wh_per_km,
            energy_improvement_percent=improvement,
            traditional_log10_lambda_energy=traditional.log10_lambda_energy,
            traditional_log10_lambda_force_slew=traditional.log10_lambda_force_slew,
            selected_log10_lambda_energy=selected.log10_lambda_energy,
            selected_log10_lambda_force_slew=selected.log10_lambda_force_slew,
            traditional_feasible=traditional.feasible,
            selected_feasible=selected.feasible,
            matched_rmse_pass=selected.rmse_mps <= traditional.rmse_mps + 1e-12,
        )
        comparisons.append(comparison)
        for role, result in (("traditional", traditional), ("training_selected", selected)):
            row = asdict(result)
            row["hardware_role"] = role
            row["comparison_rmse_bound_mps"] = (
                GLOBAL_RMSE_LIMIT_MPS if role == "traditional" else traditional.rmse_mps
            )
            row["violations"] = ";".join(result.violations)
            selected_rows.append(row)
        print(
            f"{scenario.name}: RMSE {traditional.rmse_mps:.4f}->{selected.rmse_mps:.4f}, "
            f"energy {traditional.wh_per_km:.2f}->{selected.wh_per_km:.2f} Wh/km"
        )

    comparison_rows = [asdict(item) for item in comparisons]
    _write_csv(comparison_rows, output_dir / "matched_rmse_comparison.csv")
    _write_csv(selected_rows, output_dir / "matched_rmse_selected_controllers.csv")
    _plot(comparisons, output_dir / "matched_rmse_test.png")
    traditional_mean = float(np.mean([item.traditional_wh_per_km for item in comparisons]))
    selected_mean = float(np.mean([item.selected_wh_per_km for item in comparisons]))
    report: dict[str, object] = {
        "protocol": {
            "traditional_controller": "minimum energy subject to RMSE <= 0.4 m/s",
            "selected_hardware_controller": "minimum energy subject to RMSE <= achieved traditional RMSE for the same scenario",
            "controller_candidate_count": len(controllers),
            "controller_candidates": [asdict(item) for item in controllers],
        },
        "traditional_hardware": asdict(TRADITIONAL_HARDWARE),
        "training_selected_hardware": asdict(TRAINING_SELECTED_HARDWARE),
        "comparisons": comparison_rows,
        "all_scenarios_matched_or_better_rmse": all(
            item.matched_rmse_pass for item in comparisons
        ),
        "all_scenarios_lower_energy": all(
            item.selected_wh_per_km < item.traditional_wh_per_km for item in comparisons
        ),
        "mean_traditional_wh_per_km": traditional_mean,
        "mean_selected_wh_per_km": selected_mean,
        "mean_energy_improvement_percent": (
            (traditional_mean - selected_mean) / traditional_mean * 100.0
        ),
        "cache_entry_count": len(store.values),
    }
    (output_dir / "matched_rmse_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    return report


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=Path("configs/default.yaml"))
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/matched_rmse_test"))
    args = parser.parse_args()
    report = run_matched_rmse_test(ProjectConfig.from_yaml(args.config), args.output_dir)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
