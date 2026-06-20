"""Dense MPC sweep for the training-selected hardware across training scenarios."""

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


TRAINED_HARDWARE = HardwareDesign(11.5, 0.75)
TRADITIONAL_HARDWARE = HardwareDesign(10.5, 0.60)
TRADITIONAL_MEAN_RMSE_MPS = 0.3537804368101449
TRADITIONAL_MEAN_WH_PER_KM = 311.1507182133985


@dataclass(frozen=True, slots=True)
class ControllerSweepPoint:
    log10_lambda_energy: float
    log10_lambda_force_slew: float
    mean_rmse_mps: float
    mean_wh_per_km: float
    maximum_rmse_mps: float
    feasible_scenario_count: int
    scenario_count: int
    operationally_feasible: bool
    pareto_optimal: bool = False


def dense_controller_grid() -> tuple[ControllerDesign, ...]:
    return tuple(
        ControllerDesign(energy, slew)
        for energy in (-2.0, -1.5, -1.0, -0.5, 0.0, 0.25, 0.5, 0.75)
        for slew in (-2.0, -1.5, -1.0, -0.5, 0.0)
    )


def _operationally_feasible(result: ScenarioEvaluation) -> bool:
    """Allow tracking-RMSE variation because RMSE is an objective on this plot."""
    return all(violation == "tracking_rmse" for violation in result.violations)


def pareto_flags(points: list[ControllerSweepPoint]) -> list[bool]:
    flags: list[bool] = []
    for candidate in points:
        dominated = any(
            other.operationally_feasible
            and other is not candidate
            and other.mean_rmse_mps <= candidate.mean_rmse_mps
            and other.mean_wh_per_km <= candidate.mean_wh_per_km
            and (
                other.mean_rmse_mps < candidate.mean_rmse_mps
                or other.mean_wh_per_km < candidate.mean_wh_per_km
            )
            for other in points
        )
        flags.append(candidate.operationally_feasible and not dominated)
    return flags


def _write_csv(points: list[ControllerSweepPoint], output: Path) -> None:
    with output.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(asdict(points[0]).keys()))
        writer.writeheader()
        writer.writerows(asdict(point) for point in points)


def _plot(points: list[ControllerSweepPoint], output: Path) -> None:
    feasible = [point for point in points if point.operationally_feasible]
    rejected = [point for point in points if not point.operationally_feasible]
    frontier = [point for point in points if point.pareto_optimal]
    fig, axis = plt.subplots(figsize=(10.5, 6.8), constrained_layout=True)
    scatter = axis.scatter(
        [point.mean_rmse_mps for point in feasible],
        [point.mean_wh_per_km for point in feasible],
        c=[point.log10_lambda_energy for point in feasible],
        cmap="viridis",
        s=75,
        alpha=0.68,
        edgecolor="white",
        linewidth=0.8,
        label="Dense MPC samples",
        zorder=2,
    )
    if rejected:
        axis.scatter(
            [point.mean_rmse_mps for point in rejected],
            [point.mean_wh_per_km for point in rejected],
            marker="x",
            color="0.65",
            s=55,
            label="Mission-constraint violation",
            zorder=1,
        )
    axis.scatter(
        [point.mean_rmse_mps for point in frontier],
        [point.mean_wh_per_km for point in frontier],
        marker="D",
        color="tab:orange",
        edgecolor="black",
        linewidth=0.8,
        s=105,
        label="Nondominated controller samples",
        zorder=4,
    )
    axis.scatter(
        [TRADITIONAL_MEAN_RMSE_MPS],
        [TRADITIONAL_MEAN_WH_PER_KM],
        marker="*",
        s=430,
        color="crimson",
        edgecolor="white",
        linewidth=1.5,
        label="Traditional hardware with tuned MPC",
        zorder=5,
    )
    tracking_feasible_frontier = [
        point for point in frontier if point.maximum_rmse_mps <= 0.40
    ]
    if tracking_feasible_frontier:
        best_feasible = min(
            tracking_feasible_frontier, key=lambda point: point.mean_wh_per_km
        )
        saving = (
            (TRADITIONAL_MEAN_WH_PER_KM - best_feasible.mean_wh_per_km)
            / TRADITIONAL_MEAN_WH_PER_KM
            * 100.0
        )
        axis.annotate(
            f"Best frontier point with RMSE ≤0.4 in every scenario\n"
            f"{saving:.1f}% less energy than traditional",
            xy=(best_feasible.mean_rmse_mps, best_feasible.mean_wh_per_km),
            xytext=(best_feasible.mean_rmse_mps + 0.018, best_feasible.mean_wh_per_km + 14.0),
            arrowprops={"arrowstyle": "->", "color": "tab:orange", "lw": 1.5},
            color="darkorange",
            fontsize=9,
            fontweight="bold",
        )
    axis.annotate(
        f"Traditional hardware\n{TRADITIONAL_MEAN_RMSE_MPS:.4f} m/s, "
        f"{TRADITIONAL_MEAN_WH_PER_KM:.1f} Wh/km",
        xy=(TRADITIONAL_MEAN_RMSE_MPS, TRADITIONAL_MEAN_WH_PER_KM),
        xytext=(TRADITIONAL_MEAN_RMSE_MPS + 0.018, TRADITIONAL_MEAN_WH_PER_KM + 5.0),
        arrowprops={"arrowstyle": "->", "color": "crimson", "lw": 1.4},
        color="crimson",
        fontsize=9,
    )
    axis.set(
        xlabel="Mean training speed RMSE [m/s] (lower is better)",
        ylabel="Mean training net battery energy [Wh/km] (lower is better)",
        title="Dense controller sweep for trained hardware ($g=11.5$, $s_m=0.75$)",
    )
    axis.grid(alpha=0.25)
    axis.legend(loc="best")
    fig.colorbar(scatter, ax=axis, label=r"$\log_{10}\lambda_E$")
    fig.savefig(output, dpi=200)
    plt.close(fig)


def run_dense_sweep(
    config: ProjectConfig,
    output_dir: Path,
    cache_path: Path,
) -> dict[str, object]:
    output_dir.mkdir(parents=True, exist_ok=True)
    training = tuple(scenario for scenario in SCENARIO_DATASET if scenario.split == "train")
    controllers = dense_controller_grid()
    store = EvaluationStore(cache_path, config)
    points: list[ControllerSweepPoint] = []
    raw_rows: list[dict[str, object]] = []
    for index, controller in enumerate(controllers, start=1):
        results: list[ScenarioEvaluation] = []
        for scenario in training:
            result = store.get(scenario, TRAINED_HARDWARE, controller)
            if result is None:
                result = evaluate_scenario(config, scenario, TRAINED_HARDWARE, controller)
                store.put(scenario, TRAINED_HARDWARE, controller, result)
            results.append(result)
            row = asdict(result)
            row["violations"] = ";".join(result.violations)
            raw_rows.append(row)
        point = ControllerSweepPoint(
            log10_lambda_energy=controller.log10_lambda_energy,
            log10_lambda_force_slew=controller.log10_lambda_force_slew,
            mean_rmse_mps=float(np.mean([result.rmse_mps for result in results])),
            mean_wh_per_km=float(np.mean([result.wh_per_km for result in results])),
            maximum_rmse_mps=max(result.rmse_mps for result in results),
            feasible_scenario_count=sum(_operationally_feasible(result) for result in results),
            scenario_count=len(results),
            operationally_feasible=all(_operationally_feasible(result) for result in results),
        )
        points.append(point)
        print(
            f"[{index:02d}/{len(controllers)}] weights=({controller.log10_lambda_energy:g},"
            f"{controller.log10_lambda_force_slew:g}) RMSE={point.mean_rmse_mps:.4f} "
            f"energy={point.mean_wh_per_km:.2f} Wh/km",
            flush=True,
        )
    flags = pareto_flags(points)
    points = [
        ControllerSweepPoint(**{**asdict(point), "pareto_optimal": flag})
        for point, flag in zip(points, flags)
    ]
    _write_csv(points, output_dir / "dense_controller_sweep.csv")
    with (output_dir / "dense_controller_raw_evaluations.csv").open(
        "w", encoding="utf-8", newline=""
    ) as handle:
        writer = csv.DictWriter(handle, fieldnames=list(raw_rows[0].keys()))
        writer.writeheader()
        writer.writerows(raw_rows)
    _plot(points, output_dir / "trained_hardware_controller_pareto.png")
    frontier = [asdict(point) for point in points if point.pareto_optimal]
    report: dict[str, object] = {
        "trained_hardware": asdict(TRAINED_HARDWARE),
        "traditional_reference": {
            "hardware": asdict(TRADITIONAL_HARDWARE),
            "mean_rmse_mps": TRADITIONAL_MEAN_RMSE_MPS,
            "mean_wh_per_km": TRADITIONAL_MEAN_WH_PER_KM,
        },
        "training_scenarios": [scenario.name for scenario in training],
        "controller_candidate_count": len(controllers),
        "closed_loop_evaluation_count": len(controllers) * len(training),
        "pareto_point_count": len(frontier),
        "pareto_frontier": frontier,
        "tracking_feasible_pareto_point_count": sum(
            point.maximum_rmse_mps <= 0.40 for point in points if point.pareto_optimal
        ),
        "cache_entry_count": len(store.values),
    }
    (output_dir / "dense_controller_sweep_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    return report


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=Path("configs/default.yaml"))
    parser.add_argument(
        "--output-dir", type=Path,
        default=Path("artifacts/trained_hardware_controller_sweep"),
    )
    parser.add_argument(
        "--cache", type=Path,
        default=Path("artifacts/generality_dataset/evaluation_cache.json"),
    )
    args = parser.parse_args()
    report = run_dense_sweep(
        ProjectConfig.from_yaml(args.config), args.output_dir, args.cache
    )
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
