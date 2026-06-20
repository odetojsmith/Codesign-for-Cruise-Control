"""Conventional two-stage hardware sizing followed by frozen-hardware MPC tuning."""

from __future__ import annotations

import argparse
import csv
import json
from dataclasses import asdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from .config import HardwareDesign, ProjectConfig  # noqa: E402
from .hardware_sizing import HardwareSizingResult, size_hardware  # noqa: E402
from .optimization import (  # noqa: E402
    ClosedLoopEvaluator,
    ControllerDesign,
    EvaluationCache,
    EvaluationSummary,
)


DEFAULT_RMSE_BOUNDS = (0.1, 0.2, 0.4, 0.8, 1.5)


def controller_grid(levels: tuple[float, ...]) -> tuple[ControllerDesign, ...]:
    return tuple(ControllerDesign(energy, slew) for energy in levels for slew in levels)


def pareto_flags(results: list[EvaluationSummary]) -> list[bool]:
    flags: list[bool] = []
    for candidate in results:
        dominated = any(
            other.base_feasible
            and other.key != candidate.key
            and other.aggregate_rmse_mps <= candidate.aggregate_rmse_mps
            and other.total_net_battery_wh <= candidate.total_net_battery_wh
            and (
                other.aggregate_rmse_mps < candidate.aggregate_rmse_mps
                or other.total_net_battery_wh < candidate.total_net_battery_wh
            )
            for other in results
        )
        flags.append(candidate.base_feasible and not dominated)
    return flags


def select_for_bounds(
    results: list[EvaluationSummary],
    bounds: tuple[float, ...] = DEFAULT_RMSE_BOUNDS,
) -> dict[float, EvaluationSummary | None]:
    selections: dict[float, EvaluationSummary | None] = {}
    for bound in bounds:
        feasible = [
            result
            for result in results
            if result.base_feasible and result.aggregate_rmse_mps <= bound
        ]
        selections[bound] = (
            None
            if not feasible
            else min(
                feasible,
                key=lambda result: (
                    result.total_net_battery_wh,
                    result.aggregate_rmse_mps,
                ),
            )
        )
    return selections


def _plot(results: list[EvaluationSummary], output: Path) -> None:
    flags = pareto_flags(results)
    feasible = [result for result in results if result.base_feasible]
    infeasible = [result for result in results if not result.base_feasible]
    frontier = sorted(
        [result for result, flag in zip(results, flags) if flag],
        key=lambda result: result.aggregate_rmse_mps,
    )
    fig, axis = plt.subplots(figsize=(9.5, 6.5), constrained_layout=True)
    scatter = axis.scatter(
        [result.aggregate_rmse_mps for result in feasible],
        [result.total_net_battery_wh for result in feasible],
        c=[result.controller.log10_lambda_energy for result in feasible],
        cmap="viridis",
        edgecolor="white",
        label="Feasible controller",
    )
    if infeasible:
        axis.scatter(
            [result.aggregate_rmse_mps for result in infeasible],
            [result.total_net_battery_wh for result in infeasible],
            marker="x",
            color="0.55",
            label="Constraint violation",
        )
    if frontier:
        axis.plot(
            [result.aggregate_rmse_mps for result in frontier],
            [result.total_net_battery_wh for result in frontier],
            "r-o",
            label="Separate-design frontier",
        )
    axis.set(
        xlabel="Aggregate speed RMSE [m/s]",
        ylabel="Training-scenario net battery energy [Wh]",
        title="Frozen-hardware controller tuning",
    )
    axis.grid(alpha=0.25)
    axis.legend()
    colorbar = fig.colorbar(scatter, ax=axis)
    colorbar.set_label(r"$\log_{10}\lambda_E$")
    fig.savefig(output, dpi=180)
    plt.close(fig)


def run_separate_optimization(
    config: ProjectConfig,
    output_dir: Path,
    levels: tuple[float, ...],
    bounds: tuple[float, ...] = DEFAULT_RMSE_BOUNDS,
) -> tuple[list[EvaluationSummary], dict[float, EvaluationSummary | None]]:
    output_dir.mkdir(parents=True, exist_ok=True)
    _, selected_hardware = size_hardware(config)
    cache = EvaluationCache(output_dir / "evaluations.sqlite3")
    evaluator = ClosedLoopEvaluator(config, cache)
    results: list[EvaluationSummary] = []
    candidates = controller_grid(levels)
    for index, candidate in enumerate(candidates, start=1):
        result = evaluator.evaluate(selected_hardware_to_design(selected_hardware), candidate)
        results.append(result)
        cache_label = " cache" if result.from_cache else ""
        print(
            f"[{index:02d}/{len(candidates)}]{cache_label} "
            f"weights=({candidate.log10_lambda_energy:g},{candidate.log10_lambda_force_slew:g}) "
            f"RMSE={result.aggregate_rmse_mps:.3f} energy={result.total_net_battery_wh:.2f} Wh",
            flush=True,
        )
    selections = select_for_bounds(results, bounds)
    with (output_dir / "separate_controller_sweep.csv").open(
        "w", encoding="utf-8", newline=""
    ) as handle:
        rows = [
            {
                **asdict(result),
                "hardware": json.dumps(asdict(result.hardware), sort_keys=True),
                "controller": json.dumps(asdict(result.controller), sort_keys=True),
                "violations": ";".join(result.violations),
                "scenario_metrics": json.dumps(result.scenario_metrics, sort_keys=True),
            }
            for result in results
        ]
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    report = {
        "conventional_hardware": asdict(selected_hardware),
        "rmse_bounds_mps": bounds,
        "selections": {
            str(bound): None if result is None else asdict(result)
            for bound, result in selections.items()
        },
        "cache_entries": cache.count(),
    }
    (output_dir / "separate_optimization_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    _plot(results, output_dir / "separate_design_pareto.png")
    return results, selections


def selected_hardware_to_design(result: HardwareSizingResult) -> HardwareDesign:
    return HardwareDesign(
        final_drive_ratio=result.final_drive_ratio,
        motor_scale=result.motor_scale,
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=Path("configs/default.yaml"))
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/separate_optimization"))
    parser.add_argument(
        "--quick",
        action="store_true",
        help="Use a 3x3 controller grid instead of the full 5x5 grid",
    )
    args = parser.parse_args()
    levels = (-1.0, 0.0, 1.5) if args.quick else (-3.0, -1.0, 0.0, 1.5, 3.0)
    run_separate_optimization(
        ProjectConfig.from_yaml(args.config),
        args.output_dir,
        levels,
    )


if __name__ == "__main__":
    main()
