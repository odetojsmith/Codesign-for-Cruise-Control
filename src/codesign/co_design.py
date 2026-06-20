"""Nested-grid and alternating hardware-controller co-design searches."""

from __future__ import annotations

import argparse
import csv
import json
from dataclasses import asdict, dataclass
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
from .separate_optimization import (  # noqa: E402
    DEFAULT_RMSE_BOUNDS,
    controller_grid,
    pareto_flags,
    select_for_bounds,
)


@dataclass(frozen=True, slots=True)
class AlternatingIteration:
    rmse_bound_mps: float
    iteration: int
    final_drive_ratio: float
    motor_scale: float
    log10_lambda_energy: float
    log10_lambda_force_slew: float
    aggregate_rmse_mps: float
    total_net_battery_wh: float
    relative_improvement: float | None


def _hardware_design(result: HardwareSizingResult) -> HardwareDesign:
    return HardwareDesign(result.final_drive_ratio, result.motor_scale)


def _evaluate_grid(
    evaluator: ClosedLoopEvaluator,
    hardware_candidates: tuple[HardwareDesign, ...],
    controller_candidates: tuple[ControllerDesign, ...],
) -> list[EvaluationSummary]:
    total = len(hardware_candidates) * len(controller_candidates)
    results: list[EvaluationSummary] = []
    index = 0
    for hardware in hardware_candidates:
        for controller in controller_candidates:
            index += 1
            result = evaluator.evaluate(hardware, controller)
            results.append(result)
            cache_label = " cache" if result.from_cache else ""
            print(
                f"[{index:04d}/{total}]{cache_label} h=({hardware.final_drive_ratio:g},"
                f"{hardware.motor_scale:g}) c=({controller.log10_lambda_energy:g},"
                f"{controller.log10_lambda_force_slew:g}) RMSE={result.aggregate_rmse_mps:.3f} "
                f"energy={result.total_net_battery_wh:.2f} Wh",
                flush=True,
            )
    return results


def _best(
    results: list[EvaluationSummary], bound: float
) -> EvaluationSummary | None:
    feasible = [
        result
        for result in results
        if result.base_feasible and result.aggregate_rmse_mps <= bound
    ]
    return (
        None
        if not feasible
        else min(
            feasible,
            key=lambda result: (result.total_net_battery_wh, result.aggregate_rmse_mps),
        )
    )


def alternating_search(
    evaluator: ClosedLoopEvaluator,
    initial_hardware: HardwareDesign,
    initial_controller: ControllerDesign,
    hardware_candidates: tuple[HardwareDesign, ...],
    controller_candidates: tuple[ControllerDesign, ...],
    bounds: tuple[float, ...] = DEFAULT_RMSE_BOUNDS,
    maximum_iterations: int = 6,
    relative_tolerance: float = 0.001,
) -> dict[float, list[AlternatingIteration]]:
    histories: dict[float, list[AlternatingIteration]] = {}
    for bound in bounds:
        hardware = initial_hardware
        controller = initial_controller
        previous_energy: float | None = None
        history: list[AlternatingIteration] = []
        for iteration in range(1, maximum_iterations + 1):
            controller_results = [
                evaluator.evaluate(hardware, candidate) for candidate in controller_candidates
            ]
            controller_best = _best(controller_results, bound)
            if controller_best is None:
                break
            controller = controller_best.controller
            hardware_results = [
                evaluator.evaluate(candidate, controller) for candidate in hardware_candidates
            ]
            hardware_best = _best(hardware_results, bound)
            if hardware_best is None:
                break
            hardware = hardware_best.hardware
            improvement = (
                None
                if previous_energy is None
                else (previous_energy - hardware_best.total_net_battery_wh) / previous_energy
            )
            history.append(
                AlternatingIteration(
                    rmse_bound_mps=bound,
                    iteration=iteration,
                    final_drive_ratio=hardware.final_drive_ratio,
                    motor_scale=hardware.motor_scale,
                    log10_lambda_energy=controller.log10_lambda_energy,
                    log10_lambda_force_slew=controller.log10_lambda_force_slew,
                    aggregate_rmse_mps=hardware_best.aggregate_rmse_mps,
                    total_net_battery_wh=hardware_best.total_net_battery_wh,
                    relative_improvement=improvement,
                )
            )
            if improvement is not None and improvement >= 0.0 and improvement < relative_tolerance:
                break
            previous_energy = hardware_best.total_net_battery_wh
        histories[bound] = history
    return histories


def _write_results(results: list[EvaluationSummary], output: Path) -> None:
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
    with output.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def _plot_comparison(
    separate_results: list[EvaluationSummary],
    codesign_results: list[EvaluationSummary],
    output: Path,
) -> None:
    fig, axis = plt.subplots(figsize=(9.5, 6.5), constrained_layout=True)
    for results, color, label in (
        (separate_results, "tab:blue", "Separate design"),
        (codesign_results, "tab:orange", "Integrated co-design"),
    ):
        flags = pareto_flags(results)
        frontier = sorted(
            [result for result, flag in zip(results, flags) if flag],
            key=lambda result: result.aggregate_rmse_mps,
        )
        axis.scatter(
            [result.aggregate_rmse_mps for result in results if result.base_feasible],
            [result.total_net_battery_wh for result in results if result.base_feasible],
            color=color,
            alpha=0.35,
        )
        if frontier:
            axis.plot(
                [result.aggregate_rmse_mps for result in frontier],
                [result.total_net_battery_wh for result in frontier],
                "-o",
                color=color,
                linewidth=2,
                label=label,
            )
    axis.set(
        xlabel="Aggregate speed RMSE [m/s]",
        ylabel="Training-scenario net battery energy [Wh]",
        title="Separate hardware/controller design versus integrated co-design",
    )
    axis.grid(alpha=0.25)
    axis.legend()
    fig.savefig(output, dpi=180)
    plt.close(fig)


def run_codesign(
    config: ProjectConfig,
    output_dir: Path,
    quick: bool = False,
) -> dict[str, object]:
    output_dir.mkdir(parents=True, exist_ok=True)
    sizing_results, conventional = size_hardware(config)
    feasible_sizing = [result for result in sizing_results if result.feasible]
    if quick:
        ratios = {8.0, 10.0, 11.0}
        scales = {0.6, 1.0}
        feasible_sizing = [
            result
            for result in feasible_sizing
            if result.final_drive_ratio in ratios and round(result.motor_scale, 1) in scales
        ]
        levels = (-1.0, 0.0, 1.5)
    else:
        levels = (-3.0, -1.0, 0.0, 1.5, 3.0)
    if conventional not in feasible_sizing:
        feasible_sizing.append(conventional)
    hardware_candidates = tuple(_hardware_design(result) for result in feasible_sizing)
    controller_candidates = controller_grid(levels)
    conventional_hardware = _hardware_design(conventional)
    cache = EvaluationCache(output_dir / "evaluations.sqlite3")
    evaluator = ClosedLoopEvaluator(config, cache)
    separate_results = _evaluate_grid(
        evaluator, (conventional_hardware,), controller_candidates
    )
    codesign_results = _evaluate_grid(
        evaluator, hardware_candidates, controller_candidates
    )
    separate_selections = select_for_bounds(separate_results)
    codesign_selections = select_for_bounds(codesign_results)
    alternating = alternating_search(
        evaluator,
        conventional_hardware,
        ControllerDesign(0.0, -1.0),
        hardware_candidates,
        controller_candidates,
    )
    _write_results(separate_results, output_dir / "separate_results.csv")
    _write_results(codesign_results, output_dir / "codesign_results.csv")
    _plot_comparison(
        separate_results,
        codesign_results,
        output_dir / "separate_vs_codesign_pareto.png",
    )
    report: dict[str, object] = {
        "mode": "quick" if quick else "full",
        "conventional_hardware": asdict(conventional_hardware),
        "hardware_candidate_count": len(hardware_candidates),
        "controller_candidate_count": len(controller_candidates),
        "separate_selections": {
            str(bound): None if result is None else asdict(result)
            for bound, result in separate_selections.items()
        },
        "codesign_selections": {
            str(bound): None if result is None else asdict(result)
            for bound, result in codesign_selections.items()
        },
        "alternating_histories": {
            str(bound): [asdict(item) for item in history]
            for bound, history in alternating.items()
        },
        "cache_entries": cache.count(),
    }
    (output_dir / "codesign_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    return report


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=Path("configs/default.yaml"))
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/codesign"))
    parser.add_argument("--quick", action="store_true")
    args = parser.parse_args()
    report = run_codesign(
        ProjectConfig.from_yaml(args.config),
        args.output_dir,
        quick=args.quick,
    )
    print(
        json.dumps(
            {
                "mode": report["mode"],
                "hardware_candidate_count": report["hardware_candidate_count"],
                "controller_candidate_count": report["controller_candidate_count"],
                "cache_entries": report["cache_entries"],
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
