"""Sample hardware designs with one fixed MPC to isolate hardware effects."""

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
from .hardware_sizing import evaluate_hardware  # noqa: E402
from .optimization import (  # noqa: E402
    ClosedLoopEvaluator,
    ControllerDesign,
    EvaluationCache,
)


FIXED_CONTROLLER = ControllerDesign(-1.0, -1.0)


@dataclass(frozen=True, slots=True)
class HardwareSensitivityResult:
    final_drive_ratio: float
    motor_scale: float
    motor_mass_kg: float
    conventional_feasible: bool
    zero_to_100_s: float
    aggregate_rmse_mps: float
    total_net_battery_wh: float
    aggregate_wh_per_km: float
    peak_acceleration_mps2: float
    peak_jerk_mps3: float
    base_control_feasible: bool
    urban_rmse_mps: float
    highway_rmse_mps: float
    mixed_grade_rmse_mps: float


def sample_hardware_designs(
    config: ProjectConfig,
    output_dir: Path,
    ratios: tuple[float, ...] = (7.0, 9.0, 11.0),
    scales: tuple[float, ...] = (0.6, 1.0, 1.4),
) -> list[HardwareSensitivityResult]:
    output_dir.mkdir(parents=True, exist_ok=True)
    evaluator = ClosedLoopEvaluator(
        config,
        EvaluationCache(output_dir / "evaluations.sqlite3"),
    )
    results: list[HardwareSensitivityResult] = []
    total = len(ratios) * len(scales)
    for index, (ratio, scale) in enumerate(
        ((ratio, scale) for scale in scales for ratio in ratios), start=1
    ):
        hardware = HardwareDesign(ratio, scale)
        sizing = evaluate_hardware(config, hardware)
        closed_loop = evaluator.evaluate(hardware, FIXED_CONTROLLER)
        scenario = closed_loop.scenario_metrics
        result = HardwareSensitivityResult(
            final_drive_ratio=ratio,
            motor_scale=scale,
            motor_mass_kg=sizing.motor_mass_kg,
            conventional_feasible=sizing.feasible,
            zero_to_100_s=sizing.zero_to_100_s,
            aggregate_rmse_mps=closed_loop.aggregate_rmse_mps,
            total_net_battery_wh=closed_loop.total_net_battery_wh,
            aggregate_wh_per_km=closed_loop.aggregate_wh_per_km,
            peak_acceleration_mps2=closed_loop.peak_acceleration_mps2,
            peak_jerk_mps3=closed_loop.peak_jerk_mps3,
            base_control_feasible=closed_loop.base_feasible,
            urban_rmse_mps=float(scenario["urban"]["rmse_mps"]),
            highway_rmse_mps=float(scenario["highway"]["rmse_mps"]),
            mixed_grade_rmse_mps=float(scenario["mixed_grade"]["rmse_mps"]),
        )
        results.append(result)
        cache_label = " cache" if closed_loop.from_cache else ""
        print(
            f"[{index:02d}/{total}]{cache_label} h=({ratio:g},{scale:g}) "
            f"RMSE={result.aggregate_rmse_mps:.3f} energy={result.total_net_battery_wh:.2f} Wh "
            f"0-100={result.zero_to_100_s:.2f} s",
            flush=True,
        )
    return results


def _matrix(
    results: list[HardwareSensitivityResult],
    ratios: list[float],
    scales: list[float],
    field: str,
) -> np.ndarray:
    values = np.full((len(scales), len(ratios)), np.nan)
    for result in results:
        values[scales.index(result.motor_scale), ratios.index(result.final_drive_ratio)] = float(
            getattr(result, field)
        )
    return values


def _heatmap(
    axis: plt.Axes,
    values: np.ndarray,
    ratios: list[float],
    scales: list[float],
    title: str,
    colorbar_label: str,
    figure: plt.Figure,
) -> None:
    image = axis.imshow(values, origin="lower", aspect="auto", cmap="viridis")
    axis.set_xticks(range(len(ratios)), [f"{value:g}" for value in ratios])
    axis.set_yticks(range(len(scales)), [f"{value:.1f}" for value in scales])
    axis.set(xlabel="Final-drive ratio", ylabel="Motor scale", title=title)
    for row in range(values.shape[0]):
        for column in range(values.shape[1]):
            value = values[row, column]
            text = "inf" if not np.isfinite(value) else f"{value:.2f}"
            axis.text(column, row, text, ha="center", va="center", color="white", fontsize=9)
    figure.colorbar(image, ax=axis).set_label(colorbar_label)


def plot_hardware_sensitivity(
    results: list[HardwareSensitivityResult], output: Path
) -> None:
    ratios = sorted({result.final_drive_ratio for result in results})
    scales = sorted({result.motor_scale for result in results})
    rmse = _matrix(results, ratios, scales, "aggregate_rmse_mps")
    energy = _matrix(results, ratios, scales, "total_net_battery_wh")
    acceleration = _matrix(results, ratios, scales, "zero_to_100_s")
    acceleration[~np.isfinite(acceleration)] = 12.0
    fig, axes = plt.subplots(2, 2, figsize=(14.5, 10.5), constrained_layout=True)
    _heatmap(axes[0, 0], rmse, ratios, scales, "Fixed-MPC tracking RMSE", "RMSE [m/s]", fig)
    _heatmap(
        axes[0, 1],
        energy,
        ratios,
        scales,
        "Fixed-MPC battery energy",
        "Net energy [Wh]",
        fig,
    )
    _heatmap(
        axes[1, 0],
        acceleration,
        ratios,
        scales,
        "Maximum-force 0-100 km/h time",
        "Time [s]; 12 denotes >10 s/infeasible",
        fig,
    )
    scatter = axes[1, 1].scatter(
        [result.aggregate_rmse_mps for result in results],
        [result.total_net_battery_wh for result in results],
        c=[result.final_drive_ratio for result in results],
        s=[90 + 100 * result.motor_scale for result in results],
        cmap="plasma",
        edgecolor="black",
    )
    for result in results:
        axes[1, 1].annotate(
            f"({result.final_drive_ratio:g},{result.motor_scale:g})",
            (result.aggregate_rmse_mps, result.total_net_battery_wh),
            xytext=(4, 4),
            textcoords="offset points",
            fontsize=8,
        )
    axes[1, 1].set(
        xlabel="Aggregate RMSE [m/s]",
        ylabel="Net battery energy [Wh]",
        title="Same controller, different hardware",
    )
    axes[1, 1].grid(alpha=0.25)
    fig.colorbar(scatter, ax=axes[1, 1]).set_label("Final-drive ratio")
    fig.suptitle("Hardware sensitivity with fixed MPC weights (-1, -1)", fontsize=17)
    fig.savefig(output, dpi=180)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=Path("configs/default.yaml"))
    parser.add_argument(
        "--output-dir", type=Path, default=Path("artifacts/hardware_sensitivity")
    )
    args = parser.parse_args()
    config = ProjectConfig.from_yaml(args.config)
    results = sample_hardware_designs(config, args.output_dir)
    with (args.output_dir / "hardware_sensitivity.csv").open(
        "w", encoding="utf-8", newline=""
    ) as handle:
        writer = csv.DictWriter(handle, fieldnames=list(asdict(results[0]).keys()))
        writer.writeheader()
        writer.writerows(asdict(result) for result in results)
    json_results = []
    for result in results:
        values = asdict(result)
        if not np.isfinite(values["zero_to_100_s"]):
            values["zero_to_100_s"] = None
        json_results.append(values)
    report = {
        "fixed_controller": asdict(FIXED_CONTROLLER),
        "sample_count": len(results),
        "results": json_results,
    }
    (args.output_dir / "hardware_sensitivity_report.json").write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    plot_hardware_sensitivity(results, args.output_dir / "hardware_sensitivity.png")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
