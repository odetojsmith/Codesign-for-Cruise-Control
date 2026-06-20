"""Sample MPC weights in live MetaDrive and extract an energy/tracking Pareto frontier."""

from __future__ import annotations

import argparse
import csv
import json
from dataclasses import asdict, dataclass
from math import sqrt
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from .config import ProjectConfig  # noqa: E402
from .controllers import CenterlinePIDController  # noqa: E402
from .metadrive_env import MetaDriveEVEnv  # noqa: E402
from .mpc import LongitudinalMPCController  # noqa: E402
from .powertrain import EVPowertrain  # noqa: E402
from .scenarios import CENTERLINE_PROFILE, URBAN_PROFILE, EpisodeResult, SpeedProfile, run_speed_profile  # noqa: E402


@dataclass(frozen=True, slots=True)
class WeightCandidate:
    log10_lambda_energy: float
    log10_lambda_force_slew: float

    @property
    def key(self) -> str:
        return f"e{self.log10_lambda_energy:+.1f}_s{self.log10_lambda_force_slew:+.1f}"


@dataclass(frozen=True, slots=True)
class SweepResult:
    key: str
    log10_lambda_energy: float
    log10_lambda_force_slew: float
    aggregate_rmse_mps: float
    total_net_battery_wh: float
    aggregate_wh_per_km: float
    urban_rmse_mps: float
    curved_rmse_mps: float
    peak_acceleration_mps2: float
    peak_jerk_mps3: float
    maximum_lateral_error_m: float
    fallback_count: int
    maximum_safety_slack_m: float
    completed: bool
    feasible: bool
    pareto_optimal: bool = False


def pareto_mask(results: list[SweepResult]) -> list[bool]:
    """Return nondominated flags for minimizing RMSE and battery energy."""

    flags = []
    for candidate in results:
        if not candidate.feasible:
            flags.append(False)
            continue
        dominated = any(
            other.feasible
            and other.key != candidate.key
            and other.aggregate_rmse_mps <= candidate.aggregate_rmse_mps
            and other.total_net_battery_wh <= candidate.total_net_battery_wh
            and (
                other.aggregate_rmse_mps < candidate.aggregate_rmse_mps
                or other.total_net_battery_wh < candidate.total_net_battery_wh
            )
            for other in results
        )
        flags.append(not dominated)
    return flags


def _evaluate_scenario(
    config: ProjectConfig,
    candidates: tuple[WeightCandidate, ...],
    profile: SpeedProfile,
    map_sequence: str,
    spawn_lateral_m: float,
) -> dict[str, tuple[EpisodeResult, int, float]]:
    powertrain = EVPowertrain(config.hardware, config.vehicle, config.motor, config.battery)
    env = MetaDriveEVEnv(
        powertrain,
        control_interval_s=config.control_interval_s,
        seed=config.seed,
        map_sequence=map_sequence,
        traffic_density=0.0,
        spawn_lateral_m=spawn_lateral_m,
    )
    outputs: dict[str, tuple[EpisodeResult, int, float]] = {}
    try:
        for index, candidate in enumerate(candidates, start=1):
            controller = LongitudinalMPCController(
                powertrain,
                dt_s=config.control_interval_s,
                lambda_energy=10.0**candidate.log10_lambda_energy,
                lambda_force_slew=10.0**candidate.log10_lambda_force_slew,
            )
            result = run_speed_profile(
                env,
                profile,
                controller,
                CenterlinePIDController(dt_s=config.control_interval_s),
            )
            outputs[candidate.key] = (
                result,
                controller.fallback_count,
                controller.maximum_safety_slack_m,
            )
            print(
                f"[{profile.name} {index:02d}/{len(candidates)}] {candidate.key} "
                f"RMSE={result.metrics.rmse_mps:.3f} energy={result.metrics.net_battery_wh:.2f} Wh",
                flush=True,
            )
    finally:
        env.close()
    return outputs


def _combine(
    candidate: WeightCandidate,
    urban: tuple[EpisodeResult, int, float],
    curved: tuple[EpisodeResult, int, float],
) -> SweepResult:
    urban_result, urban_fallbacks, urban_slack = urban
    curved_result, curved_fallbacks, curved_slack = curved
    points = urban_result.trajectory + curved_result.trajectory
    rmse = sqrt(
        sum((point.speed_mps - point.reference_speed_mps) ** 2 for point in points)
        / len(points)
    )
    energy = urban_result.metrics.net_battery_wh + curved_result.metrics.net_battery_wh
    distance = urban_result.metrics.distance_m + curved_result.metrics.distance_m
    peak_acceleration = max(
        urban_result.metrics.peak_acceleration_mps2,
        curved_result.metrics.peak_acceleration_mps2,
    )
    peak_jerk = max(urban_result.metrics.peak_jerk_mps3, curved_result.metrics.peak_jerk_mps3)
    lateral = max(
        urban_result.metrics.maximum_abs_lateral_error_m,
        curved_result.metrics.maximum_abs_lateral_error_m,
    )
    fallbacks = urban_fallbacks + curved_fallbacks
    slack = max(urban_slack, curved_slack)
    completed = urban_result.metrics.completed and curved_result.metrics.completed
    feasible = (
        completed
        and peak_acceleration <= 3.05
        and peak_jerk <= 4.0
        and lateral < 1.75
        and fallbacks == 0
    )
    return SweepResult(
        key=candidate.key,
        log10_lambda_energy=candidate.log10_lambda_energy,
        log10_lambda_force_slew=candidate.log10_lambda_force_slew,
        aggregate_rmse_mps=rmse,
        total_net_battery_wh=energy,
        aggregate_wh_per_km=energy / (distance / 1000.0),
        urban_rmse_mps=urban_result.metrics.rmse_mps,
        curved_rmse_mps=curved_result.metrics.rmse_mps,
        peak_acceleration_mps2=peak_acceleration,
        peak_jerk_mps3=peak_jerk,
        maximum_lateral_error_m=lateral,
        fallback_count=fallbacks,
        maximum_safety_slack_m=slack,
        completed=completed,
        feasible=feasible,
    )


def _plot(results: list[SweepResult], output: Path) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(15.5, 6.5), constrained_layout=True)
    feasible = [result for result in results if result.feasible]
    infeasible = [result for result in results if not result.feasible]
    frontier = sorted(
        (result for result in results if result.pareto_optimal),
        key=lambda result: result.aggregate_rmse_mps,
    )
    default = next(
        (
            result
            for result in results
            if result.log10_lambda_energy == 0.0
            and result.log10_lambda_force_slew == -1.0
        ),
        None,
    )
    scatter = None
    for axis, title in zip(axes, ("Full sampled range", "Practical tracking region")):
        scatter = axis.scatter(
            [result.aggregate_rmse_mps for result in feasible],
            [result.total_net_battery_wh for result in feasible],
            c=[result.log10_lambda_energy for result in feasible],
            s=[55 + 12 * (result.log10_lambda_force_slew + 3) for result in feasible],
            cmap="viridis",
            vmin=-3.0,
            vmax=3.0,
            edgecolor="white",
            linewidth=0.7,
            label="Feasible samples",
        )
        if infeasible:
            axis.scatter(
                [result.aggregate_rmse_mps for result in infeasible],
                [result.total_net_battery_wh for result in infeasible],
                marker="x",
                color="0.55",
                label="Constraint violation",
            )
        axis.plot(
            [result.aggregate_rmse_mps for result in frontier],
            [result.total_net_battery_wh for result in frontier],
            "r-o",
            linewidth=2.0,
            markersize=5,
            label="Sampled Pareto frontier",
        )
        if default is not None:
            axis.scatter(
                [default.aggregate_rmse_mps],
                [default.total_net_battery_wh],
                marker="*",
                s=240,
                color="black",
                label="Current default (0, -1)",
                zorder=5,
            )
        axis.set(
            title=title,
            xlabel="Aggregate speed-tracking RMSE [m/s]",
            ylabel="Urban + curved net battery energy [Wh]",
        )
        axis.grid(alpha=0.25)
    axes[1].set_xlim(0.18, 1.6)
    axes[1].set_ylim(68.0, 96.0)
    for result in frontier:
        if result.aggregate_rmse_mps <= 1.6:
            axes[1].annotate(
                f"({result.log10_lambda_energy:g}, {result.log10_lambda_force_slew:g})",
                (result.aggregate_rmse_mps, result.total_net_battery_wh),
                xytext=(5, 5),
                textcoords="offset points",
                fontsize=8,
            )
    if scatter is None:
        raise RuntimeError("no feasible sweep points to plot")
    colorbar = fig.colorbar(scatter, ax=axes)
    colorbar.set_label(r"$\log_{10}\lambda_E$")
    axes[0].legend()
    fig.suptitle(
        "Live MetaDrive MPC-weight sweep (marker size increases with force-slew weight)",
        fontsize=15,
    )
    fig.savefig(output, dpi=180)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=Path("configs/default.yaml"))
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/mpc_sweep"))
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    config = ProjectConfig.from_yaml(args.config)
    levels = (-3.0, -1.0, 0.0, 1.5, 3.0)
    candidates = tuple(WeightCandidate(energy, slew) for energy in levels for slew in levels)
    urban = _evaluate_scenario(
        config, candidates, URBAN_PROFILE, "SSSSSSSSSSSS", spawn_lateral_m=0.5
    )
    curved = _evaluate_scenario(
        config, candidates, CENTERLINE_PROFILE, "SCSCSC", spawn_lateral_m=1.0
    )
    combined = [_combine(candidate, urban[candidate.key], curved[candidate.key]) for candidate in candidates]
    flags = pareto_mask(combined)
    results = [
        SweepResult(**{**asdict(result), "pareto_optimal": flag})
        for result, flag in zip(combined, flags)
    ]
    _plot(results, args.output_dir / "mpc_pareto_frontier.png")
    with (args.output_dir / "mpc_weight_sweep.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(asdict(results[0]).keys()))
        writer.writeheader()
        writer.writerows(asdict(result) for result in results)
    frontier = sorted(
        (result for result in results if result.pareto_optimal),
        key=lambda result: result.aggregate_rmse_mps,
    )
    default = next(
        result
        for result in results
        if result.log10_lambda_energy == 0.0 and result.log10_lambda_force_slew == -1.0
    )
    summary = {
        "sample_count": len(results),
        "feasible_count": sum(result.feasible for result in results),
        "default": asdict(default),
        "pareto_frontier": [asdict(result) for result in frontier],
        "minimum_energy": asdict(min(results, key=lambda result: result.total_net_battery_wh)),
        "minimum_rmse": asdict(min(results, key=lambda result: result.aggregate_rmse_mps)),
    }
    (args.output_dir / "mpc_sweep_summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
