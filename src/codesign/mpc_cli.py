"""Run deterministic PID/MPC comparisons and generate initial MPC evidence."""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from .config import ProjectConfig  # noqa: E402
from .controllers import CenterlinePIDController, LongitudinalPIDController  # noqa: E402
from .metadrive_env import MetaDriveEVEnv  # noqa: E402
from .mpc import LongitudinalMPCController  # noqa: E402
from .powertrain import EVPowertrain  # noqa: E402
from .scenarios import CENTERLINE_PROFILE, URBAN_PROFILE, EpisodeResult, run_speed_profile  # noqa: E402


def _run(
    config: ProjectConfig,
    profile: object,
    map_sequence: str,
    spawn_lateral_m: float,
    use_mpc: bool,
) -> tuple[EpisodeResult, LongitudinalMPCController | None]:
    powertrain = EVPowertrain(config.hardware, config.vehicle, config.motor, config.battery)
    env = MetaDriveEVEnv(
        powertrain,
        control_interval_s=config.control_interval_s,
        seed=config.seed,
        map_sequence=map_sequence,
        traffic_density=0.0,
        spawn_lateral_m=spawn_lateral_m,
    )
    mpc = LongitudinalMPCController(powertrain, dt_s=config.control_interval_s) if use_mpc else None
    controller = mpc or LongitudinalPIDController(dt_s=config.control_interval_s)
    try:
        result = run_speed_profile(
            env,
            profile,  # type: ignore[arg-type]
            controller,
            CenterlinePIDController(dt_s=config.control_interval_s),
        )
        return result, mpc
    finally:
        env.close()


def _plot(urban_pid: EpisodeResult, urban_mpc: EpisodeResult, curved_mpc: EpisodeResult, output: Path) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(13, 9), constrained_layout=True)
    for result, label in ((urban_pid, "PID"), (urban_mpc, "MPC")):
        time = [point.time_s for point in result.trajectory]
        axes[0, 0].plot(time, [point.speed_mps for point in result.trajectory], label=label)
    axes[0, 0].plot(
        [point.time_s for point in urban_mpc.trajectory],
        [point.reference_speed_mps for point in urban_mpc.trajectory],
        "k--",
        label="Curvature-aware reference",
    )
    axes[0, 0].set(title="Urban speed tracking", xlabel="Time [s]", ylabel="Speed [m/s]")
    axes[0, 0].legend()

    for result, label in ((urban_pid, "PID"), (urban_mpc, "MPC")):
        axes[0, 1].plot(
            [point.time_s for point in result.trajectory],
            [point.requested_force_n for point in result.trajectory],
            label=label,
        )
    axes[0, 1].set(title="Requested wheel force", xlabel="Time [s]", ylabel="Force [N]")
    axes[0, 1].legend()

    axes[1, 0].plot(
        [point.time_s for point in curved_mpc.trajectory],
        [point.reference_speed_mps for point in curved_mpc.trajectory],
        "k--",
        label="Reference",
    )
    axes[1, 0].plot(
        [point.time_s for point in curved_mpc.trajectory],
        [point.speed_mps for point in curved_mpc.trajectory],
        label="MPC",
    )
    axes[1, 0].set(title="Curved-route coordination", xlabel="Time [s]", ylabel="Speed [m/s]")
    axes[1, 0].legend()

    axis = axes[1, 1]
    axis.plot(
        [point.time_s for point in curved_mpc.trajectory],
        [point.lateral_error_m for point in curved_mpc.trajectory],
        label="Lateral error",
    )
    steering_axis = axis.twinx()
    steering_axis.plot(
        [point.time_s for point in curved_mpc.trajectory],
        [point.steering_command for point in curved_mpc.trajectory],
        color="tab:orange",
        label="Steering",
    )
    axis.set(title="Fixed lateral PID with feedforward", xlabel="Time [s]", ylabel="Lateral error [m]")
    steering_axis.set_ylabel("Normalized steering", color="tab:orange")
    fig.suptitle("Initial longitudinal MPC validation", fontsize=16)
    fig.savefig(output, dpi=170)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=Path("configs/default.yaml"))
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/mpc_validation"))
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    config = ProjectConfig.from_yaml(args.config)
    urban_pid, _ = _run(config, URBAN_PROFILE, "SSSSSSSSSSSS", 0.5, False)
    urban_mpc, urban_controller = _run(config, URBAN_PROFILE, "SSSSSSSSSSSS", 0.5, True)
    curved_mpc, curved_controller = _run(config, CENTERLINE_PROFILE, "SCSCSC", 1.0, True)
    if urban_controller is None or curved_controller is None:
        raise RuntimeError("MPC controller was not created")

    for name, result in (
        ("urban_pid", urban_pid),
        ("urban_mpc", urban_mpc),
        ("curved_mpc", curved_mpc),
    ):
        result.write_csv(args.output_dir / f"{name}_trajectory.csv")
    _plot(urban_pid, urban_mpc, curved_mpc, args.output_dir / "mpc_validation.png")

    checks = {
        "urban_mpc_completed": urban_mpc.metrics.completed,
        "curved_mpc_completed": curved_mpc.metrics.completed,
        "urban_rmse_below_0_4_mps": urban_mpc.metrics.rmse_mps < 0.4,
        "curved_rmse_below_0_4_mps": curved_mpc.metrics.rmse_mps < 0.4,
        "acceleration_below_3_05_mps2": max(
            urban_mpc.metrics.peak_acceleration_mps2,
            curved_mpc.metrics.peak_acceleration_mps2,
        )
        < 3.05,
        "jerk_below_4_mps3": max(
            urban_mpc.metrics.peak_jerk_mps3, curved_mpc.metrics.peak_jerk_mps3
        )
        < 4.0,
        "no_solver_fallbacks": (
            urban_controller.fallback_count == 0 and curved_controller.fallback_count == 0
        ),
        "no_safety_slack": max(
            urban_controller.maximum_safety_slack_m,
            curved_controller.maximum_safety_slack_m,
        )
        < 1e-5,
    }
    report = {
        "overall_passed": all(checks.values()),
        "checks": checks,
        "urban_pid": asdict(urban_pid.metrics),
        "urban_mpc": asdict(urban_mpc.metrics),
        "curved_mpc": asdict(curved_mpc.metrics),
        "solver": {
            "urban_solves": urban_controller.solve_count,
            "urban_fallbacks": urban_controller.fallback_count,
            "curved_solves": curved_controller.solve_count,
            "curved_fallbacks": curved_controller.fallback_count,
        },
    }
    (args.output_dir / "mpc_validation_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2))
    if not report["overall_passed"]:
        raise SystemExit("MPC validation failed")


if __name__ == "__main__":
    main()
