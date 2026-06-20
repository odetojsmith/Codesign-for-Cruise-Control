"""Animate low- and high-RMSE MPC trajectories on the urban drive cycle."""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.animation import FuncAnimation, PillowWriter  # noqa: E402

from .config import ProjectConfig  # noqa: E402
from .controllers import CenterlinePIDController  # noqa: E402
from .metadrive_env import MetaDriveEVEnv  # noqa: E402
from .mpc import LongitudinalMPCController  # noqa: E402
from .powertrain import EVPowertrain  # noqa: E402
from .scenarios import URBAN_PROFILE, EpisodeResult, run_speed_profile  # noqa: E402


LOW_RMSE_WEIGHTS = (-1.0, -1.0)
HIGH_RMSE_WEIGHTS = (1.5, 1.5)


def _run(config: ProjectConfig, weights: tuple[float, float]) -> EpisodeResult:
    powertrain = EVPowertrain(config.hardware, config.vehicle, config.motor, config.battery)
    env = MetaDriveEVEnv(
        powertrain,
        control_interval_s=config.control_interval_s,
        seed=config.seed,
        map_sequence="SSSSSSSSSSSS",
        traffic_density=0.0,
        spawn_lateral_m=0.5,
    )
    controller = LongitudinalMPCController(
        powertrain,
        dt_s=config.control_interval_s,
        lambda_energy=10.0**weights[0],
        lambda_force_slew=10.0**weights[1],
    )
    try:
        return run_speed_profile(
            env,
            URBAN_PROFILE,
            controller,
            CenterlinePIDController(dt_s=config.control_interval_s),
        )
    finally:
        env.close()


def _arrays(result: EpisodeResult) -> dict[str, np.ndarray]:
    return {
        "time": np.asarray([point.time_s for point in result.trajectory]),
        "reference": np.asarray([point.reference_speed_mps for point in result.trajectory]),
        "speed": np.asarray([point.speed_mps for point in result.trajectory]),
        "force": np.asarray([point.requested_force_n for point in result.trajectory]),
        "energy": np.asarray([point.cumulative_battery_wh for point in result.trajectory]),
        "distance": np.asarray([point.distance_m for point in result.trajectory]),
    }


def create_animation(low: EpisodeResult, high: EpisodeResult, output: Path) -> None:
    low_data = _arrays(low)
    high_data = _arrays(high)
    count = min(len(low.trajectory), len(high.trajectory))
    frames = list(range(0, count, 2))
    if frames[-1] != count - 1:
        frames.append(count - 1)

    fig = plt.figure(figsize=(13.5, 9.5), constrained_layout=True)
    grid = fig.add_gridspec(3, 2, height_ratios=(0.65, 1.7, 1.4))
    road_axis = fig.add_subplot(grid[0, :])
    speed_axis = fig.add_subplot(grid[1, :])
    force_axis = fig.add_subplot(grid[2, 0])
    energy_axis = fig.add_subplot(grid[2, 1])
    low_color = "tab:blue"
    high_color = "tab:orange"

    maximum_distance = max(low_data["distance"][-1], high_data["distance"][-1]) * 1.03
    road_axis.axhspan(-0.75, 0.75, color="0.88")
    road_axis.axhline(0.0, color="white", linestyle="--", linewidth=2)
    road_axis.set(xlim=(0.0, maximum_distance), ylim=(-1.0, 1.0), yticks=[], xlabel="Distance traveled [m]")
    road_axis.set_title("Same road and reference; different longitudinal priorities")
    low_car = road_axis.scatter([], [], marker=">", s=240, color=low_color, label="Low RMSE")
    high_car = road_axis.scatter([], [], marker=">", s=240, color=high_color, label="High RMSE / energy-oriented")
    road_axis.legend(loc="upper left", ncol=2)
    clock = road_axis.text(0.99, 0.88, "", transform=road_axis.transAxes, ha="right", fontsize=12)

    reference_line, = speed_axis.plot([], [], "k--", linewidth=2, label="Reference")
    low_speed_line, = speed_axis.plot([], [], color=low_color, linewidth=2.2, label="Low RMSE")
    high_speed_line, = speed_axis.plot([], [], color=high_color, linewidth=2.2, label="High RMSE")
    low_speed_dot, = speed_axis.plot([], [], "o", color=low_color)
    high_speed_dot, = speed_axis.plot([], [], "o", color=high_color)
    speed_axis.set(
        xlim=(0.0, low_data["time"][-1]),
        ylim=(-0.5, max(low_data["reference"]) + 2.0),
        xlabel="Time [s]",
        ylabel="Speed [m/s]",
        title="Speed trajectory",
    )
    speed_axis.grid(alpha=0.25)
    speed_axis.legend(loc="upper right")
    speed_status = speed_axis.text(
        0.01,
        0.96,
        "",
        transform=speed_axis.transAxes,
        va="top",
        bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.85},
    )

    low_force_line, = force_axis.plot([], [], color=low_color, linewidth=2)
    high_force_line, = force_axis.plot([], [], color=high_color, linewidth=2)
    force_limit = max(np.max(np.abs(low_data["force"])), np.max(np.abs(high_data["force"]))) * 1.1
    force_axis.set(
        xlim=(0.0, low_data["time"][-1]),
        ylim=(-force_limit, force_limit),
        xlabel="Time [s]",
        ylabel="Requested force [N]",
        title="Control trajectory",
    )
    force_axis.axhline(0.0, color="0.4", linewidth=0.8)
    force_axis.grid(alpha=0.25)

    low_energy_line, = energy_axis.plot([], [], color=low_color, linewidth=2)
    high_energy_line, = energy_axis.plot([], [], color=high_color, linewidth=2)
    energy_min = min(np.min(low_data["energy"]), np.min(high_data["energy"]), 0.0)
    energy_max = max(np.max(low_data["energy"]), np.max(high_data["energy"]))
    energy_axis.set(
        xlim=(0.0, low_data["time"][-1]),
        ylim=(energy_min - 2.0, energy_max * 1.08),
        xlabel="Time [s]",
        ylabel="Net battery energy [Wh]",
        title="Energy consequence",
    )
    energy_axis.grid(alpha=0.25)
    energy_status = energy_axis.text(
        0.02,
        0.95,
        "",
        transform=energy_axis.transAxes,
        va="top",
        bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.85},
    )

    fig.suptitle(
        "MPC trajectory tradeoff: low RMSE $(-1,-1)$ vs high RMSE $(1.5,1.5)$",
        fontsize=16,
    )

    def update(index: int) -> tuple[object, ...]:
        stop = index + 1
        time = low_data["time"][:stop]
        reference_line.set_data(time, low_data["reference"][:stop])
        low_speed_line.set_data(time, low_data["speed"][:stop])
        high_speed_line.set_data(time, high_data["speed"][:stop])
        low_speed_dot.set_data([time[-1]], [low_data["speed"][index]])
        high_speed_dot.set_data([time[-1]], [high_data["speed"][index]])
        low_force_line.set_data(time, low_data["force"][:stop])
        high_force_line.set_data(time, high_data["force"][:stop])
        low_energy_line.set_data(time, low_data["energy"][:stop])
        high_energy_line.set_data(time, high_data["energy"][:stop])
        low_car.set_offsets([[low_data["distance"][index], 0.38]])
        high_car.set_offsets([[high_data["distance"][index], -0.38]])
        clock.set_text(f"t = {time[-1]:4.1f} s")
        low_error = low_data["speed"][index] - low_data["reference"][index]
        high_error = high_data["speed"][index] - high_data["reference"][index]
        speed_status.set_text(
            f"Instantaneous error\nLow RMSE: {low_error:+.2f} m/s\nHigh RMSE: {high_error:+.2f} m/s"
        )
        energy_status.set_text(
            f"Energy so far\nLow RMSE: {low_data['energy'][index]:.1f} Wh\n"
            f"High RMSE: {high_data['energy'][index]:.1f} Wh"
        )
        return (
            reference_line,
            low_speed_line,
            high_speed_line,
            low_speed_dot,
            high_speed_dot,
            low_force_line,
            high_force_line,
            low_energy_line,
            high_energy_line,
            low_car,
            high_car,
            clock,
            speed_status,
            energy_status,
        )

    animation = FuncAnimation(fig, update, frames=frames, interval=125, blit=False, repeat=True)
    animation.save(output, writer=PillowWriter(fps=8), dpi=105)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=Path("configs/default.yaml"))
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/trajectory_animation"))
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    config = ProjectConfig.from_yaml(args.config)
    low = _run(config, LOW_RMSE_WEIGHTS)
    high = _run(config, HIGH_RMSE_WEIGHTS)
    low.write_csv(args.output_dir / "low_rmse_trajectory.csv")
    high.write_csv(args.output_dir / "high_rmse_trajectory.csv")
    create_animation(low, high, args.output_dir / "rmse_trajectory_comparison.gif")
    report = {
        "low_rmse": {"log10_weights": LOW_RMSE_WEIGHTS, "metrics": asdict(low.metrics)},
        "high_rmse": {"log10_weights": HIGH_RMSE_WEIGHTS, "metrics": asdict(high.metrics)},
    }
    (args.output_dir / "trajectory_comparison.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
