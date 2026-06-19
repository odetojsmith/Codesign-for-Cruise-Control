"""Live MetaDrive actuator-response calibration."""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

from .config import ProjectConfig
from .metadrive_env import MetaDriveEVEnv
from .powertrain import EVPowertrain


@dataclass(frozen=True, slots=True)
class CalibrationSample:
    mode: str
    requested_force_n: float
    mean_acceleration_mps2: float
    equivalent_force_n: float
    force_gain: float
    start_speed_mps: float
    end_speed_mps: float


def _measure_acceleration(
    env: MetaDriveEVEnv,
    requested_force_n: float,
    duration_s: float,
    discard_s: float,
) -> CalibrationSample:
    env.reset()
    dt = env.control_interval_s
    speeds: list[float] = [env.speed_mps]
    times: list[float] = [0.0]
    for index in range(1, int(round(duration_s / dt)) + 1):
        _, _, terminated, truncated, _ = env.step((0.0, requested_force_n))
        speeds.append(env.speed_mps)
        times.append(index * dt)
        if terminated or truncated:
            break

    selected = [(t, v) for t, v in zip(times, speeds) if t >= discard_s]
    if len(selected) < 2:
        raise RuntimeError("calibration episode ended before enough samples were collected")
    selected_times, selected_speeds = zip(*selected)
    acceleration = float(np.polyfit(selected_times, selected_speeds, 1)[0])
    equivalent_force = acceleration * env.powertrain.total_vehicle_mass_kg
    return CalibrationSample(
        mode="traction",
        requested_force_n=requested_force_n,
        mean_acceleration_mps2=acceleration,
        equivalent_force_n=equivalent_force,
        force_gain=equivalent_force / requested_force_n,
        start_speed_mps=speeds[0],
        end_speed_mps=speeds[-1],
    )


def _measure_braking(
    env: MetaDriveEVEnv,
    requested_force_n: float,
    target_speed_mps: float = 12.0,
    duration_s: float = 2.0,
) -> CalibrationSample:
    env.reset()
    dt = env.control_interval_s
    launch_force = 0.6 * env.maximum_traction_force_n
    for _ in range(int(round(12.0 / dt))):
        _, _, terminated, truncated, _ = env.step((0.0, launch_force))
        if terminated or truncated:
            raise RuntimeError("calibration route ended during braking launch phase")
        if env.speed_mps >= target_speed_mps:
            break
    else:
        raise RuntimeError("vehicle did not reach braking calibration speed")

    speeds: list[float] = [env.speed_mps]
    times: list[float] = [0.0]
    for index in range(1, int(round(duration_s / dt)) + 1):
        _, _, terminated, truncated, _ = env.step((0.0, requested_force_n))
        speeds.append(env.speed_mps)
        times.append(index * dt)
        if terminated or truncated or env.speed_mps <= 0.5:
            break
    if len(speeds) < 3:
        raise RuntimeError("braking calibration ended before enough samples were collected")
    acceleration = float(np.polyfit(times, speeds, 1)[0])
    equivalent_force = acceleration * env.powertrain.total_vehicle_mass_kg
    return CalibrationSample(
        mode="regeneration",
        requested_force_n=requested_force_n,
        mean_acceleration_mps2=acceleration,
        equivalent_force_n=equivalent_force,
        force_gain=equivalent_force / requested_force_n,
        start_speed_mps=speeds[0],
        end_speed_mps=speeds[-1],
    )


def run_actuator_calibration(
    config: ProjectConfig,
    traction_fractions: tuple[float, ...] = (0.25, 0.5, 0.75, 1.0),
    regeneration_fractions: tuple[float, ...] = (0.25, 0.5, 0.75, 1.0),
) -> tuple[CalibrationSample, ...]:
    powertrain = EVPowertrain(config.hardware, config.vehicle, config.motor, config.battery)
    env = MetaDriveEVEnv(
        powertrain,
        control_interval_s=config.control_interval_s,
        use_render=False,
        seed=config.seed,
        map_sequence="SSSSSSSS",
        traffic_density=0.0,
    )
    try:
        maximum_force = env.maximum_traction_force_n
        maximum_regeneration_force = env.maximum_regenerative_force_n
        traction = tuple(
            _measure_acceleration(env, fraction * maximum_force, duration_s=3.0, discard_s=0.6)
            for fraction in traction_fractions
        )
        regeneration = tuple(
            _measure_braking(env, -fraction * maximum_regeneration_force)
            for fraction in regeneration_fractions
        )
        return traction + regeneration
    finally:
        env.close()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=Path("configs/default.yaml"))
    parser.add_argument("--output", type=Path, default=Path("artifacts/actuator_calibration.json"))
    args = parser.parse_args()
    samples = run_actuator_calibration(ProjectConfig.from_yaml(args.config))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps([asdict(sample) for sample in samples], indent=2) + "\n", encoding="utf-8"
    )
    for sample in samples:
        print(
            f"{sample.mode:12s} force={sample.requested_force_n:8.1f} N  "
            f"accel={sample.mean_acceleration_mps2:6.3f} m/s^2  gain={sample.force_gain:6.3f}"
        )


if __name__ == "__main__":
    main()
