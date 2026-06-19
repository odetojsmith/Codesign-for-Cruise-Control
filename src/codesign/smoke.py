"""Command-line smoke tests for the core model and optional MetaDrive backend."""

from __future__ import annotations

import argparse
from pathlib import Path

from .config import ProjectConfig
from .metadrive_env import MetaDriveEVEnv
from .powertrain import EVPowertrain, EnergyState


def _default_config_path() -> Path:
    return Path(__file__).resolve().parents[2] / "configs" / "default.yaml"


def run_core(config: ProjectConfig) -> EnergyState:
    powertrain = EVPowertrain(config.hardware, config.vehicle, config.motor, config.battery)
    energy = EnergyState()
    for speed_mps, force_n in [(0.0, 2500.0), (10.0, 3000.0), (20.0, 1500.0), (15.0, -2000.0)]:
        step = powertrain.evaluate(force_n, speed_mps)
        energy.update(step, config.control_interval_s)
    print(
        f"core smoke OK: mass={powertrain.total_vehicle_mass_kg:.1f} kg, "
        f"net={energy.net_battery_wh:.3f} Wh, regen={energy.regenerated_wh:.3f} Wh"
    )
    return energy


def run_metadrive(config: ProjectConfig, steps: int = 20) -> None:
    env = MetaDriveEVEnv(
        EVPowertrain(config.hardware, config.vehicle, config.motor, config.battery),
        control_interval_s=config.control_interval_s,
        use_render=config.use_render,
        seed=config.seed,
    )
    try:
        env.reset()
        for _ in range(steps):
            _, _, terminated, truncated, _ = env.step((0.0, 1800.0))
            if terminated or truncated:
                break
        print(f"MetaDrive smoke OK: net={env.energy.net_battery_wh:.3f} Wh")
    finally:
        env.close()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=_default_config_path())
    parser.add_argument("--core-only", action="store_true")
    parser.add_argument("--metadrive", action="store_true")
    args = parser.parse_args()

    config = ProjectConfig.from_yaml(args.config)
    run_core(config)
    if args.metadrive and not args.core_only:
        run_metadrive(config)


if __name__ == "__main__":
    main()

