"""Run deterministic pre-MPC reference tracking with the temporary baseline controller."""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict
from pathlib import Path

from .config import ProjectConfig
from .metadrive_env import MetaDriveEVEnv
from .powertrain import EVPowertrain
from .scenarios import (
    HIGHWAY_PROFILE,
    URBAN_PROFILE,
    ProportionalForceController,
    run_speed_profile,
)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=Path("configs/default.yaml"))
    parser.add_argument("--profile", choices=("urban", "highway"), default="urban")
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/scenario"))
    parser.add_argument("--gain", type=float, default=1200.0)
    parser.add_argument("--traffic-density", type=float, default=0.0)
    parser.add_argument("--render", action="store_true")
    args = parser.parse_args()

    config = ProjectConfig.from_yaml(args.config)
    profile = URBAN_PROFILE if args.profile == "urban" else HIGHWAY_PROFILE
    env = MetaDriveEVEnv(
        EVPowertrain(config.hardware, config.vehicle, config.motor, config.battery),
        control_interval_s=config.control_interval_s,
        use_render=args.render,
        seed=config.seed,
        map_sequence="SSSSSSSSSSSS",
        traffic_density=args.traffic_density,
    )
    try:
        result = run_speed_profile(
            env,
            profile,
            ProportionalForceController(gain_n_per_mps=args.gain),
        )
    finally:
        env.close()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    result.write_csv(args.output_dir / f"{profile.name}_trajectory.csv")
    (args.output_dir / f"{profile.name}_metrics.json").write_text(
        json.dumps(asdict(result.metrics), indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(asdict(result.metrics), indent=2))


if __name__ == "__main__":
    main()
