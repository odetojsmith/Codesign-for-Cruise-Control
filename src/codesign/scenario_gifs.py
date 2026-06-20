"""Generate annotated top-down GIFs for the urban and curved MPC test scenarios."""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from .config import ProjectConfig
from .controllers import CenterlinePIDController
from .metadrive_env import MetaDriveEVEnv
from .mpc import LongitudinalMPCController
from .powertrain import EVPowertrain
from .scenarios import CENTERLINE_PROFILE, URBAN_PROFILE, EpisodeResult, SpeedProfile, run_speed_profile


def _font(size: int) -> ImageFont.ImageFont:
    try:
        return ImageFont.load_default(size=size)
    except TypeError:  # pragma: no cover - older Pillow fallback
        return ImageFont.load_default()


def _annotate(
    frame: np.ndarray[Any, Any],
    title: str,
    time_s: float,
    reference_speed_mps: float,
    speed_mps: float,
    energy_wh: float,
) -> Image.Image:
    image = Image.fromarray(np.asarray(frame, dtype=np.uint8))
    draw = ImageDraw.Draw(image, "RGBA")
    draw.rounded_rectangle((12, 12, 430, 94), radius=10, fill=(255, 255, 255, 225), outline=(30, 30, 30, 220))
    draw.text((26, 20), title, fill=(15, 15, 15, 255), font=_font(18))
    draw.text(
        (26, 50),
        f"t={time_s:4.1f} s   reference={reference_speed_mps:4.1f} m/s   speed={speed_mps:4.1f} m/s",
        fill=(15, 15, 15, 255),
        font=_font(15),
    )
    draw.text((26, 72), f"net battery energy={energy_wh:5.1f} Wh", fill=(15, 15, 15, 255), font=_font(15))
    return image


def _save(images: list[Image.Image], output: Path) -> None:
    if not images:
        raise RuntimeError("scenario produced no animation frames")
    images[0].save(
        output,
        save_all=True,
        append_images=images[1:],
        duration=110,
        loop=0,
        optimize=True,
    )


def _run(
    config: ProjectConfig,
    profile: SpeedProfile,
    map_sequence: str,
    spawn_lateral_m: float,
    title: str,
    output: Path,
) -> EpisodeResult:
    powertrain = EVPowertrain(config.hardware, config.vehicle, config.motor, config.battery)
    env = MetaDriveEVEnv(
        powertrain,
        control_interval_s=config.control_interval_s,
        seed=config.seed,
        map_sequence=map_sequence,
        traffic_density=0.0,
        spawn_lateral_m=spawn_lateral_m,
    )
    controller = LongitudinalMPCController(
        powertrain,
        dt_s=config.control_interval_s,
    )
    images: list[Image.Image] = []

    def capture(index: int, active_env: Any) -> None:
        if index % 2 != 0:
            return
        time_s = index * config.control_interval_s
        frame = np.asarray(active_env.render_topdown()).swapaxes(0, 1)
        images.append(
            _annotate(
                frame,
                title,
                time_s,
                profile.reference_at(time_s),
                active_env.speed_mps,
                active_env.energy.net_battery_wh,
            )
        )

    try:
        result = run_speed_profile(
            env,
            profile,
            controller,
            CenterlinePIDController(dt_s=config.control_interval_s),
            step_callback=capture,
        )
    finally:
        env.close()
    _save(images, output)
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=Path("configs/default.yaml"))
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/scenario_gifs"))
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    config = ProjectConfig.from_yaml(args.config)
    urban = _run(
        config,
        URBAN_PROFILE,
        "SSSSSSSSSSSS",
        0.5,
        "Urban stop-go - default MPC",
        args.output_dir / "urban_mpc_topdown.gif",
    )
    curved = _run(
        config,
        CENTERLINE_PROFILE,
        "SCSCSC",
        1.0,
        "Curved route - default MPC",
        args.output_dir / "curved_mpc_topdown.gif",
    )
    report = {"urban": asdict(urban.metrics), "curved": asdict(curved.metrics)}
    (args.output_dir / "scenario_gifs_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
