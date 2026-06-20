"""Generate synchronized top-down low- versus high-RMSE scenario GIFs."""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image, ImageDraw

from .config import ProjectConfig
from .controllers import CenterlinePIDController
from .metadrive_env import MetaDriveEVEnv
from .mpc import LongitudinalMPCController
from .powertrain import EVPowertrain
from .scenario_gifs import _annotate, _font, _save
from .scenarios import CENTERLINE_PROFILE, URBAN_PROFILE, EpisodeResult, SpeedProfile, run_speed_profile
from .trajectory_animation import HIGH_RMSE_WEIGHTS, LOW_RMSE_WEIGHTS


@dataclass(frozen=True, slots=True)
class CapturedRun:
    result: EpisodeResult
    frame_indices: tuple[int, ...]
    frames: tuple[np.ndarray[Any, Any], ...]
    positions_xy_m: tuple[tuple[float, float], ...]
    topdown_scales_px_per_m: tuple[float, ...]


def _run(
    config: ProjectConfig,
    profile: SpeedProfile,
    map_sequence: str,
    spawn_lateral_m: float,
    weights: tuple[float, float],
) -> CapturedRun:
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
        lambda_energy=10.0**weights[0],
        lambda_force_slew=10.0**weights[1],
    )
    frame_indices: list[int] = []
    frames: list[np.ndarray[Any, Any]] = []
    positions_xy_m: list[tuple[float, float]] = []
    topdown_scales_px_per_m: list[float] = []

    def capture(index: int, active_env: Any) -> None:
        if index % 2 != 0:
            return
        frame_indices.append(index)
        frames.append(np.asarray(active_env.render_topdown()).swapaxes(0, 1))
        positions_xy_m.append(active_env.position_xy_m)
        topdown_scales_px_per_m.append(active_env.topdown_scaling_px_per_m)

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
    return CapturedRun(
        result,
        tuple(frame_indices),
        tuple(frames),
        tuple(positions_xy_m),
        tuple(topdown_scales_px_per_m),
    )


def _reference_positions(run: CapturedRun, dt_s: float) -> np.ndarray:
    """Integrate the closed-loop planner reference into route progress."""

    reference_speed = np.asarray(
        [point.reference_speed_mps for point in run.result.trajectory], dtype=float
    )
    return np.cumsum(reference_speed * dt_s)


def _route_position(
    route_distances_m: np.ndarray,
    route_positions_xy_m: np.ndarray,
    distance_m: float,
) -> tuple[tuple[float, float], float]:
    unique_distances, unique_indices = np.unique(route_distances_m, return_index=True)
    unique_positions = route_positions_xy_m[unique_indices]
    clipped_distance = float(np.clip(distance_m, unique_distances[0], unique_distances[-1]))
    x_m = float(np.interp(clipped_distance, unique_distances, unique_positions[:, 0]))
    y_m = float(np.interp(clipped_distance, unique_distances, unique_positions[:, 1]))
    segment = int(np.clip(np.searchsorted(unique_distances, clipped_distance), 1, len(unique_distances) - 1))
    delta = unique_positions[segment] - unique_positions[segment - 1]
    heading_rad = float(np.arctan2(delta[1], delta[0]))
    return (x_m, y_m), heading_rad


def _draw_reference_ghost(
    image: Image.Image,
    ego_position_xy_m: tuple[float, float],
    ghost_position_xy_m: tuple[float, float],
    ghost_heading_rad: float,
    topdown_scale_px_per_m: float,
    actual_position_m: float,
    reference_position_m: float,
) -> None:
    draw = ImageDraw.Draw(image, "RGBA")
    left, right, top, bottom = 12, 430, 101, 130
    draw.rounded_rectangle(
        (left, top, right, bottom),
        radius=8,
        fill=(255, 255, 255, 230),
        outline=(30, 30, 30, 220),
    )
    lag_m = reference_position_m - actual_position_m
    draw.text(
        (left + 14, top + 6),
        f"position: actual={actual_position_m:5.1f} m  reference={reference_position_m:5.1f} m"
        f"  lag={lag_m:+5.1f} m",
        fill=(15, 15, 15, 255),
        font=_font(15),
    )

    # The existing animation pipeline swaps the renderer's image axes before saving.
    center_x = image.width / 2 - topdown_scale_px_per_m * (
        ghost_position_xy_m[1] - ego_position_xy_m[1]
    )
    center_y = image.height / 2 + topdown_scale_px_per_m * (
        ghost_position_xy_m[0] - ego_position_xy_m[0]
    )
    forward = np.asarray((-np.sin(ghost_heading_rad), np.cos(ghost_heading_rad)))
    leftward = np.asarray((-forward[1], forward[0]))
    center = np.asarray((center_x, center_y))
    half_length_px, half_width_px = 14.0, 7.0
    polygon = [
        tuple(center + half_length_px * forward + half_width_px * leftward),
        tuple(center + half_length_px * forward - half_width_px * leftward),
        tuple(center - half_length_px * forward - half_width_px * leftward),
        tuple(center - half_length_px * forward + half_width_px * leftward),
    ]
    draw.polygon(polygon, fill=(220, 35, 135, 190), outline=(70, 0, 40, 255))
    draw.line(
        (image.width / 2, image.height / 2, center_x, center_y),
        fill=(220, 35, 135, 115),
        width=2,
    )
    label_x = int(np.clip(center_x + 12, 4, image.width - 105))
    label_y = int(np.clip(center_y - 25, 135, image.height - 24))
    draw.rounded_rectangle(
        (label_x, label_y, label_x + 99, label_y + 21),
        radius=5,
        fill=(255, 245, 251, 225),
        outline=(180, 20, 105, 220),
    )
    draw.text((label_x + 7, label_y + 3), "REFERENCE", fill=(160, 10, 90, 255), font=_font(13))


def _annotated_frames(
    run: CapturedRun,
    title: str,
    dt_s: float,
    route_distances_m: np.ndarray,
    route_positions_xy_m: np.ndarray,
) -> list[Image.Image]:
    images: list[Image.Image] = []
    reference_positions = _reference_positions(run, dt_s)
    for capture_index, (index, frame) in enumerate(
        zip(run.frame_indices, run.frames, strict=True)
    ):
        point_index = min(index, len(run.result.trajectory) - 1)
        point = run.result.trajectory[point_index]
        image = _annotate(
            frame,
            title,
            point.time_s,
            point.reference_speed_mps,
            point.speed_mps,
            point.cumulative_battery_wh,
        )
        ghost_position, ghost_heading = _route_position(
            route_distances_m,
            route_positions_xy_m,
            reference_positions[point_index],
        )
        _draw_reference_ghost(
            image,
            run.positions_xy_m[capture_index],
            ghost_position,
            ghost_heading,
            run.topdown_scales_px_per_m[capture_index],
            point.distance_m,
            reference_positions[point_index],
        )
        images.append(image)
    return images


def _combine(left: list[Image.Image], right: list[Image.Image]) -> list[Image.Image]:
    count = min(len(left), len(right))
    images: list[Image.Image] = []
    for left_frame, right_frame in zip(left[:count], right[:count], strict=True):
        width = left_frame.width + right_frame.width
        height = max(left_frame.height, right_frame.height)
        canvas = Image.new("RGB", (width, height), "white")
        canvas.paste(left_frame.convert("RGB"), (0, 0))
        canvas.paste(right_frame.convert("RGB"), (left_frame.width, 0))
        draw = ImageDraw.Draw(canvas)
        draw.line((left_frame.width, 0, left_frame.width, height), fill=(20, 20, 20), width=4)
        images.append(canvas)
    return images


def _scenario(
    config: ProjectConfig,
    profile: SpeedProfile,
    map_sequence: str,
    spawn_lateral_m: float,
    scenario_name: str,
    output: Path,
) -> dict[str, object]:
    low = _run(config, profile, map_sequence, spawn_lateral_m, LOW_RMSE_WEIGHTS)
    high = _run(config, profile, map_sequence, spawn_lateral_m, HIGH_RMSE_WEIGHTS)
    low_title = f"{scenario_name} - LOW RMSE  weights=(-1,-1)"
    high_title = f"{scenario_name} - HIGH RMSE  weights=(1.5,1.5)"
    route_distances_m = np.asarray(
        [low.result.trajectory[index].distance_m for index in low.frame_indices]
    )
    route_positions_xy_m = np.asarray(low.positions_xy_m)
    _save(
        _combine(
            _annotated_frames(
                low,
                low_title,
                config.control_interval_s,
                route_distances_m,
                route_positions_xy_m,
            ),
            _annotated_frames(
                high,
                high_title,
                config.control_interval_s,
                route_distances_m,
                route_positions_xy_m,
            ),
        ),
        output,
    )
    return {
        "low_rmse": {"log10_weights": LOW_RMSE_WEIGHTS, "metrics": asdict(low.result.metrics)},
        "high_rmse": {
            "log10_weights": HIGH_RMSE_WEIGHTS,
            "metrics": asdict(high.result.metrics),
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=Path("configs/default.yaml"))
    parser.add_argument(
        "--output-dir", type=Path, default=Path("artifacts/scenario_comparison_gifs")
    )
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    config = ProjectConfig.from_yaml(args.config)
    report = {
        "urban": _scenario(
            config,
            URBAN_PROFILE,
            "SSSSSSSSSSSS",
            0.5,
            "Urban",
            args.output_dir / "urban_low_high_mpc.gif",
        ),
        "curved": _scenario(
            config,
            CENTERLINE_PROFILE,
            "SCSCSC",
            1.0,
            "Curved",
            args.output_dir / "curved_low_high_mpc.gif",
        ),
    }
    (args.output_dir / "scenario_comparison_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
