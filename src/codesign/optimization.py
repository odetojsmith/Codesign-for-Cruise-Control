"""Shared deterministic closed-loop evaluator with persistent SQLite caching."""

from __future__ import annotations

import hashlib
import json
import sqlite3
from dataclasses import asdict, dataclass, replace
from math import sqrt
from pathlib import Path
from typing import Any

from .config import HardwareDesign, ProjectConfig
from .controllers import CenterlinePIDController
from .metadrive_env import MetaDriveEVEnv
from .mpc import LongitudinalMPCController
from .powertrain import EVPowertrain
from .scenarios import (
    HIGHWAY_TRAINING_PROFILE,
    MIXED_GRADE_PROFILE,
    MIXED_GRADE_SPEED_PROFILE,
    URBAN_PROFILE,
    EpisodeResult,
    RoadGradeProfile,
    SpeedProfile,
    run_speed_profile,
)


@dataclass(frozen=True, slots=True)
class ControllerDesign:
    log10_lambda_energy: float
    log10_lambda_force_slew: float


@dataclass(frozen=True, slots=True)
class ScenarioSpec:
    name: str
    profile: SpeedProfile
    map_sequence: str
    spawn_lateral_m: float = 0.0
    grade_profile: RoadGradeProfile | None = None

    def key_data(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "profile": asdict(self.profile),
            "map_sequence": self.map_sequence,
            "spawn_lateral_m": self.spawn_lateral_m,
            "grade_profile": None if self.grade_profile is None else asdict(self.grade_profile),
        }


@dataclass(frozen=True, slots=True)
class EvaluationSummary:
    key: str
    hardware: HardwareDesign
    controller: ControllerDesign
    aggregate_rmse_mps: float
    total_net_battery_wh: float
    aggregate_wh_per_km: float
    total_distance_m: float
    peak_acceleration_mps2: float
    peak_jerk_mps3: float
    maximum_lateral_error_m: float
    fallback_count: int
    maximum_safety_slack_m: float
    completed: bool
    base_feasible: bool
    violations: tuple[str, ...]
    scenario_metrics: dict[str, dict[str, float | bool]]
    from_cache: bool = False

    @classmethod
    def from_dict(cls, data: dict[str, Any], from_cache: bool = False) -> "EvaluationSummary":
        values = dict(data)
        values["hardware"] = HardwareDesign(**values["hardware"])
        values["controller"] = ControllerDesign(**values["controller"])
        values["violations"] = tuple(values["violations"])
        values["from_cache"] = from_cache
        return cls(**values)


def default_training_scenarios() -> tuple[ScenarioSpec, ...]:
    return (
        ScenarioSpec("urban", URBAN_PROFILE, "SSSSSSSSSSSS", 0.5),
        ScenarioSpec("highway", HIGHWAY_TRAINING_PROFILE, "S" * 12, 0.5),
        ScenarioSpec(
            "mixed_grade",
            MIXED_GRADE_SPEED_PROFILE,
            "S" * 12,
            0.5,
            MIXED_GRADE_PROFILE,
        ),
    )


class EvaluationCache:
    def __init__(self, path: str | Path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with self._connect() as connection:
            connection.execute("PRAGMA journal_mode=WAL")
            connection.execute(
                """
                CREATE TABLE IF NOT EXISTS evaluations (
                    key TEXT PRIMARY KEY,
                    request_json TEXT NOT NULL,
                    result_json TEXT NOT NULL,
                    created_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP
                )
                """
            )

    def _connect(self) -> sqlite3.Connection:
        return sqlite3.connect(self.path, timeout=30.0)

    def get(self, key: str) -> EvaluationSummary | None:
        with self._connect() as connection:
            row = connection.execute(
                "SELECT result_json FROM evaluations WHERE key = ?", (key,)
            ).fetchone()
        if row is None:
            return None
        return EvaluationSummary.from_dict(json.loads(row[0]), from_cache=True)

    def put(self, key: str, request: dict[str, Any], result: EvaluationSummary) -> None:
        stored = replace(result, from_cache=False)
        with self._connect() as connection:
            connection.execute(
                "INSERT OR REPLACE INTO evaluations(key, request_json, result_json) VALUES (?, ?, ?)",
                (
                    key,
                    json.dumps(request, sort_keys=True),
                    json.dumps(asdict(stored), sort_keys=True),
                ),
            )

    def count(self) -> int:
        with self._connect() as connection:
            return int(connection.execute("SELECT COUNT(*) FROM evaluations").fetchone()[0])


@dataclass(slots=True)
class ClosedLoopEvaluator:
    config: ProjectConfig
    cache: EvaluationCache
    scenarios: tuple[ScenarioSpec, ...] = default_training_scenarios()
    maximum_acceleration_mps2: float = 3.05
    maximum_jerk_mps3: float = 4.0
    maximum_lateral_error_m: float = 1.75

    def _request(
        self, hardware: HardwareDesign, controller: ControllerDesign
    ) -> dict[str, Any]:
        return {
            "schema_version": 1,
            "hardware": asdict(hardware),
            "controller": asdict(controller),
            "vehicle": asdict(self.config.vehicle),
            "motor": asdict(self.config.motor),
            "battery": asdict(self.config.battery),
            "control_interval_s": self.config.control_interval_s,
            "seed": self.config.seed,
            "scenarios": [scenario.key_data() for scenario in self.scenarios],
            "constraints": {
                "maximum_acceleration_mps2": self.maximum_acceleration_mps2,
                "maximum_jerk_mps3": self.maximum_jerk_mps3,
                "maximum_lateral_error_m": self.maximum_lateral_error_m,
            },
        }

    @staticmethod
    def _key(request: dict[str, Any]) -> str:
        payload = json.dumps(request, sort_keys=True, separators=(",", ":"))
        return hashlib.sha256(payload.encode("utf-8")).hexdigest()

    def _run_scenario(
        self,
        scenario: ScenarioSpec,
        hardware: HardwareDesign,
        controller_design: ControllerDesign,
    ) -> tuple[EpisodeResult, int, float]:
        run_config = replace(self.config, hardware=hardware)
        powertrain = EVPowertrain(
            run_config.hardware,
            run_config.vehicle,
            run_config.motor,
            run_config.battery,
        )
        grade_query = None if scenario.grade_profile is None else scenario.grade_profile.grade_at
        maximum_grade = (
            0.0
            if scenario.grade_profile is None
            else max(abs(value) for value in scenario.grade_profile.grade_fraction)
        )
        env = MetaDriveEVEnv(
            powertrain,
            control_interval_s=run_config.control_interval_s,
            seed=run_config.seed,
            map_sequence=scenario.map_sequence,
            traffic_density=0.0,
            spawn_lateral_m=scenario.spawn_lateral_m,
            road_grade_profile=grade_query,
            maximum_abs_grade_fraction=maximum_grade,
        )
        controller = LongitudinalMPCController(
            powertrain,
            dt_s=run_config.control_interval_s,
            lambda_energy=10.0**controller_design.log10_lambda_energy,
            lambda_force_slew=10.0**controller_design.log10_lambda_force_slew,
        )
        try:
            result = run_speed_profile(
                env,
                scenario.profile,
                controller,
                CenterlinePIDController(dt_s=run_config.control_interval_s),
            )
        finally:
            env.close()
        return result, controller.fallback_count, controller.maximum_safety_slack_m

    def evaluate(
        self,
        hardware: HardwareDesign,
        controller: ControllerDesign,
    ) -> EvaluationSummary:
        request = self._request(hardware, controller)
        key = self._key(request)
        cached = self.cache.get(key)
        if cached is not None:
            return cached
        runs = [self._run_scenario(scenario, hardware, controller) for scenario in self.scenarios]
        points = [point for result, _, _ in runs for point in result.trajectory]
        aggregate_rmse = sqrt(
            sum((point.speed_mps - point.reference_speed_mps) ** 2 for point in points)
            / len(points)
        )
        total_energy = sum(result.metrics.net_battery_wh for result, _, _ in runs)
        total_distance = sum(result.metrics.distance_m for result, _, _ in runs)
        peak_acceleration = max(result.metrics.peak_acceleration_mps2 for result, _, _ in runs)
        peak_jerk = max(result.metrics.peak_jerk_mps3 for result, _, _ in runs)
        lateral_error = max(
            result.metrics.maximum_abs_lateral_error_m for result, _, _ in runs
        )
        fallback_count = sum(fallbacks for _, fallbacks, _ in runs)
        safety_slack = max(slack for _, _, slack in runs)
        completed = all(result.metrics.completed for result, _, _ in runs)
        violations: list[str] = []
        if not completed:
            violations.append("incomplete_episode")
        if peak_acceleration > self.maximum_acceleration_mps2:
            violations.append("acceleration")
        if peak_jerk > self.maximum_jerk_mps3:
            violations.append("jerk")
        if lateral_error >= self.maximum_lateral_error_m:
            violations.append("lateral_error")
        if fallback_count:
            violations.append("mpc_fallback")
        scenario_metrics = {
            scenario.name: {
                "rmse_mps": result.metrics.rmse_mps,
                "net_battery_wh": result.metrics.net_battery_wh,
                "distance_m": result.metrics.distance_m,
                "completed": result.metrics.completed,
            }
            for scenario, (result, _, _) in zip(self.scenarios, runs)
        }
        summary = EvaluationSummary(
            key=key,
            hardware=hardware,
            controller=controller,
            aggregate_rmse_mps=aggregate_rmse,
            total_net_battery_wh=total_energy,
            aggregate_wh_per_km=total_energy / (total_distance / 1000.0),
            total_distance_m=total_distance,
            peak_acceleration_mps2=peak_acceleration,
            peak_jerk_mps3=peak_jerk,
            maximum_lateral_error_m=lateral_error,
            fallback_count=fallback_count,
            maximum_safety_slack_m=safety_slack,
            completed=completed,
            base_feasible=not violations,
            violations=tuple(violations),
            scenario_metrics=scenario_metrics,
        )
        self.cache.put(key, request, summary)
        return summary
