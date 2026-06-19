"""Thin MetaDrive adapter with hardware-dependent longitudinal force limiting."""

from __future__ import annotations

from dataclasses import dataclass, field
from math import hypot
from typing import Any

from .powertrain import EVPowertrain, EnergyState, PowertrainStep


class MetaDriveUnavailable(RuntimeError):
    pass


@dataclass(frozen=True, slots=True)
class LeadVehicleState:
    gap_m: float
    speed_mps: float


@dataclass(frozen=True, slots=True)
class LaneState:
    lateral_error_m: float
    heading_error_rad: float
    lane_width_m: float


@dataclass(slots=True)
class MetaDriveEVEnv:
    """Composition wrapper that keeps MetaDrive an optional dependency.

    Actions are `(steering, requested_wheel_force_n)`. The wrapper limits longitudinal force
    through the selected EV powertrain and translates it to MetaDrive's normalized second action.
    """

    powertrain: EVPowertrain
    control_interval_s: float = 0.2
    use_render: bool = False
    seed: int = 7
    map_sequence: str = "S"
    traffic_density: float = 0.0
    spawn_lateral_m: float = 0.0
    _env: Any = field(init=False, repr=False)
    _max_engine_force_n: float = field(init=False, repr=False)
    _max_regen_force_n: float = field(init=False, repr=False)
    energy: EnergyState = field(init=False)
    last_powertrain_step: PowertrainStep | None = field(init=False, default=None)

    def __post_init__(self) -> None:
        try:
            from metadrive.envs import VaryingDynamicsEnv
        except ImportError as exc:  # pragma: no cover - depends on optional package
            raise MetaDriveUnavailable(
                "MetaDrive is not installed; run `pip install -e '.[simulation]'`"
            ) from exc

        self._max_engine_force_n = self._zero_speed_wheel_force()
        self._max_regen_force_n = self._zero_speed_regen_force()
        engine_force_per_wheel = self._max_engine_force_n / 4.0
        brake_torque_per_wheel = (
            self._max_regen_force_n * self.powertrain.vehicle.wheel_radius_m / 4.0
        )
        env_config: dict[str, Any] = {
            "use_render": self.use_render,
            "num_scenarios": 1,
            "start_seed": self.seed,
            "map": self.map_sequence,
            "traffic_density": self.traffic_density,
            "random_traffic": False,
            "random_spawn_lane_index": False,
            "physics_world_step_size": 0.02,
            "decision_repeat": max(1, round(self.control_interval_s / 0.02)),
            "vehicle_config": {
                "vehicle_model": "varying_dynamics",
                # Negative engine force represents regenerative braking. The wrapper prevents
                # reverse motion at zero speed.
                "enable_reverse": True,
                "mass": self.powertrain.total_vehicle_mass_kg,
                "spawn_lateral": self.spawn_lateral_m,
            },
            "random_dynamics": {
                "max_engine_force": (engine_force_per_wheel, engine_force_per_wheel),
                "max_brake_force": (brake_torque_per_wheel, brake_torque_per_wheel),
                "wheel_friction": (1.0, 1.0),
                "max_steering": (40.0, 40.0),
                "mass": (
                    self.powertrain.total_vehicle_mass_kg,
                    self.powertrain.total_vehicle_mass_kg,
                ),
            },
        }
        self._env = VaryingDynamicsEnv(env_config)
        self.energy = EnergyState()
        self.last_powertrain_step: PowertrainStep | None = None

    def _zero_speed_wheel_force(self) -> float:
        return (
            self.powertrain.peak_torque_nm
            * self.powertrain.hardware.final_drive_ratio
            * self.powertrain.vehicle.final_drive_efficiency
            / self.powertrain.vehicle.wheel_radius_m
        )

    def _zero_speed_regen_force(self) -> float:
        return (
            self.powertrain.peak_torque_nm
            * self.powertrain.motor.regenerative_torque_fraction
            * self.powertrain.hardware.final_drive_ratio
            / (
                self.powertrain.vehicle.wheel_radius_m
                * self.powertrain.vehicle.final_drive_efficiency
            )
        )

    @property
    def maximum_traction_force_n(self) -> float:
        return self._max_engine_force_n

    @property
    def maximum_regenerative_force_n(self) -> float:
        return self._max_regen_force_n

    def reset(self) -> tuple[Any, dict[str, Any]]:
        self.energy = EnergyState()
        self.last_powertrain_step = None
        return self._env.reset(seed=self.seed)

    @property
    def speed_mps(self) -> float:
        return float(self._env.agent.speed_km_h) / 3.6

    @property
    def position_xy_m(self) -> tuple[float, float]:
        position = self._env.agent.position
        return float(position[0]), float(position[1])

    def lead_vehicle_state(self, lateral_tolerance_m: float = 2.0) -> LeadVehicleState | None:
        """Return the nearest traffic vehicle ahead in the ego lane corridor."""

        ego = self._env.agent
        nearest: LeadVehicleState | None = None
        for vehicle in self._env.engine.traffic_manager.traffic_vehicles:
            relative = ego.convert_to_local_coordinates(vehicle.position, ego.position)
            longitudinal, lateral = float(relative[0]), float(relative[1])
            if longitudinal <= 0 or abs(lateral) > lateral_tolerance_m:
                continue
            center_distance = hypot(longitudinal, lateral)
            gap = max(0.0, center_distance - (float(ego.LENGTH) + float(vehicle.LENGTH)) / 2.0)
            candidate = LeadVehicleState(gap_m=gap, speed_mps=float(vehicle.speed_km_h) / 3.6)
            if nearest is None or candidate.gap_m < nearest.gap_m:
                nearest = candidate
        return nearest

    def lane_state(self) -> LaneState:
        ego = self._env.agent
        navigation = ego.navigation
        if ego.lane in navigation.current_ref_lanes:
            lane = ego.lane
        else:
            lane = navigation.current_ref_lanes[0]
        longitudinal, lateral = lane.local_coordinates(ego.position)
        lane_heading = float(lane.heading_theta_at(longitudinal + 1.0))
        heading_error = (lane_heading - float(ego.heading_theta) + 3.141592653589793) % (
            2.0 * 3.141592653589793
        ) - 3.141592653589793
        return LaneState(
            lateral_error_m=float(lateral),
            heading_error_rad=heading_error,
            lane_width_m=float(navigation.get_current_lane_width()),
        )

    def render_topdown(self, screen_size: tuple[int, int] = (700, 700)) -> Any:
        return self._env.render(
            mode="topdown",
            target_agent_heading_up=False,
            draw_target_vehicle_trajectory=True,
            film_size=(2000, 2000),
            screen_size=screen_size,
        )

    def step(self, action: tuple[float, float]) -> tuple[Any, float, bool, bool, dict[str, Any]]:
        steering, requested_force_n = action
        speed_mps = self.speed_mps
        powertrain_step = self.powertrain.evaluate(float(requested_force_n), speed_mps)
        self.last_powertrain_step = powertrain_step
        self.energy.update(powertrain_step, self.control_interval_s)

        if powertrain_step.applied_wheel_force_n >= 0:
            longitudinal = powertrain_step.applied_wheel_force_n / self._max_engine_force_n
        else:
            # With reverse enabled, MetaDrive applies a signed engine force. This gives a linear
            # regenerative-force actuator; the EV layer prevents applying it at zero speed.
            longitudinal = max(
                -1.0, powertrain_step.applied_wheel_force_n / self._max_engine_force_n
            )
        longitudinal = max(-1.0, min(1.0, longitudinal))

        observation, reward, terminated, truncated, info = self._env.step(
            [max(-1.0, min(1.0, float(steering))), longitudinal]
        )
        info = dict(info)
        info.update(
            {
                "battery_power_w": powertrain_step.battery_power_w,
                "net_battery_wh": self.energy.net_battery_wh,
                "powertrain_saturated": powertrain_step.saturated,
                "speed_mps": self.speed_mps,
            }
        )
        return observation, reward, terminated, truncated, info

    def close(self) -> None:
        self._env.close()
