"""Thin MetaDrive adapter with hardware-dependent longitudinal force limiting."""

from __future__ import annotations

from dataclasses import dataclass, field
from math import atan, hypot, pi, sin
from typing import Any, Callable

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
    curvature_per_m: float = 0.0


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
    terminate_on_out_of_road: bool = True
    road_grade_profile: Callable[[float], float] | None = None
    maximum_abs_grade_fraction: float = 0.0
    _env: Any = field(init=False, repr=False)
    _max_traction_force_n: float = field(init=False, repr=False)
    _metadrive_force_scale_n: float = field(init=False, repr=False)
    _max_regen_force_n: float = field(init=False, repr=False)
    energy: EnergyState = field(init=False)
    last_powertrain_step: PowertrainStep | None = field(init=False, default=None)
    last_net_chassis_force_n: float = field(init=False, default=0.0)
    _route_distance_m: float = field(init=False, default=0.0, repr=False)
    _previous_position_xy_m: tuple[float, float] = field(
        init=False, default=(0.0, 0.0), repr=False
    )

    def __post_init__(self) -> None:
        try:
            from metadrive.envs import VaryingDynamicsEnv
        except ImportError as exc:  # pragma: no cover - depends on optional package
            raise MetaDriveUnavailable(
                "MetaDrive is not installed; run `pip install -e '.[simulation]'`"
            ) from exc

        self._max_traction_force_n = self._zero_speed_wheel_force()
        self._max_regen_force_n = self._zero_speed_regen_force()
        maximum_braking_force_n = (
            self.powertrain.total_vehicle_mass_kg
            * self.powertrain.vehicle.maximum_friction_braking_acceleration_mps2
        )
        maximum_grade_force_n = self.powertrain.total_vehicle_mass_kg * 9.80665 * abs(
            self.maximum_abs_grade_fraction
        )
        self._metadrive_force_scale_n = max(
            self._max_traction_force_n + maximum_grade_force_n,
            maximum_braking_force_n + maximum_grade_force_n,
        )
        engine_force_per_wheel = self._metadrive_force_scale_n / 4.0
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
            "out_of_road_done": self.terminate_on_out_of_road,
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
        return self._max_traction_force_n

    @property
    def maximum_regenerative_force_n(self) -> float:
        return self._max_regen_force_n

    def reset(self) -> tuple[Any, dict[str, Any]]:
        self.powertrain.reset_state()
        self.energy = EnergyState()
        self.last_powertrain_step = None
        self.last_net_chassis_force_n = 0.0
        self._route_distance_m = 0.0
        output = self._env.reset(seed=self.seed)
        self._previous_position_xy_m = self.position_xy_m
        return output

    @property
    def speed_mps(self) -> float:
        return float(self._env.agent.speed_km_h) / 3.6

    @property
    def position_xy_m(self) -> tuple[float, float]:
        position = self._env.agent.position
        return float(position[0]), float(position[1])

    @property
    def heading_rad(self) -> float:
        """Chassis heading reported by MetaDrive, wrapped to [-pi, pi]."""

        return float(self._env.agent.heading_theta)

    @property
    def applied_steering_command(self) -> float:
        """Normalized steering state stored by MetaDrive after action processing."""

        return float(self._env.agent.steering)

    @property
    def applied_steering_angle_deg(self) -> float:
        """Front-wheel angle sent to Bullet by MetaDrive."""

        agent = self._env.agent
        return float(agent.steering) * float(agent.max_steering)

    @property
    def wheelbase_m(self) -> float:
        agent = self._env.agent
        return float(agent.FRONT_WHEELBASE + agent.REAR_WHEELBASE)

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
            curvature_per_m=self.road_curvature_preview((0.0,))[0],
        )

    def road_curvature_preview(self, distances_m: tuple[float, ...]) -> tuple[float, ...]:
        """Estimate centerline curvature along the current and next route lanes."""

        if any(distance < 0 for distance in distances_m):
            raise ValueError("preview distances must be nonnegative")
        ego = self._env.agent
        navigation = ego.navigation
        current = ego.lane if ego.lane in navigation.current_ref_lanes else navigation.current_ref_lanes[0]
        current_longitudinal, _ = current.local_coordinates(ego.position)
        lanes = [current]
        if navigation.next_ref_lanes:
            lane_number = current.index[-1] if current.index is not None else 0
            matching = [lane for lane in navigation.next_ref_lanes if lane.index[-1] == lane_number]
            lanes.append(matching[0] if matching else navigation.next_ref_lanes[0])

        def heading_at(distance_m: float) -> float:
            remaining = distance_m + max(0.0, current_longitudinal)
            for lane in lanes:
                if remaining <= lane.length:
                    return float(lane.heading_theta_at(max(0.0, remaining)))
                remaining -= lane.length
            return float(lanes[-1].heading_theta_at(lanes[-1].length))

        window_m = 2.0
        values = []
        for distance in distances_m:
            heading_0 = heading_at(distance)
            heading_1 = heading_at(distance + window_m)
            change = (heading_1 - heading_0 + pi) % (2.0 * pi) - pi
            values.append(change / window_m)
        return tuple(values)

    def road_grade_preview(self, distances_m: tuple[float, ...]) -> tuple[float, ...]:
        if any(distance < 0 for distance in distances_m):
            raise ValueError("preview distances must be nonnegative")
        if self.road_grade_profile is None:
            return tuple(0.0 for _ in distances_m)
        return tuple(
            float(self.road_grade_profile(self._route_distance_m + distance))
            for distance in distances_m
        )

    def render_topdown(self, screen_size: tuple[int, int] = (700, 700)) -> Any:
        return self._env.render(
            mode="topdown",
            target_agent_heading_up=False,
            draw_target_vehicle_trajectory=True,
            film_size=(2000, 2000),
            screen_size=screen_size,
        )

    @property
    def topdown_scaling_px_per_m(self) -> float:
        """Return the active top-down renderer scale after the first rendered frame."""

        renderer = self._env.top_down_renderer
        if renderer is None:
            raise RuntimeError("top-down renderer has not been initialized")
        return float(renderer.scaling)

    def step(self, action: tuple[float, float]) -> tuple[Any, float, bool, bool, dict[str, Any]]:
        steering, requested_force_n = action
        speed_mps = self.speed_mps
        grade_fraction = self.road_grade_preview((0.0,))[0]
        grade_force_n = (
            self.powertrain.total_vehicle_mass_kg
            * 9.80665
            * sin(atan(grade_fraction))
        )
        powertrain_step = self.powertrain.evaluate(float(requested_force_n), speed_mps)
        self.last_powertrain_step = powertrain_step
        self.energy.update(powertrain_step, self.control_interval_s)
        self.powertrain.update_thermal(powertrain_step, self.control_interval_s)

        net_chassis_force_n = powertrain_step.applied_wheel_force_n - grade_force_n
        self.last_net_chassis_force_n = net_chassis_force_n

        if net_chassis_force_n >= 0:
            longitudinal = (
                net_chassis_force_n / self._metadrive_force_scale_n
            )
        else:
            # With reverse enabled, MetaDrive applies a signed engine force. This gives a linear
            # regenerative-force actuator; the EV layer prevents applying it at zero speed.
            longitudinal = max(
                -1.0,
                net_chassis_force_n / self._metadrive_force_scale_n,
            )
        longitudinal = max(-1.0, min(1.0, longitudinal))

        observation, reward, terminated, truncated, info = self._env.step(
            [max(-1.0, min(1.0, float(steering))), longitudinal]
        )
        position = self.position_xy_m
        self._route_distance_m += hypot(
            position[0] - self._previous_position_xy_m[0],
            position[1] - self._previous_position_xy_m[1],
        )
        self._previous_position_xy_m = position
        info = dict(info)
        info.update(
            {
                "battery_power_w": powertrain_step.battery_power_w,
                "net_battery_wh": self.energy.net_battery_wh,
                "powertrain_saturated": powertrain_step.saturated,
                "speed_mps": self.speed_mps,
                "road_grade_fraction": grade_fraction,
                "grade_resistance_force_n": grade_force_n,
                "motor_temperature_c": self.powertrain.thermal_state.temperature_c,
                "thermal_derating_factor": self.powertrain.thermal_derating_factor,
            }
        )
        return observation, reward, terminated, truncated, info

    def close(self) -> None:
        self._env.close()
