"""Thin MetaDrive adapter with hardware-dependent longitudinal force limiting."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from .powertrain import EVPowertrain, EnergyState, PowertrainStep


class MetaDriveUnavailable(RuntimeError):
    pass


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
    _env: Any = field(init=False, repr=False)
    _max_engine_force_n: float = field(init=False, repr=False)
    _max_regen_force_n: float = field(init=False, repr=False)
    energy: EnergyState = field(init=False)
    last_powertrain_step: PowertrainStep | None = field(init=False, default=None)

    def __post_init__(self) -> None:
        try:
            from metadrive.envs import MetaDriveEnv
        except ImportError as exc:  # pragma: no cover - depends on optional package
            raise MetaDriveUnavailable(
                "MetaDrive is not installed; run `pip install -e '.[simulation]'`"
            ) from exc

        self._max_engine_force_n = self._zero_speed_wheel_force()
        self._max_regen_force_n = self._zero_speed_regen_force()
        env_config: dict[str, Any] = {
            "use_render": self.use_render,
            "num_scenarios": 1,
            "start_seed": self.seed,
            "physics_world_step_size": 0.02,
            "decision_repeat": max(1, round(self.control_interval_s / 0.02)),
            "vehicle_config": {
                "mass": self.powertrain.total_vehicle_mass_kg,
            },
        }
        self._env = MetaDriveEnv(env_config)
        # MetaDrive 0.4.3 accepts type-specific dynamics after environment construction.
        # Engine force is applied to four wheels; brake force is wheel torque per wheel.
        self._env.config["vehicle_config"]["max_engine_force"] = self._max_engine_force_n / 4.0
        self._env.config["vehicle_config"]["max_brake_force"] = (
            self._max_regen_force_n * self.powertrain.vehicle.wheel_radius_m / 4.0
        )
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

    def reset(self) -> tuple[Any, dict[str, Any]]:
        self.energy = EnergyState()
        self.last_powertrain_step = None
        return self._env.reset(seed=self.seed)

    def step(self, action: tuple[float, float]) -> tuple[Any, float, bool, bool, dict[str, Any]]:
        steering, requested_force_n = action
        speed_mps = float(self._env.agent.speed_km_h) / 3.6
        powertrain_step = self.powertrain.evaluate(float(requested_force_n), speed_mps)
        self.last_powertrain_step = powertrain_step
        self.energy.update(powertrain_step, self.control_interval_s)

        if powertrain_step.applied_wheel_force_n >= 0:
            longitudinal = powertrain_step.applied_wheel_force_n / self._max_engine_force_n
        else:
            # MetaDrive's negative action is normalized braking, not negative engine force.
            longitudinal = max(
                -1.0, powertrain_step.applied_wheel_force_n / self._max_regen_force_n
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
            }
        )
        return observation, reward, terminated, truncated, info

    def close(self) -> None:
        self._env.close()
