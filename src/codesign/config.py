"""Validated immutable configuration records used across simulation backends."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


def _positive(name: str, value: float) -> None:
    if value <= 0:
        raise ValueError(f"{name} must be positive, got {value}")


@dataclass(frozen=True, slots=True)
class HardwareDesign:
    """Hardware decision variables."""

    final_drive_ratio: float
    motor_scale: float

    def __post_init__(self) -> None:
        _positive("final_drive_ratio", self.final_drive_ratio)
        _positive("motor_scale", self.motor_scale)


@dataclass(frozen=True, slots=True)
class VehicleConfig:
    base_mass_kg: float = 1500.0
    wheel_radius_m: float = 0.31
    final_drive_efficiency: float = 0.97

    def __post_init__(self) -> None:
        _positive("base_mass_kg", self.base_mass_kg)
        _positive("wheel_radius_m", self.wheel_radius_m)
        if not 0 < self.final_drive_efficiency <= 1:
            raise ValueError("final_drive_efficiency must be in (0, 1]")


@dataclass(frozen=True, slots=True)
class MotorConfig:
    base_peak_torque_nm: float = 300.0
    base_peak_power_kw: float = 150.0
    max_speed_rpm: float = 12_000.0
    base_mass_kg: float = 75.0
    regenerative_torque_fraction: float = 0.60

    def __post_init__(self) -> None:
        for name in (
            "base_peak_torque_nm",
            "base_peak_power_kw",
            "max_speed_rpm",
            "base_mass_kg",
        ):
            _positive(name, getattr(self, name))
        if not 0 <= self.regenerative_torque_fraction <= 1:
            raise ValueError("regenerative_torque_fraction must be in [0, 1]")


@dataclass(frozen=True, slots=True)
class BatteryConfig:
    capacity_kwh: float = 60.0
    inverter_efficiency: float = 0.97
    auxiliary_power_w: float = 500.0

    def __post_init__(self) -> None:
        _positive("capacity_kwh", self.capacity_kwh)
        if not 0 < self.inverter_efficiency <= 1:
            raise ValueError("inverter_efficiency must be in (0, 1]")
        if self.auxiliary_power_w < 0:
            raise ValueError("auxiliary_power_w cannot be negative")


@dataclass(frozen=True, slots=True)
class ProjectConfig:
    hardware: HardwareDesign
    vehicle: VehicleConfig
    motor: MotorConfig
    battery: BatteryConfig
    control_interval_s: float = 0.2
    seed: int = 7
    use_render: bool = False

    def __post_init__(self) -> None:
        _positive("control_interval_s", self.control_interval_s)

    @classmethod
    def from_yaml(cls, path: str | Path) -> "ProjectConfig":
        data: dict[str, Any] = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        simulation = data.get("simulation", {})
        return cls(
            hardware=HardwareDesign(**data["hardware"]),
            vehicle=VehicleConfig(**data.get("vehicle", {})),
            motor=MotorConfig(**data.get("motor", {})),
            battery=BatteryConfig(**data.get("battery", {})),
            control_interval_s=float(simulation.get("control_interval_s", 0.2)),
            seed=int(simulation.get("seed", 7)),
            use_render=bool(simulation.get("use_render", False)),
        )

