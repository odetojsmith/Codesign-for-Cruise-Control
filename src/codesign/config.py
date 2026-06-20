"""Validated immutable configuration records used across simulation backends."""

from __future__ import annotations

from dataclasses import dataclass, field
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
    maximum_friction_braking_acceleration_mps2: float = 6.0
    rolling_resistance_coefficient: float = 0.012
    drag_coefficient: float = 0.29
    frontal_area_m2: float = 2.3
    air_density_kg_per_m3: float = 1.225

    def __post_init__(self) -> None:
        _positive("base_mass_kg", self.base_mass_kg)
        _positive("wheel_radius_m", self.wheel_radius_m)
        _positive(
            "maximum_friction_braking_acceleration_mps2",
            self.maximum_friction_braking_acceleration_mps2,
        )
        _positive("rolling_resistance_coefficient", self.rolling_resistance_coefficient)
        _positive("drag_coefficient", self.drag_coefficient)
        _positive("frontal_area_m2", self.frontal_area_m2)
        _positive("air_density_kg_per_m3", self.air_density_kg_per_m3)
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
    maximum_discharge_power_kw: float | None = None
    maximum_charge_power_kw: float | None = None

    def __post_init__(self) -> None:
        _positive("capacity_kwh", self.capacity_kwh)
        if not 0 < self.inverter_efficiency <= 1:
            raise ValueError("inverter_efficiency must be in (0, 1]")
        if self.auxiliary_power_w < 0:
            raise ValueError("auxiliary_power_w cannot be negative")
        for name in ("maximum_discharge_power_kw", "maximum_charge_power_kw"):
            value = getattr(self, name)
            if value is not None:
                _positive(name, value)


@dataclass(frozen=True, slots=True)
class MotorThermalConfig:
    """Illustrative lumped motor thermal model used by demanding duty cycles."""

    enabled: bool = False
    ambient_temperature_c: float = 25.0
    initial_temperature_c: float = 25.0
    base_thermal_capacity_j_per_k: float = 35_000.0
    base_thermal_conductance_w_per_k: float = 120.0
    derating_start_temperature_c: float = 90.0
    maximum_temperature_c: float = 120.0
    minimum_torque_fraction: float = 0.25

    def __post_init__(self) -> None:
        _positive("base_thermal_capacity_j_per_k", self.base_thermal_capacity_j_per_k)
        _positive("base_thermal_conductance_w_per_k", self.base_thermal_conductance_w_per_k)
        if self.initial_temperature_c < self.ambient_temperature_c:
            raise ValueError("initial motor temperature cannot be below ambient")
        if self.maximum_temperature_c <= self.derating_start_temperature_c:
            raise ValueError("maximum motor temperature must exceed derating start")
        if not 0 < self.minimum_torque_fraction <= 1:
            raise ValueError("minimum_torque_fraction must be in (0, 1]")


@dataclass(frozen=True, slots=True)
class ProjectConfig:
    hardware: HardwareDesign
    vehicle: VehicleConfig
    motor: MotorConfig
    battery: BatteryConfig
    thermal: MotorThermalConfig = field(default_factory=MotorThermalConfig)
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
            thermal=MotorThermalConfig(**data.get("thermal", {})),
            control_interval_s=float(simulation.get("control_interval_s", 0.2)),
            seed=int(simulation.get("seed", 7)),
            use_render=bool(simulation.get("use_render", False)),
        )
