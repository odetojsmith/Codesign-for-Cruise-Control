"""Hardware-dependent EV force limits and electrical-energy accounting."""

from __future__ import annotations

from dataclasses import dataclass, field
from math import pi

from .config import BatteryConfig, HardwareDesign, MotorConfig, VehicleConfig
from .efficiency import EfficiencyMap, default_motoring_map


@dataclass(frozen=True, slots=True)
class PowertrainStep:
    requested_wheel_force_n: float
    applied_wheel_force_n: float
    motor_speed_rad_s: float
    motor_torque_nm: float
    motor_efficiency: float
    battery_power_w: float
    traction_power_w: float
    auxiliary_power_w: float
    saturated: bool
    speed_limited: bool


@dataclass(slots=True)
class EnergyState:
    """Episode-level energy totals. Positive regeneration is recovered energy."""

    gross_traction_wh: float = 0.0
    regenerated_wh: float = 0.0
    auxiliary_wh: float = 0.0
    net_battery_wh: float = 0.0

    def update(self, step: PowertrainStep, dt_s: float) -> None:
        if dt_s <= 0:
            raise ValueError("dt_s must be positive")
        self.gross_traction_wh += max(step.traction_power_w, 0.0) * dt_s / 3600.0
        self.regenerated_wh += max(-step.traction_power_w, 0.0) * dt_s / 3600.0
        self.auxiliary_wh += step.auxiliary_power_w * dt_s / 3600.0
        self.net_battery_wh += step.battery_power_w * dt_s / 3600.0

    def state_of_charge(self, battery: BatteryConfig, initial_soc: float = 1.0) -> float:
        if not 0 <= initial_soc <= 1:
            raise ValueError("initial_soc must be in [0, 1]")
        return initial_soc - self.net_battery_wh / (battery.capacity_kwh * 1000.0)


@dataclass(slots=True)
class EVPowertrain:
    hardware: HardwareDesign
    vehicle: VehicleConfig = field(default_factory=VehicleConfig)
    motor: MotorConfig = field(default_factory=MotorConfig)
    battery: BatteryConfig = field(default_factory=BatteryConfig)
    efficiency_map: EfficiencyMap = field(default_factory=default_motoring_map)

    @property
    def peak_torque_nm(self) -> float:
        return self.motor.base_peak_torque_nm * self.hardware.motor_scale

    @property
    def peak_power_w(self) -> float:
        return self.motor.base_peak_power_kw * 1000.0 * self.hardware.motor_scale

    @property
    def max_speed_rad_s(self) -> float:
        return self.motor.max_speed_rpm * 2.0 * pi / 60.0

    @property
    def motor_mass_kg(self) -> float:
        return self.motor.base_mass_kg * self.hardware.motor_scale

    @property
    def total_vehicle_mass_kg(self) -> float:
        return self.vehicle.base_mass_kg + self.motor_mass_kg

    def motor_speed(self, vehicle_speed_mps: float) -> float:
        return abs(vehicle_speed_mps) * self.hardware.final_drive_ratio / self.vehicle.wheel_radius_m

    def torque_limit(self, motor_speed_rad_s: float, regenerating: bool = False) -> float:
        if motor_speed_rad_s > self.max_speed_rad_s:
            return 0.0
        power_limited = self.peak_power_w / max(motor_speed_rad_s, 1e-9)
        limit = min(self.peak_torque_nm, power_limited)
        if regenerating:
            limit *= self.motor.regenerative_torque_fraction
        return limit

    def evaluate(self, requested_wheel_force_n: float, vehicle_speed_mps: float) -> PowertrainStep:
        """Limit a requested wheel force and evaluate instantaneous battery power."""

        if requested_wheel_force_n < 0 and vehicle_speed_mps <= 0.1:
            requested_for_limits_n = 0.0
        else:
            requested_for_limits_n = requested_wheel_force_n
        omega = self.motor_speed(vehicle_speed_mps)
        regenerating = requested_for_limits_n < 0 and vehicle_speed_mps > 0
        eta_g = self.vehicle.final_drive_efficiency
        ratio = self.hardware.final_drive_ratio
        radius = self.vehicle.wheel_radius_m

        if regenerating:
            requested_torque = requested_for_limits_n * radius * eta_g / ratio
        else:
            requested_torque = requested_for_limits_n * radius / (eta_g * ratio)

        limit = self.torque_limit(omega, regenerating=regenerating)
        applied_torque = max(-limit, min(limit, requested_torque))
        speed_limited = omega > self.max_speed_rad_s
        saturated = speed_limited or abs(applied_torque - requested_torque) > 1e-9

        if applied_torque < 0:
            applied_force = applied_torque * ratio / (radius * eta_g)
        else:
            applied_force = applied_torque * ratio * eta_g / radius

        normalized_speed = omega / self.max_speed_rad_s
        normalized_torque = abs(applied_torque) / max(self.peak_torque_nm, 1e-9)
        efficiency = self.efficiency_map.interpolate(normalized_speed, normalized_torque)
        mechanical_power = applied_torque * omega

        if mechanical_power >= 0:
            traction_power = mechanical_power / (efficiency * self.battery.inverter_efficiency)
        else:
            traction_power = mechanical_power * efficiency * self.battery.inverter_efficiency
        battery_power = traction_power + self.battery.auxiliary_power_w

        return PowertrainStep(
            requested_wheel_force_n=requested_wheel_force_n,
            applied_wheel_force_n=applied_force,
            motor_speed_rad_s=omega,
            motor_torque_nm=applied_torque,
            motor_efficiency=efficiency,
            battery_power_w=battery_power,
            traction_power_w=traction_power,
            auxiliary_power_w=self.battery.auxiliary_power_w,
            saturated=saturated,
            speed_limited=speed_limited,
        )
