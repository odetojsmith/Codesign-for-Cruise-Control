"""Hardware-dependent EV force limits and electrical-energy accounting."""

from __future__ import annotations

from dataclasses import dataclass, field
from math import atan, cos, pi, sin

from .config import (
    BatteryConfig,
    HardwareDesign,
    MotorConfig,
    MotorThermalConfig,
    VehicleConfig,
)
from .efficiency import EfficiencyMap, default_motoring_map


@dataclass(frozen=True, slots=True)
class PowertrainStep:
    requested_wheel_force_n: float
    applied_wheel_force_n: float
    vehicle_speed_mps: float
    motor_speed_rad_s: float
    motor_torque_nm: float
    motor_efficiency: float
    battery_power_w: float
    traction_power_w: float
    auxiliary_power_w: float
    saturated: bool
    speed_limited: bool
    regenerative_wheel_force_n: float = 0.0
    friction_brake_force_n: float = 0.0
    motor_temperature_c: float = 25.0
    thermal_derating_factor: float = 1.0
    battery_power_limited: bool = False


@dataclass(slots=True)
class MotorThermalState:
    temperature_c: float
    peak_temperature_c: float
    accumulated_loss_kj: float = 0.0


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
    thermal: MotorThermalConfig = field(default_factory=MotorThermalConfig)
    efficiency_map: EfficiencyMap = field(default_factory=default_motoring_map)
    thermal_state: MotorThermalState = field(init=False)

    def __post_init__(self) -> None:
        self.reset_state()

    def reset_state(self) -> None:
        initial = self.thermal.initial_temperature_c
        self.thermal_state = MotorThermalState(initial, initial)

    @property
    def thermal_derating_factor(self) -> float:
        if not self.thermal.enabled:
            return 1.0
        temperature = self.thermal_state.temperature_c
        if temperature <= self.thermal.derating_start_temperature_c:
            return 1.0
        span = self.thermal.maximum_temperature_c - self.thermal.derating_start_temperature_c
        progress = min(1.0, (temperature - self.thermal.derating_start_temperature_c) / span)
        return 1.0 - progress * (1.0 - self.thermal.minimum_torque_fraction)

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

    def road_load_force(self, vehicle_speed_mps: float, grade_fraction: float = 0.0) -> float:
        """Return rolling, aerodynamic, and signed grade resistance at constant speed."""

        speed = abs(vehicle_speed_mps)
        angle = atan(grade_fraction)
        rolling = (
            self.total_vehicle_mass_kg
            * 9.80665
            * self.vehicle.rolling_resistance_coefficient
            * cos(angle)
        )
        aerodynamic = (
            0.5
            * self.vehicle.air_density_kg_per_m3
            * self.vehicle.drag_coefficient
            * self.vehicle.frontal_area_m2
            * speed**2
        )
        grade = self.total_vehicle_mass_kg * 9.80665 * sin(angle)
        return rolling + aerodynamic + grade

    def motor_speed(self, vehicle_speed_mps: float) -> float:
        return abs(vehicle_speed_mps) * self.hardware.final_drive_ratio / self.vehicle.wheel_radius_m

    def torque_limit(self, motor_speed_rad_s: float, regenerating: bool = False) -> float:
        if motor_speed_rad_s > self.max_speed_rad_s:
            return 0.0
        power_limited = self.peak_power_w / max(motor_speed_rad_s, 1e-9)
        limit = min(self.peak_torque_nm, power_limited) * self.thermal_derating_factor
        if regenerating:
            limit *= self.motor.regenerative_torque_fraction
        return limit

    def force_limits(self, vehicle_speed_mps: float) -> tuple[float, float]:
        """Return total braking and traction wheel-force limits at the given speed."""

        omega = self.motor_speed(vehicle_speed_mps)
        eta_g = self.vehicle.final_drive_efficiency
        ratio = self.hardware.final_drive_ratio
        radius = self.vehicle.wheel_radius_m
        traction = self.torque_limit(omega) * ratio * eta_g / radius
        braking = (
            self.total_vehicle_mass_kg
            * self.vehicle.maximum_friction_braking_acceleration_mps2
        )
        return -braking, traction

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

        speed_limited = omega > self.max_speed_rad_s
        friction_force = 0.0
        regenerative_force = 0.0
        battery_power_limited = False
        if regenerating:
            total_braking_limit = (
                self.total_vehicle_mass_kg
                * self.vehicle.maximum_friction_braking_acceleration_mps2
            )
            applied_force = max(-total_braking_limit, requested_for_limits_n)
            regenerative_torque_limit = self.torque_limit(omega, regenerating=True)
            regenerative_force_limit = regenerative_torque_limit * ratio / (radius * eta_g)
            regenerative_force = max(applied_force, -regenerative_force_limit)
            applied_torque = regenerative_force * radius * eta_g / ratio
            if self.battery.maximum_charge_power_kw is not None and omega > 1e-9:
                for _ in range(4):
                    normalized_speed = omega / self.max_speed_rad_s
                    normalized_torque = abs(applied_torque) / max(self.peak_torque_nm, 1e-9)
                    efficiency = self.efficiency_map.interpolate(
                        normalized_speed, normalized_torque
                    )
                    mechanical_power = applied_torque * omega
                    battery_power = (
                        mechanical_power * efficiency * self.battery.inverter_efficiency
                        + self.battery.auxiliary_power_w
                    )
                    charge_limit_w = self.battery.maximum_charge_power_kw * 1000.0
                    if battery_power >= -charge_limit_w - 1e-9:
                        break
                    allowed_mechanical_w = (
                        charge_limit_w + self.battery.auxiliary_power_w
                    ) / (efficiency * self.battery.inverter_efficiency)
                    applied_torque = -min(abs(applied_torque), allowed_mechanical_w / omega)
                    battery_power_limited = True
                regenerative_force = applied_torque * ratio / (radius * eta_g)
            friction_force = applied_force - regenerative_force
            saturated = abs(applied_force - requested_for_limits_n) > 1e-9
        else:
            requested_torque = requested_for_limits_n * radius / (eta_g * ratio)
            limit = self.torque_limit(omega)
            applied_torque = max(0.0, min(limit, requested_torque))
            if self.battery.maximum_discharge_power_kw is not None and omega > 1e-9:
                for _ in range(4):
                    normalized_speed = omega / self.max_speed_rad_s
                    normalized_torque = abs(applied_torque) / max(self.peak_torque_nm, 1e-9)
                    efficiency = self.efficiency_map.interpolate(
                        normalized_speed, normalized_torque
                    )
                    mechanical_power = applied_torque * omega
                    battery_power = (
                        mechanical_power / (efficiency * self.battery.inverter_efficiency)
                        + self.battery.auxiliary_power_w
                    )
                    discharge_limit_w = self.battery.maximum_discharge_power_kw * 1000.0
                    if battery_power <= discharge_limit_w + 1e-9:
                        break
                    available_w = max(0.0, discharge_limit_w - self.battery.auxiliary_power_w)
                    applied_torque = min(
                        applied_torque,
                        available_w * efficiency * self.battery.inverter_efficiency / omega,
                    )
                    battery_power_limited = True
            applied_force = applied_torque * ratio * eta_g / radius
            saturated = (
                speed_limited
                or battery_power_limited
                or abs(applied_torque - requested_torque) > 1e-9
            )

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
            vehicle_speed_mps=vehicle_speed_mps,
            motor_speed_rad_s=omega,
            motor_torque_nm=applied_torque,
            motor_efficiency=efficiency,
            battery_power_w=battery_power,
            traction_power_w=traction_power,
            auxiliary_power_w=self.battery.auxiliary_power_w,
            saturated=saturated,
            speed_limited=speed_limited,
            regenerative_wheel_force_n=regenerative_force,
            friction_brake_force_n=friction_force,
            motor_temperature_c=self.thermal_state.temperature_c,
            thermal_derating_factor=self.thermal_derating_factor,
            battery_power_limited=battery_power_limited,
        )

    def update_thermal(self, step: PowertrainStep, dt_s: float) -> None:
        if dt_s <= 0:
            raise ValueError("dt_s must be positive")
        if not self.thermal.enabled:
            return
        mechanical_power = step.motor_torque_nm * step.motor_speed_rad_s
        if mechanical_power >= 0:
            motor_loss_w = mechanical_power * (1.0 / step.motor_efficiency - 1.0)
        else:
            motor_loss_w = abs(mechanical_power) * (1.0 - step.motor_efficiency)
        scale = self.hardware.motor_scale
        capacity = self.thermal.base_thermal_capacity_j_per_k * scale
        conductance = self.thermal.base_thermal_conductance_w_per_k * scale**0.7
        cooling_w = conductance * (
            self.thermal_state.temperature_c - self.thermal.ambient_temperature_c
        )
        self.thermal_state.temperature_c += (motor_loss_w - cooling_w) * dt_s / capacity
        self.thermal_state.peak_temperature_c = max(
            self.thermal_state.peak_temperature_c, self.thermal_state.temperature_c
        )
        self.thermal_state.accumulated_loss_kj += motor_loss_w * dt_s / 1000.0
