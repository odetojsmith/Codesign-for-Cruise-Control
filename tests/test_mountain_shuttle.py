import pytest

from codesign.config import BatteryConfig, HardwareDesign, MotorThermalConfig
from codesign.mountain_shuttle import (
    MOUNTAIN_SHUTTLE_GRADE,
    MOUNTAIN_SHUTTLE_PROFILE,
    REFERENCE_DISTANCE_M,
)
from codesign.powertrain import EVPowertrain


def test_mountain_shuttle_has_repeated_hills_and_stops() -> None:
    assert MOUNTAIN_SHUTTLE_PROFILE.duration_s == pytest.approx(100.0)
    assert MOUNTAIN_SHUTTLE_PROFILE.speed_mps.count(0.0) >= 8
    assert max(MOUNTAIN_SHUTTLE_GRADE.grade_fraction) == pytest.approx(0.10)
    assert min(MOUNTAIN_SHUTTLE_GRADE.grade_fraction) == pytest.approx(-0.10)
    assert MOUNTAIN_SHUTTLE_GRADE.distance_m[-1] == pytest.approx(REFERENCE_DISTANCE_M)


def test_battery_power_limits_reduce_traction_and_regeneration() -> None:
    limited = EVPowertrain(
        HardwareDesign(9.0, 1.0),
        battery=BatteryConfig(
            maximum_discharge_power_kw=30.0,
            maximum_charge_power_kw=15.0,
        ),
    )
    traction = limited.evaluate(1e9, 20.0)
    braking = limited.evaluate(-1e9, 20.0)
    assert traction.battery_power_limited
    assert traction.battery_power_w <= 30_000.0 + 1.0
    assert braking.battery_power_limited
    assert braking.battery_power_w >= -15_000.0 - 1.0
    assert braking.friction_brake_force_n < 0.0


def test_motor_heating_eventually_derates_available_torque() -> None:
    thermal = MotorThermalConfig(
        enabled=True,
        initial_temperature_c=80.0,
        derating_start_temperature_c=81.0,
        maximum_temperature_c=90.0,
        base_thermal_capacity_j_per_k=5_000.0,
        base_thermal_conductance_w_per_k=1.0,
    )
    powertrain = EVPowertrain(HardwareDesign(9.0, 0.6), thermal=thermal)
    initial_limit = powertrain.torque_limit(powertrain.motor_speed(15.0))
    for _ in range(100):
        step = powertrain.evaluate(1e9, 15.0)
        powertrain.update_thermal(step, 0.2)
    assert powertrain.thermal_state.temperature_c > thermal.derating_start_temperature_c
    assert powertrain.torque_limit(powertrain.motor_speed(15.0)) < initial_limit
