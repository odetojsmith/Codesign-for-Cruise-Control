import pytest

from codesign import EnergyState, EVPowertrain, HardwareDesign


def test_final_drive_changes_motor_speed_and_wheel_force() -> None:
    low_ratio = EVPowertrain(HardwareDesign(6.0, 1.0))
    high_ratio = EVPowertrain(HardwareDesign(12.0, 1.0))
    assert high_ratio.motor_speed(10.0) == pytest.approx(2 * low_ratio.motor_speed(10.0))

    low_step = low_ratio.evaluate(1e9, 0.0)
    high_step = high_ratio.evaluate(1e9, 0.0)
    assert high_step.applied_wheel_force_n == pytest.approx(2 * low_step.applied_wheel_force_n)


def test_motor_scaling_changes_limits_and_mass() -> None:
    small = EVPowertrain(HardwareDesign(9.0, 0.6))
    large = EVPowertrain(HardwareDesign(9.0, 1.4))
    assert large.peak_torque_nm > small.peak_torque_nm
    assert large.peak_power_w > small.peak_power_w
    assert large.total_vehicle_mass_kg > small.total_vehicle_mass_kg


def test_power_limit_reduces_high_speed_torque() -> None:
    powertrain = EVPowertrain(HardwareDesign(9.0, 1.0))
    low_speed_limit = powertrain.torque_limit(powertrain.max_speed_rad_s * 0.1)
    high_speed_limit = powertrain.torque_limit(powertrain.max_speed_rad_s * 0.9)
    assert high_speed_limit < low_speed_limit


def test_overspeed_prevents_traction() -> None:
    powertrain = EVPowertrain(HardwareDesign(12.0, 1.0))
    speed_above_limit = (
        powertrain.max_speed_rad_s
        * powertrain.vehicle.wheel_radius_m
        / powertrain.hardware.final_drive_ratio
        * 1.01
    )
    step = powertrain.evaluate(1000.0, speed_above_limit)
    assert step.speed_limited
    assert step.applied_wheel_force_n == pytest.approx(0.0)


def test_braking_blends_regeneration_and_friction_without_counting_friction_as_energy() -> None:
    powertrain = EVPowertrain(HardwareDesign(9.0, 1.0))
    regenerative_limit = abs(powertrain.evaluate(-1e9, 10.0).regenerative_wheel_force_n)
    requested = -(regenerative_limit + 1000.0)
    step = powertrain.evaluate(requested, 10.0)
    assert step.applied_wheel_force_n == pytest.approx(requested)
    assert step.regenerative_wheel_force_n == pytest.approx(-regenerative_limit)
    assert step.friction_brake_force_n == pytest.approx(-1000.0)
    assert step.traction_power_w == pytest.approx(
        step.motor_torque_nm * step.motor_speed_rad_s * step.motor_efficiency * 0.97
    )


def test_regeneration_reduces_net_energy() -> None:
    powertrain = EVPowertrain(HardwareDesign(9.0, 1.0))
    energy = EnergyState()
    motoring = powertrain.evaluate(2000.0, 15.0)
    regeneration = powertrain.evaluate(-2000.0, 15.0)
    energy.update(motoring, 1.0)
    energy_before_regen = energy.net_battery_wh
    energy.update(regeneration, 1.0)
    assert energy.regenerated_wh > 0
    assert energy.net_battery_wh < energy_before_regen


def test_auxiliary_load_consumes_energy_at_rest() -> None:
    powertrain = EVPowertrain(HardwareDesign(9.0, 1.0))
    step = powertrain.evaluate(0.0, 0.0)
    assert step.battery_power_w == pytest.approx(powertrain.battery.auxiliary_power_w)


def test_regenerative_force_cannot_reverse_vehicle_from_rest() -> None:
    powertrain = EVPowertrain(HardwareDesign(9.0, 1.0))
    step = powertrain.evaluate(-2000.0, 0.0)
    assert step.applied_wheel_force_n == pytest.approx(0.0)
