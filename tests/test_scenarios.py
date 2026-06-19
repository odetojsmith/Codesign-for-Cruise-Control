from dataclasses import dataclass, field

import pytest

from codesign.metadrive_env import LeadVehicleState
from codesign.powertrain import EnergyState, PowertrainStep
from codesign.scenarios import ProportionalForceController, SpeedProfile, run_speed_profile


@dataclass
class FakeEnvironment:
    control_interval_s: float = 0.2
    speed_mps: float = 0.0
    position_xy_m: tuple[float, float] = (0.0, 0.0)
    energy: EnergyState = field(default_factory=EnergyState)
    last_powertrain_step: PowertrainStep | None = None

    def reset(self):
        self.speed_mps = 0.0
        self.position_xy_m = (0.0, 0.0)
        self.energy = EnergyState()
        return object(), {}

    def lead_vehicle_state(self, lateral_tolerance_m: float = 2.0):
        return LeadVehicleState(30.0, 5.0)

    def step(self, action):
        _, force = action
        acceleration = force / 1000.0
        self.speed_mps = max(0.0, self.speed_mps + acceleration * self.control_interval_s)
        self.position_xy_m = (
            self.position_xy_m[0] + self.speed_mps * self.control_interval_s,
            0.0,
        )
        battery_power = max(force * self.speed_mps, 0.0) + 500.0
        self.last_powertrain_step = PowertrainStep(
            requested_wheel_force_n=force,
            applied_wheel_force_n=force,
            motor_speed_rad_s=0.0,
            motor_torque_nm=0.0,
            motor_efficiency=0.9,
            battery_power_w=battery_power,
            traction_power_w=battery_power - 500.0,
            auxiliary_power_w=500.0,
            saturated=False,
            speed_limited=False,
        )
        self.energy.update(self.last_powertrain_step, self.control_interval_s)
        return object(), 0.0, False, False, {}


def test_profile_interpolates_and_validates() -> None:
    profile = SpeedProfile("test", (0.0, 1.0, 2.0), (0.0, 10.0, 10.0))
    assert profile.reference_at(0.5) == pytest.approx(5.0)
    with pytest.raises(ValueError):
        SpeedProfile("bad", (0.0, 0.0), (0.0, 1.0))


def test_speed_profile_runner_records_independent_metrics(tmp_path) -> None:
    profile = SpeedProfile("test", (0.0, 1.0, 2.0), (0.0, 4.0, 4.0))
    result = run_speed_profile(FakeEnvironment(), profile, ProportionalForceController())
    assert result.metrics.completed
    assert result.metrics.distance_m > 0
    assert result.metrics.net_battery_wh > 0
    assert result.metrics.minimum_gap_m == pytest.approx(30.0)
    assert len(result.trajectory) == 11

    output = tmp_path / "trajectory.csv"
    result.write_csv(output)
    assert output.read_text().startswith("time_s,reference_speed_mps")
