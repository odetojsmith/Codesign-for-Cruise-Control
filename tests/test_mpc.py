import pytest

from codesign import EVPowertrain, HardwareDesign
from codesign.mpc import LongitudinalMPCController
from codesign.scenarios import ControlObservation


def make_controller(**kwargs) -> LongitudinalMPCController:
    return LongitudinalMPCController(
        EVPowertrain(HardwareDesign(9.0, 1.0)), horizon_steps=8, **kwargs
    )


def observation(
    speed: float,
    reference: float,
    previous_force: float = 0.0,
    gap: float | None = None,
    lead_speed: float | None = None,
    grade: float = 0.0,
) -> ControlObservation:
    return ControlObservation(
        time_s=0.0,
        speed_mps=speed,
        reference_speed_mps=reference,
        previous_force_n=previous_force,
        lead_gap_m=gap,
        lead_speed_mps=lead_speed,
        reference_preview_mps=(reference,) * 9,
        curvature_preview_per_m=(0.0,) * 9,
        grade_preview_fraction=(grade,) * 9,
    )


def test_mpc_accelerates_toward_reference_and_respects_jerk() -> None:
    controller = make_controller()
    force = controller.command(observation(5.0, 10.0))
    assert 0.0 < force <= controller.mass_kg * controller.maximum_jerk_mps3 * controller.dt_s
    assert controller.last_diagnostics is not None
    assert not controller.last_diagnostics.used_fallback


def test_mpc_brakes_for_slow_close_lead_vehicle() -> None:
    controller = make_controller()
    force = controller.command(observation(10.0, 12.0, gap=12.0, lead_speed=3.0))
    assert force < 0.0
    assert controller.last_diagnostics is not None
    assert controller.last_diagnostics.used_fallback


def test_mpc_predicts_bounded_lead_braking_without_slack_when_gap_is_feasible() -> None:
    controller = make_controller()
    force = controller.command(observation(10.0, 10.0, gap=60.0, lead_speed=10.0))
    assert force <= controller.powertrain.force_limits(10.0)[1]
    assert controller.last_diagnostics is not None
    assert controller.last_diagnostics.safety_slack_m < 1e-3
    assert not controller.last_diagnostics.used_fallback


def test_curvature_reduces_longitudinal_force_bound() -> None:
    controller = make_controller(combined_acceleration_limit_mps2=3.0)
    reference = controller._pad((10.0,) * 9, 9, 10.0)
    straight = controller._pad((0.0,) * 9, 9, 0.0)
    curved = controller._pad((0.025,) * 9, 9, 0.0)
    _, straight_maximum = controller._force_bounds(reference, straight)
    _, curved_maximum = controller._force_bounds(reference, curved)
    assert curved_maximum[0] < straight_maximum[0]


def test_mpc_is_deterministic_after_reset() -> None:
    controller = make_controller()
    first = controller.command(observation(6.0, 9.0))
    controller.reset()
    second = controller.command(observation(6.0, 9.0))
    assert second == pytest.approx(first, abs=1e-4)


def test_uphill_grade_increases_required_mpc_force() -> None:
    flat = make_controller().command(observation(10.0, 10.0, grade=0.0))
    uphill = make_controller().command(observation(10.0, 10.0, grade=0.06))
    assert uphill > flat
