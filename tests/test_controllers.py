import pytest

from codesign.controllers import CenterlinePIDController, LongitudinalPIDController, PID
from codesign.metadrive_env import LaneState
from codesign.scenarios import ControlObservation


def test_pid_clamps_and_resets() -> None:
    pid = PID(2.0, 1.0, 0.0, 0.1, -1.0, 1.0)
    assert pid.update(10.0) == pytest.approx(1.0)
    pid.reset()
    assert pid.update(0.0) == pytest.approx(0.0)


def test_longitudinal_pid_requests_positive_force_below_reference() -> None:
    controller = LongitudinalPIDController()
    controller.reset()
    force = controller.command(ControlObservation(0.0, 5.0, 10.0, 0.0))
    assert 0 < force <= controller.maximum_force_n


def test_centerline_pid_uses_metadrive_positive_error_convention() -> None:
    controller = CenterlinePIDController()
    controller.reset()
    steering = controller.command(LaneState(1.0, 0.0, 3.5))
    assert steering > 0
