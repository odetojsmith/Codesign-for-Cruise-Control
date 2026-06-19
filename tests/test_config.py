from pathlib import Path

import pytest

from codesign.config import HardwareDesign, ProjectConfig


def test_default_config_loads() -> None:
    config = ProjectConfig.from_yaml(Path("configs/default.yaml"))
    assert config.hardware == HardwareDesign(final_drive_ratio=9.0, motor_scale=1.0)
    assert config.control_interval_s == pytest.approx(0.2)


@pytest.mark.parametrize(
    ("ratio", "scale"),
    [(0.0, 1.0), (-1.0, 1.0), (9.0, 0.0), (9.0, -0.1)],
)
def test_hardware_design_rejects_nonpositive_values(ratio: float, scale: float) -> None:
    with pytest.raises(ValueError):
        HardwareDesign(ratio, scale)

