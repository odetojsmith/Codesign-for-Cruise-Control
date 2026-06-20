import pytest

from codesign import EVPowertrain, HardwareDesign
from codesign.config import ProjectConfig
from codesign.hardware_sizing import evaluate_hardware, size_hardware


def test_road_load_increases_with_speed_and_uphill_grade() -> None:
    powertrain = EVPowertrain(HardwareDesign(9.0, 1.0))
    assert powertrain.road_load_force(20.0) > powertrain.road_load_force(10.0)
    assert powertrain.road_load_force(10.0, 0.06) > powertrain.road_load_force(10.0)
    assert powertrain.road_load_force(10.0, -0.06) < powertrain.road_load_force(10.0)


def test_high_ratio_fails_120_kph_motor_speed_requirement() -> None:
    config = ProjectConfig.from_yaml("configs/default.yaml")
    result = evaluate_hardware(config, HardwareDesign(12.0, 1.0))
    assert not result.top_speed_feasible


def test_conventional_sizing_selects_a_feasible_grid_candidate() -> None:
    config = ProjectConfig.from_yaml("configs/default.yaml")
    results, selected = size_hardware(
        config,
        final_drive_ratios=(7.0, 9.0, 11.0),
        motor_scales=(0.8, 1.0, 1.2),
    )
    assert len(results) == 9
    assert selected.feasible
    assert selected.selected
    assert selected.cycle_wh_per_km == pytest.approx(
        min(
            result.cycle_wh_per_km
            for result in results
            if result.feasible and result.motor_mass_kg == selected.motor_mass_kg
        ),
        rel=0.005,
    )
