import pytest

from codesign.config import HardwareDesign
from codesign.generality_dataset import (
    SCENARIO_DATASET,
    ScenarioDefinition,
    ScenarioEvaluation,
    select_controller,
    select_training_hardware,
    training_hardware_pareto_flags,
)


def _result(
    scenario: str,
    ratio: float,
    scale: float,
    energy: float,
    controller: float,
    feasible: bool = True,
):
    return ScenarioEvaluation(
        scenario=scenario,
        split="train",
        final_drive_ratio=ratio,
        motor_scale=scale,
        log10_lambda_energy=controller,
        log10_lambda_force_slew=-1.0,
        rmse_mps=0.4,
        net_battery_wh=energy,
        wh_per_km=energy,
        distance_m=1000.0,
        progress_fraction=1.0,
        maximum_station_distance_error_m=0.2,
        maximum_station_speed_mps=0.1,
        peak_motor_temperature_c=60.0,
        friction_brake_wh=10.0,
        recovered_battery_wh=20.0,
        fallback_count=0,
        feasible=feasible,
        violations=() if feasible else ("tracking_rmse",),
    )


def test_dataset_split_is_disjoint_and_profiles_are_consistent() -> None:
    train = {scenario.name for scenario in SCENARIO_DATASET if scenario.split == "train"}
    test = {scenario.name for scenario in SCENARIO_DATASET if scenario.split == "test"}
    assert len(train) == 4
    assert len(test) == 3
    assert train.isdisjoint(test)
    for scenario in SCENARIO_DATASET:
        assert scenario.speed_profile().duration_s == pytest.approx(scenario.duration_s)
        assert scenario.grade_profile().distance_m[-1] == pytest.approx(
            scenario.reference_distance_m
        )


def test_controller_is_selected_independently_per_scenario() -> None:
    first = select_controller([_result("a", 10.0, 0.75, 250.0, -1.0), _result("a", 10.0, 0.75, 220.0, 0.5)])
    second = select_controller([_result("b", 10.0, 0.75, 210.0, -1.0), _result("b", 10.0, 0.75, 230.0, 0.5)])
    assert first is not None and second is not None
    assert first.log10_lambda_energy == 0.5
    assert second.log10_lambda_energy == -1.0


def test_controller_selection_minimizes_energy_only_inside_rmse_feasible_set() -> None:
    selected = select_controller(
        [
            _result("a", 10.0, 0.75, 190.0, -1.0),
            _result("a", 10.0, 0.75, 170.0, 0.5),
            _result("a", 10.0, 0.75, 140.0, 1.0, feasible=False),
        ]
    )
    assert selected is not None
    assert selected.net_battery_wh == pytest.approx(170.0)


def test_training_hardware_requires_all_training_scenarios() -> None:
    scenarios = (
        ScenarioDefinition("a", "train", 1, 1, 10, 1, 2, 2, 2, 0.05, -0.05),
        ScenarioDefinition("b", "train", 2, 1, 10, 1, 2, 2, 2, 0.05, -0.05),
    )
    hardware = (HardwareDesign(9.0, 0.6), HardwareDesign(11.0, 0.75))
    selections = {
        (9.0, 0.6, "a"): _result("a", 9.0, 0.6, 180.0, 0.5),
        (11.0, 0.75, "a"): _result("a", 11.0, 0.75, 170.0, 0.5),
        (11.0, 0.75, "b"): _result("b", 11.0, 0.75, 175.0, -1.0),
    }
    selected, summaries = select_training_hardware(selections, hardware, scenarios)
    assert selected == HardwareDesign(11.0, 0.75)
    assert sum(bool(row["feasible"]) for row in summaries) == 1


def test_training_hardware_pareto_flags_reject_dominated_points() -> None:
    summaries = [
        {"feasible": True, "mean_rmse_mps": 0.30, "mean_wh_per_km": 300.0},
        {"feasible": True, "mean_rmse_mps": 0.35, "mean_wh_per_km": 280.0},
        {"feasible": True, "mean_rmse_mps": 0.36, "mean_wh_per_km": 310.0},
        {"feasible": False, "mean_rmse_mps": 0.20, "mean_wh_per_km": 250.0},
    ]
    assert training_hardware_pareto_flags(summaries) == [True, True, False, False]
