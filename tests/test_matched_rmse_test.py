from codesign.generality_dataset import ScenarioEvaluation
from codesign.matched_rmse_test import select_minimum_energy_under_rmse


def _result(rmse: float, energy: float, feasible: bool = True) -> ScenarioEvaluation:
    return ScenarioEvaluation(
        scenario="test",
        split="test",
        final_drive_ratio=11.5,
        motor_scale=0.75,
        log10_lambda_energy=0.0,
        log10_lambda_force_slew=-0.5,
        rmse_mps=rmse,
        net_battery_wh=energy,
        wh_per_km=energy,
        distance_m=1000.0,
        progress_fraction=1.0,
        maximum_station_distance_error_m=0.1,
        maximum_station_speed_mps=0.1,
        peak_motor_temperature_c=60.0,
        friction_brake_wh=10.0,
        recovered_battery_wh=20.0,
        fallback_count=0,
        feasible=feasible,
        violations=() if feasible else ("tracking_rmse",),
    )


def test_selection_minimizes_energy_only_below_comparison_rmse() -> None:
    selected = select_minimum_energy_under_rmse(
        [_result(0.31, 220.0), _result(0.34, 200.0), _result(0.36, 170.0)],
        0.35,
    )
    assert selected is not None
    assert selected.rmse_mps == 0.34
    assert selected.wh_per_km == 200.0


def test_selection_rejects_infeasible_candidate_even_when_rmse_matches() -> None:
    selected = select_minimum_energy_under_rmse(
        [_result(0.30, 150.0, feasible=False), _result(0.32, 210.0)],
        0.33,
    )
    assert selected is not None
    assert selected.wh_per_km == 210.0
