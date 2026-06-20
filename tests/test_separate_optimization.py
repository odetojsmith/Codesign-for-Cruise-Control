from codesign import HardwareDesign
from codesign.optimization import ControllerDesign, EvaluationSummary
from codesign.separate_optimization import pareto_flags, select_for_bounds


def result(key: str, rmse: float, energy: float, feasible: bool = True) -> EvaluationSummary:
    return EvaluationSummary(
        key=key,
        hardware=HardwareDesign(9.0, 1.0),
        controller=ControllerDesign(0.0, -1.0),
        aggregate_rmse_mps=rmse,
        total_net_battery_wh=energy,
        aggregate_wh_per_km=150.0,
        total_distance_m=1000.0,
        peak_acceleration_mps2=2.0,
        peak_jerk_mps3=3.0,
        maximum_lateral_error_m=0.2,
        fallback_count=0,
        maximum_safety_slack_m=0.0,
        completed=True,
        base_feasible=feasible,
        violations=() if feasible else ("jerk",),
        scenario_metrics={},
    )


def test_selects_minimum_energy_at_each_external_rmse_bound() -> None:
    results = [result("a", 0.2, 100.0), result("b", 0.4, 80.0), result("c", 0.7, 60.0)]
    selected = select_for_bounds(results, (0.1, 0.3, 0.8))
    assert selected[0.1] is None
    assert selected[0.3].key == "a"
    assert selected[0.8].key == "c"


def test_pareto_flags_exclude_dominated_and_infeasible_points() -> None:
    results = [
        result("a", 0.2, 100.0),
        result("b", 0.4, 80.0),
        result("dominated", 0.5, 110.0),
        result("bad", 0.1, 50.0, feasible=False),
    ]
    assert pareto_flags(results) == [True, True, False, False]
