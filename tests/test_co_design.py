from codesign import HardwareDesign
from codesign.co_design import _best
from codesign.optimization import ControllerDesign, EvaluationSummary


def summary(key: str, rmse: float, energy: float) -> EvaluationSummary:
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
        base_feasible=True,
        violations=(),
        scenario_metrics={},
    )


def test_best_uses_energy_only_after_applying_rmse_bound() -> None:
    candidates = [summary("tight", 0.3, 100.0), summary("efficient", 0.7, 70.0)]
    assert _best(candidates, 0.4).key == "tight"
    assert _best(candidates, 0.8).key == "efficient"
    assert _best(candidates, 0.2) is None
