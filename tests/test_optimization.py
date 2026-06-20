from codesign import HardwareDesign
from codesign.optimization import ControllerDesign, EvaluationCache, EvaluationSummary


def test_sqlite_evaluation_cache_round_trip(tmp_path) -> None:
    cache = EvaluationCache(tmp_path / "evaluations.sqlite3")
    summary = EvaluationSummary(
        key="abc",
        hardware=HardwareDesign(9.0, 1.0),
        controller=ControllerDesign(0.0, -1.0),
        aggregate_rmse_mps=0.3,
        total_net_battery_wh=100.0,
        aggregate_wh_per_km=150.0,
        total_distance_m=666.7,
        peak_acceleration_mps2=2.0,
        peak_jerk_mps3=3.0,
        maximum_lateral_error_m=0.4,
        fallback_count=0,
        maximum_safety_slack_m=0.0,
        completed=True,
        base_feasible=True,
        violations=(),
        scenario_metrics={"urban": {"rmse_mps": 0.3, "completed": True}},
    )
    cache.put("abc", {"request": 1}, summary)
    restored = cache.get("abc")
    assert restored is not None
    assert restored.from_cache
    assert restored.hardware == summary.hardware
    assert restored.controller == summary.controller
    assert restored.violations == ()
    assert cache.count() == 1
