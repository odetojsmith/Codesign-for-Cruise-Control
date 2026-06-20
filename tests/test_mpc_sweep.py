from codesign.mpc_sweep import SweepResult, pareto_mask


def result(key: str, rmse: float, energy: float, feasible: bool = True) -> SweepResult:
    return SweepResult(
        key=key,
        log10_lambda_energy=0.0,
        log10_lambda_force_slew=0.0,
        aggregate_rmse_mps=rmse,
        total_net_battery_wh=energy,
        aggregate_wh_per_km=energy,
        urban_rmse_mps=rmse,
        curved_rmse_mps=rmse,
        peak_acceleration_mps2=1.0,
        peak_jerk_mps3=1.0,
        maximum_lateral_error_m=0.1,
        fallback_count=0,
        maximum_safety_slack_m=0.0,
        completed=True,
        feasible=feasible,
    )


def test_pareto_mask_rejects_dominated_and_infeasible_points() -> None:
    results = [
        result("tracking", 0.1, 12.0),
        result("energy", 0.3, 8.0),
        result("dominated", 0.4, 13.0),
        result("invalid", 0.05, 5.0, feasible=False),
    ]
    assert pareto_mask(results) == [True, True, False, False]
