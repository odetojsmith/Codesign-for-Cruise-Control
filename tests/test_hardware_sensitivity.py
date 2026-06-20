from codesign.hardware_sensitivity import HardwareSensitivityResult, _matrix


def sample(ratio: float, scale: float, rmse: float) -> HardwareSensitivityResult:
    return HardwareSensitivityResult(
        final_drive_ratio=ratio,
        motor_scale=scale,
        motor_mass_kg=75.0 * scale,
        conventional_feasible=True,
        zero_to_100_s=9.0,
        aggregate_rmse_mps=rmse,
        total_net_battery_wh=100.0,
        aggregate_wh_per_km=150.0,
        peak_acceleration_mps2=2.0,
        peak_jerk_mps3=3.0,
        base_control_feasible=True,
        urban_rmse_mps=rmse,
        highway_rmse_mps=rmse,
        mixed_grade_rmse_mps=rmse,
    )


def test_matrix_places_hardware_samples_by_scale_and_ratio() -> None:
    results = [sample(7.0, 0.6, 0.2), sample(9.0, 0.6, 0.3), sample(7.0, 1.0, 0.4)]
    values = _matrix(results, [7.0, 9.0], [0.6, 1.0], "aggregate_rmse_mps")
    assert values[0, 0] == 0.2
    assert values[0, 1] == 0.3
    assert values[1, 0] == 0.4
