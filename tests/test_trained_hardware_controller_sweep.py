from codesign.trained_hardware_controller_sweep import (
    ControllerSweepPoint,
    dense_controller_grid,
    pareto_flags,
)


def _point(rmse: float, energy: float, feasible: bool = True) -> ControllerSweepPoint:
    return ControllerSweepPoint(0.0, 0.0, rmse, energy, rmse, 4, 4, feasible)


def test_dense_controller_grid_contains_40_unique_candidates() -> None:
    grid = dense_controller_grid()
    assert len(grid) == 40
    assert len(set(grid)) == 40


def test_controller_pareto_flags_do_not_connect_or_retain_dominated_points() -> None:
    points = [
        _point(0.30, 300.0),
        _point(0.35, 280.0),
        _point(0.36, 310.0),
        _point(0.20, 250.0, feasible=False),
    ]
    assert pareto_flags(points) == [True, True, False, False]
