import pytest

from codesign.speed_planner import build_speed_plan, curvature_aware_speed_plan


def test_curve_limit_reduces_speed_and_backward_pass_brakes_before_curve() -> None:
    mission = (12.0, 12.0, 12.0, 12.0)
    curvature = (0.0, 0.0, 0.08, 0.08)
    planned = curvature_aware_speed_plan(mission, curvature, dt_s=0.2)
    assert planned[2] == pytest.approx(5.0)
    assert planned[1] == pytest.approx(5.6)
    assert planned[0] == pytest.approx(6.2)


def test_speed_plan_queries_curvature_at_preview_distance() -> None:
    queried = ()

    def curvature_query(distances):
        nonlocal queried
        queried = distances
        return tuple(0.0 for _ in distances)

    plan = build_speed_plan((5.0, 6.0, 7.0), curvature_query, dt_s=0.2)
    assert queried == pytest.approx((0.0, 1.0, 2.2))
    assert plan.reference_mps == pytest.approx((5.0, 5.6, 6.2))
