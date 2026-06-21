import numpy as np

from codesign.expanded_generality import (
    EXPANDED_SCENARIO_DATASET,
    STRESS_BOUNDS,
    TRAIN_BOUNDS,
    _latin_hypercube,
    _best_operational_tracking,
    scenario_family,
    screening_controller_grid,
    test_controller_grid as dense_test_controller_grid,
)
from codesign.generality_dataset import ScenarioEvaluation


def test_expanded_dataset_has_disjoint_30_10_split() -> None:
    training = [item for item in EXPANDED_SCENARIO_DATASET if item.split == "train"]
    testing = [item for item in EXPANDED_SCENARIO_DATASET if item.split == "test"]
    assert len(training) == 30
    assert len(testing) == 10
    assert {item.name for item in training}.isdisjoint(item.name for item in testing)
    assert {item.seed for item in training}.isdisjoint(item.seed for item in testing)
    assert sum(scenario_family(item) == "interpolation" for item in testing) == 5
    assert sum(scenario_family(item) == "stress" for item in testing) == 5


def test_latin_hypercube_is_reproducible_and_stratified() -> None:
    first = _latin_hypercube(30, 4, 123)
    second = _latin_hypercube(30, 4, 123)
    assert np.array_equal(first, second)
    assert np.all((first > 0.0) & (first < 1.0))
    for column in range(first.shape[1]):
        strata = np.floor(first[:, column] * 30).astype(int)
        assert sorted(strata) == list(range(30))


def test_stress_cases_extend_key_training_ranges() -> None:
    assert STRESS_BOUNDS["uphill"][1] > TRAIN_BOUNDS["uphill"][1]
    assert STRESS_BOUNDS["payload"][0] >= TRAIN_BOUNDS["payload"][1]
    assert STRESS_BOUNDS["charge"][1] < TRAIN_BOUNDS["charge"][1]
    assert STRESS_BOUNDS["temperature"][1] > TRAIN_BOUNDS["temperature"][1]


def test_expanded_controller_grids_have_expected_density() -> None:
    assert len(screening_controller_grid()) == 15
    assert len(dense_test_controller_grid()) == 40


def _evaluation(rmse: float, energy: float, violations: tuple[str, ...]) -> ScenarioEvaluation:
    return ScenarioEvaluation(
        "case", "test", 11.5, 0.75, 0.0, 0.0, rmse, energy, energy, 100.0,
        1.0, 0.0, 0.0, 60.0, 0.0, 0.0, 0, not violations, violations,
    )


def test_fallback_prefers_completed_mission_over_lower_incomplete_rmse() -> None:
    selected = _best_operational_tracking(
        [
            _evaluation(0.28, 800.0, ("incomplete_episode", "terminal_progress")),
            _evaluation(0.33, 490.0, ()),
            _evaluation(0.36, 480.0, ()),
        ]
    )
    assert selected.rmse_mps == 0.33
    assert selected.wh_per_km == 490.0
