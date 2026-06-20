# Controller interfaces

## Purpose

Controllers are isolated from MetaDrive and the energy model. They receive physical observations
and return requests; the actuator layer remains authoritative for feasibility.

## Longitudinal observation

`ControlObservation` contains:

| Field | Unit | Meaning |
|---|---:|---|
| `time_s` | s | Scenario time |
| `speed_mps` | m/s | Current ego speed |
| `reference_speed_mps` | m/s | Planner/reference command |
| `previous_force_n` | N | Previous requested force |
| `lead_gap_m` | m or absent | Nearest in-lane vehicle gap |
| `lead_speed_mps` | m/s or absent | Lead-vehicle speed |
| `reference_preview_mps` | m/s | Curvature-aware horizon reference |
| `curvature_preview_per_m` | 1/m | Road-curvature horizon |

## Protocols

```python
class LongitudinalController(Protocol):
    def reset(self) -> None: ...
    def command(self, observation: ControlObservation) -> float: ...

class LateralController(Protocol):
    def reset(self) -> None: ...
    def command(self, lane: LaneState) -> float: ...
```

The longitudinal output is wheel force in newtons. The lateral output is normalized steering in
$[-1,1]$.

## Why the interface matters

- PID and MPC use the same scenario runner and planned reference.
- Final metrics are independent of controller internals.
- Hardware saturation is applied after the controller request.
- MetaDrive and future CARLA adapters can consume the same controller outputs.
- Controller candidates can be optimized without changing scenario logic.

## Episode data

Every step records reference and measured speed, requested and applied force, steering, lane and
heading error, battery power, motor power, efficiency, energy, gap, saturation, acceleration, and
jerk.

The resulting `EpisodeMetrics` includes speed RMSE, Wh/km, distance, peak acceleration and jerk,
minimum gap, saturation fraction, lateral RMSE, maximum lateral error, and completion status.

## Source

- Protocol and observations: [`scenarios.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/src/codesign/scenarios.py)
- PID controllers: [`controllers.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/src/codesign/controllers.py)
- MPC controller: [`mpc.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/src/codesign/mpc.py)
