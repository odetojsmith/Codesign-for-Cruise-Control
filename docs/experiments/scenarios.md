# Scenarios and metrics

## Implemented speed profiles

Profiles are immutable piecewise-linear tables evaluated by interpolation.

### Urban stop-go

- Duration: 44 s
- Speed range: 0–12 m/s
- Includes acceleration, cruise, full stop, dwell, and restart
- Exercises traction, auxiliary consumption, and regeneration

### Highway changes

- Duration: 48 s
- Speed range: 16–30 m/s
- Exercises the motor power-limited and high-speed operating regions

The optimization evaluator uses a compact, dynamically feasible launch from 0 to 20 m/s because
MetaDrive's procedural training route ends near 130 m at high speed. The separate hardware check
independently enforces sustained operation at 120 km/h.

### Curved centerline

- Duration: 30 s
- Ramps from 0 to 12 m/s
- Uses repeated straight/curve MetaDrive blocks
- Starts with a 1 m lateral offset
- Produces top-down visual evidence

### Mixed grade

- Duration: 16 s and approximately 168 m
- Grade range: −6% to +6% as a function of route distance
- Speed range: 0–14 m/s
- MPC preview includes the grade disturbance
- MetaDrive receives net chassis force while battery energy uses full wheel force

## Training and future validation matrix

| Scenario | Current status | Intended use |
|---|---|---|
| Urban stop-go | Implemented | Energy and tracking |
| Highway changes | Compact training profile implemented | Closed-loop high-speed launch |
| Curved centerline | Implemented | Lateral-control isolation |
| Deterministic traffic | Lead detection verified | Future safety/MPC |
| Emergency lead braking | Virtual deterministic lead implemented | Safety stress test |
| Grade ±6% | Implemented | Hardware/energy coupling |
| Payload ±10% | Planned | Robustness |
| Drag ±10% | Planned | Robustness |
| Friction variation | Planned | Transfer robustness |

## Controller-independent metrics

| Metric | Definition |
|---|---|
| Speed RMSE | RMS difference between measured and reference speed |
| Net Wh | Time integral of battery power |
| Wh/km | Net Wh divided by traveled kilometers |
| Peak acceleration | Maximum absolute discrete speed derivative |
| Peak jerk | Maximum absolute acceleration derivative |
| Minimum gap | Smallest detected in-lane lead gap |
| Saturation fraction | Fraction of steps with powertrain clipping |
| Lateral RMSE | RMS lane-center error |
| Maximum lateral error | Largest absolute lane-center error |
| Completion | Whether full profile duration executed |

## Result files

Each run can write:

- trajectory CSV with every control-step variable;
- JSON metrics;
- top-down PNG/GIF;
- validation dashboard;
- machine-readable pass/fail report.

## Source and tests

- Profiles and runner: [`scenarios.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/src/codesign/scenarios.py)
- Scenario CLI: [`scenario_cli.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/src/codesign/scenario_cli.py)
- Tests: [`test_scenarios.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/tests/test_scenarios.py)
