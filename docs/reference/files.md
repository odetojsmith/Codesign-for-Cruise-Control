# File inventory

This page is the source-code map. Every row states why a file exists, its principal interface, and
where to look for verification.

## Project entry files

| File | Responsibility |
|---|---|
| [`PLAN.md`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/PLAN.md) | Original experiment and implementation plan |
| [`project_status.md`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/project_status.md) | Current completed work, evidence, limitations, and next boundary |
| [`README.md`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/README.md) | Installation and common commands |
| [`pyproject.toml`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/pyproject.toml) | Package metadata, dependency groups, test/lint settings, CLI entry points |
| [`mkdocs.yml`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/mkdocs.yml) | Documentation navigation, theme, search, math, and Mermaid configuration |
| [`configs/default.yaml`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/configs/default.yaml) | Default hardware, vehicle, motor, battery, and simulation values |

## Runtime package

| File | Principal interfaces | Responsibility |
|---|---|---|
| [`config.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/src/codesign/config.py) | `HardwareDesign`, `VehicleConfig`, `MotorConfig`, `BatteryConfig`, `ProjectConfig` | Immutable configuration and YAML loading |
| [`efficiency.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/src/codesign/efficiency.py) | `EfficiencyMap`, `default_motoring_map` | Validated bilinear motor-map interpolation |
| [`powertrain.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/src/codesign/powertrain.py) | `EVPowertrain`, `PowertrainStep`, `EnergyState` | Final-drive conversion, limits, regeneration, battery power and Wh |
| [`metadrive_env.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/src/codesign/metadrive_env.py) | `MetaDriveEVEnv`, `LaneState`, `LeadVehicleState` | MetaDrive dynamics adapter, observations, force application, rendering |
| [`controllers.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/src/codesign/controllers.py) | `PID`, `LongitudinalPIDController`, `CenterlinePIDController` | Validation speed and centerline controllers |
| [`scenarios.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/src/codesign/scenarios.py) | `SpeedProfile`, controller protocols, `run_speed_profile`, result records | Deterministic profiles, execution, trajectory logging, metrics |
| [`speed_planner.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/src/codesign/speed_planner.py) | `SpeedPlan`, `curvature_aware_speed_plan` | Curvature limits and feasible reference preview |
| [`mpc.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/src/codesign/mpc.py) | `LongitudinalMPCController`, `MPCDiagnostics` | Parametric OSQP longitudinal controller |
| [`mpc_sweep.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/src/codesign/mpc_sweep.py) | `WeightCandidate`, `SweepResult`, `pareto_mask` | Live weight grid and Pareto extraction |
| `hardware_sizing.py` | `evaluate_hardware`, `size_hardware` | Backward-cycle sizing and vehicle-level feasibility checks |
| `optimization.py` | `ClosedLoopEvaluator`, `EvaluationCache` | Shared three-scenario evaluator and resumable SQLite cache |
| `separate_optimization.py` | `run_separate_optimization` | Frozen-hardware controller sweep at external RMSE bounds |
| `co_design.py` | `run_codesign`, `alternating_search` | Nested hardware/controller grid and alternating search |
| `hardware_sensitivity.py` | `sample_hardware_designs` | Fixed-controller hardware influence experiment |
| [`braking_validation.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/src/codesign/braking_validation.py) | Brake calibration and virtual lead environment | Blended and lead-braking evidence |
| [`trajectory_animation.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/src/codesign/trajectory_animation.py) | Low/high RMSE comparison runner | Animated distance, speed, force, and energy trajectories |
| [`scenario_gifs.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/src/codesign/scenario_gifs.py) | Annotated top-down recorder | Urban and curved default-MPC GIFs |
| [`steering_validation.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/src/codesign/steering_validation.py) | Steering sweep and step-response records | Open-loop command, curvature, symmetry, and transient checks |

## Commands

| File / command | Purpose |
|---|---|
| [`smoke.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/src/codesign/smoke.py) / `codesign-smoke` | Quick core and optional MetaDrive checks |
| [`calibration.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/src/codesign/calibration.py) / `codesign-calibrate` | Eight-point traction/regeneration calibration |
| [`steering_validation.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/src/codesign/steering_validation.py) / `codesign-steering-validate` | Open-loop steering validation and JSON evidence |
| [`scenario_cli.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/src/codesign/scenario_cli.py) / `codesign-scenario` | Run one PID/reference scenario and save trajectory/metrics |
| [`validation_cli.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/src/codesign/validation_cli.py) / `codesign-validate` | Generate visual evidence and enforce acceptance checks |
| [`mpc_cli.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/src/codesign/mpc_cli.py) / `codesign-mpc-validate` | PID/MPC live comparison and acceptance report |
| `codesign-mpc-sweep` | Sample MPC weights on urban and curved live scenarios |
| `codesign-braking-validate` | Validate total braking, brake split, and MPC safety gap |
| `codesign-trajectory-animation` | Generate the synchronized low/high RMSE trajectory GIF |
| `codesign-scenario-gifs` | Generate annotated urban and curved MetaDrive GIFs |
| `codesign-size-hardware` | Run the 117-point conventional hardware-sizing grid |
| `codesign-separate-opt` | Tune MPC after freezing conventionally selected hardware |
| `codesign-optimize --quick` | Run cached quick nested and alternating co-design |
| `codesign-hardware-sensitivity` | Compare nine hardware designs with one fixed MPC |
| [`mountain_shuttle.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/src/codesign/mountain_shuttle.py) / `codesign-mountain-shuttle --quick` | Repeated ±10% hill mission and separate-versus-co-design comparison |
| [`generality_dataset.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/src/codesign/generality_dataset.py) / `codesign-generality-dataset --quick` | Train/test scenario manifest, adaptive-controller hardware selection, and held-out evaluation |

## Tests

| File | Coverage |
|---|---|
| [`test_config.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/tests/test_config.py) | YAML loading and invalid hardware values |
| [`test_efficiency.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/tests/test_efficiency.py) | Bilinear interpolation and boundary clamping |
| [`test_powertrain.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/tests/test_powertrain.py) | Ratio, scaling, limits, regeneration, auxiliaries, reverse prevention |
| [`test_controllers.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/tests/test_controllers.py) | PID reset, saturation direction, longitudinal/lateral command signs |
| [`test_scenarios.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/tests/test_scenarios.py) | Profile interpolation, runner metrics, lead/lane logging, CSV output |
| [`test_steering_validation.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/tests/test_steering_validation.py) | Wrapped yaw increments and response-threshold extraction |
| [`test_speed_planner.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/tests/test_speed_planner.py) | Curvature cap and forward/backward speed feasibility |
| [`test_mpc.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/tests/test_mpc.py) | Tracking, jerk, curvature, lead braking, slack, and determinism |
| [`test_mpc_sweep.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/tests/test_mpc_sweep.py) | Nondominated filtering and infeasible-point rejection |
| `test_hardware_sizing.py` | Road loads, top-speed rejection, and conventional selection |
| `test_optimization.py` | SQLite cache serialization and recovery |
| `test_separate_optimization.py` | External-RMSE selection and Pareto logic |
| `test_co_design.py` | Energy selection after applying the RMSE constraint |
| `test_hardware_sensitivity.py` | Hardware-grid matrix placement for plotting |
| `test_mountain_shuttle.py` | Repeated mission definition, battery power caps, and thermal derating |
| `test_generality_dataset.py` | Split isolation, profile consistency, per-scenario control tuning, and training-only hardware selection |

## Documentation

The `docs/` hierarchy mirrors the implementation:

```text
docs/
├── architecture/    system ownership and data flow
├── models/          hardware and EV energy equations
├── simulation/      MetaDrive integration
├── control/         interfaces, PID, and MPC boundary
├── experiments/     scenarios, metrics, reproducibility
├── optimization/    separate and integrated methods
├── validation/      acceptance evidence and visuals
└── reference/       configuration, file map, traceability
```

## Files not yet present

The full-resolution Optuna search, multi-seed robustness runner, CARLA adapter, and Windows
validation runner are not yet present.
