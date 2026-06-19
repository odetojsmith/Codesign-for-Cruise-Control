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

## Commands

| File / command | Purpose |
|---|---|
| [`smoke.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/src/codesign/smoke.py) / `codesign-smoke` | Quick core and optional MetaDrive checks |
| [`calibration.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/src/codesign/calibration.py) / `codesign-calibrate` | Eight-point traction/regeneration calibration |
| [`scenario_cli.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/src/codesign/scenario_cli.py) / `codesign-scenario` | Run one PID/reference scenario and save trajectory/metrics |
| [`validation_cli.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/src/codesign/validation_cli.py) / `codesign-validate` | Generate visual evidence and enforce acceptance checks |

## Tests

| File | Coverage |
|---|---|
| [`test_config.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/tests/test_config.py) | YAML loading and invalid hardware values |
| [`test_efficiency.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/tests/test_efficiency.py) | Bilinear interpolation and boundary clamping |
| [`test_powertrain.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/tests/test_powertrain.py) | Ratio, scaling, limits, regeneration, auxiliaries, reverse prevention |
| [`test_controllers.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/tests/test_controllers.py) | PID reset, saturation direction, longitudinal/lateral command signs |
| [`test_scenarios.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/tests/test_scenarios.py) | Profile interpolation, runner metrics, lead/lane logging, CSV output |

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

The following planned modules should not be inferred as implemented:

- longitudinal MPC and solver wrapper;
- backward-facing conventional hardware sizing;
- nested/alternating optimization;
- optimization cache and result database;
- plotting of co-design Pareto fronts;
- CARLA adapter and Windows validation runner.

