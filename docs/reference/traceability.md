# Traceability matrix

This matrix connects engineering requirements to implementation, verification, and visible
evidence. “Planned” rows are specifications rather than completed claims.

## Implemented requirements

| Requirement | Implementation | Automated test/check | Evidence |
|---|---|---|---|
| Hardware record rejects invalid values | `config.py` | `test_config.py` | Test suite |
| Final drive changes motor speed and wheel force | `powertrain.py` | `test_powertrain.py` | Actuator calibration plot |
| Motor scaling changes torque, power, and mass | `powertrain.py` | `test_powertrain.py` | Hardware model page |
| Torque falls in power-limited region | `powertrain.py` | `test_powertrain.py` | Unit test |
| Overspeed prevents traction | `powertrain.py` | `test_powertrain.py` | Unit test |
| Regeneration recovers energy | `powertrain.py` | `test_powertrain.py` | Urban battery-power plot |
| Vehicle cannot reverse under zero-speed regen | `powertrain.py` | `test_powertrain.py` | Unit test |
| MetaDrive receives correct physical force | `metadrive_env.py` | `codesign-validate` | Eight-point $ma=F$ plot |
| Steering command reaches MetaDrive | `metadrive_env.py` | `codesign-validate` | Command-handoff plot |
| Steering response has correct sign and scale | `steering_validation.py` | `codesign-validate` | Curvature sweep |
| Steering response is left/right symmetric | `steering_validation.py` | `codesign-validate` | Symmetry plot |
| Steering transient is characterized | `steering_validation.py` | `codesign-validate` | Step-yaw-rate plot |
| Curve speed is limited consistently | `speed_planner.py` | `test_speed_planner.py` | Curved MPC reference plot |
| Braking blends regeneration and friction | `powertrain.py` | `test_powertrain.py` | Force trajectory fields |
| MPC respects force, jerk, and curvature limits | `mpc.py` | `test_mpc.py`, `codesign-mpc-validate` | MPC dashboard |
| MPC solver failure is observable | `MPCDiagnostics` | `test_mpc.py` | Fallback counts in report |
| MPC weights expose energy–tracking tradeoff | `mpc_sweep.py` | live 25-point sweep | Pareto plot and CSV |
| Total blended braking reaches chassis | `braking_validation.py` | `codesign-braking-validate` | $ma$ force plot |
| MPC maintains deterministic braking gap | `braking_validation.py` | `codesign-braking-validate` | Gap-requirement plot |
| Energy integration is consistent | `EnergyState` | `codesign-validate` | 0 Wh residual |
| Driveline power conversion is consistent | `EVPowertrain` | `codesign-validate` | $<1.1\times10^{-11}$ W residual |
| Speed reference can be tracked | longitudinal PID | live scenarios | Speed plots |
| Vehicle stays inside lane | centerline PID | `codesign-validate` | Dashboard and top-down GIF |
| Scenario metrics are controller-independent | `scenarios.py` | `test_scenarios.py` | CSV/JSON results |
| Lead-vehicle gap is observable | `metadrive_env.py` | deterministic live check | Project status |
| Grade disturbance affects dynamics and MPC | `metadrive_env.py`, `mpc.py` | `test_mpc.py` | Mixed-grade smoke run |
| Fixed-RMSE energy minimization | `separate_optimization.py` | `test_separate_optimization.py` | Quick-grid report |
| Traditional hardware baseline | `hardware_sizing.py` | `test_hardware_sizing.py` | Feasibility map and CSV |
| Integrated co-design | `co_design.py` | `test_co_design.py` | Quick Pareto frontier |
| Alternating method | `alternating_search` | cached quick run | Iteration records in JSON |
| Optimization resumes safely | `EvaluationCache` | `test_optimization.py` | 54-entry SQLite database |
| Train/test hardware isolation | `generality_dataset.py` | `test_generality_dataset.py` | Versioned split and held-out report |
| Matched-RMSE comparison | `matched_rmse_test.py` | `test_matched_rmse_test.py` | Three held-out dominance results |
| Dense trained-hardware controller frontier | `trained_hardware_controller_sweep.py` | `test_trained_hardware_controller_sweep.py` | 40-point CSV and discrete Pareto plot |

## Planned requirements

| Requirement | Planned implementation | Acceptance evidence |
|---|---|---|
| Physical traffic-actor safe following | Rendered controlled lead vehicle | Minimum-gap plots and stress tests |
| Matched-protocol dual frontiers | Same controller grids for both frozen hardware designs | Training/test frontiers and fixed-RMSE table |
| Robustness | Payload/drag/friction/grade/traffic scenarios | Multi-seed uncertainty distributions |
| Simulator transfer | CARLA adapter | MetaDrive–CARLA comparison table |

## Evidence interpretation

!!! success "What current evidence supports"
    Software-level actuator delivery, modeled power-flow consistency, deterministic scenario
    execution, PID/MPC closed-loop operation, conventional sizing, cached optimization execution,
    lane containment, conventional sizing, mountain co-design, current train/test energy gains,
    and matched-RMSE sampled dominance.

!!! warning "What current evidence does not support"
    Production EV energy accuracy, global optimality, broad-scene generality, or CARLA transfer
    validity. Those claims require the planned data and experiments.
