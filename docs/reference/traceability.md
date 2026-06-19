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
| Energy integration is consistent | `EnergyState` | `codesign-validate` | 0 Wh residual |
| Driveline power conversion is consistent | `EVPowertrain` | `codesign-validate` | $<1.1\times10^{-11}$ W residual |
| Speed reference can be tracked | longitudinal PID | live scenarios | Speed plots |
| Vehicle stays inside lane | centerline PID | `codesign-validate` | Dashboard and top-down GIF |
| Scenario metrics are controller-independent | `scenarios.py` | `test_scenarios.py` | CSV/JSON results |
| Lead-vehicle gap is observable | `metadrive_env.py` | deterministic live check | Project status |

## Planned requirements

| Requirement | Planned implementation | Acceptance evidence |
|---|---|---|
| Safe following constraints | Longitudinal MPC | Minimum-gap plots and stress tests |
| Fixed-RMSE energy minimization | MPC tuning and experiment runner | Energy at matched RMSE bounds |
| Traditional hardware baseline | Backward-facing sizing | Feasibility and Wh/km tables |
| Integrated co-design | Nested grid/Optuna search | Co-design Pareto frontier |
| Alternating method | Alternating optimizer | Iteration convergence plot |
| Robustness | Payload/drag/friction/grade scenarios | Seed and uncertainty distributions |
| Simulator transfer | CARLA adapter | MetaDrive–CARLA comparison table |

## Evidence interpretation

!!! success "What current evidence supports"
    Software-level actuator delivery, modeled power-flow consistency, deterministic scenario
    execution, PID closed-loop operation, lane containment, and metric generation.

!!! warning "What current evidence does not support"
    Production EV energy accuracy, optimal-control superiority, co-design superiority, or CARLA
    transfer validity. Those claims require the planned data and experiments.

