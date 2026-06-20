# Project Status

Last updated: 2026-06-20

## Current milestone

The simulator, hardware-dependent actuator, energy and thermal layers, PID/MPC controllers,
conventional sizing, persistent evaluator, and nested/alternating co-design workflow are implemented
and verified. The current recoverable milestone is a disjoint four-scenario training and
three-scenario test experiment with scenario-specific MPC retuning after hardware freezing. The
training-selected hardware reduces held-out energy while satisfying the shared tracking and mission
constraints. The searchable MkDocs site records the implementation and evidence. Broader dataset
validation, sourced motor data, and CARLA transfer validation are next.

## Completed

- Established Python package, dependency groups, default configuration, and test layout.
- Added immutable hardware, vehicle, motor, battery, and project configuration records.
- Implemented final-drive conversion, motor scaling, torque/power/speed saturation, regenerative
  braking, motor-efficiency interpolation, auxiliary load, battery energy, and state of charge.
- Added a MetaDrive wrapper that applies hardware-dependent force limits and records EV energy.
- Switched the ego vehicle to MetaDrive's varying-dynamics vehicle so the requested hardware mass,
  traction force, and regenerative force are used by the actual Bullet chassis.
- Calibrated both positive traction and signed regenerative braking at 25%, 50%, 75%, and 100% of
  their nominal limits; measured force gain is 1.000 at every tested point.
- Added deterministic urban and highway speed profiles, a controller protocol, a temporary
  proportional-force baseline, traffic/lead-vehicle detection, trajectory CSV logging, and
  controller-independent metrics.
- Added longitudinal speed PID and lateral centerline PID controllers for pre-MPC validation.
- Added open-loop steering command-handoff, curvature, symmetry, and step-response validation that
  is independent of the lateral PID.
- Added centerline-curvature preview, lateral-acceleration speed limits, and forward/backward
  feasible-speed passes shared by PID and MPC.
- Added curvature steering feedforward, anti-windup feedback, saturation, and steering-rate limits.
- Added regenerative/friction blended braking with separate force and energy accounting.
- Implemented a 20-step CVXPY/OSQP longitudinal MPC with hardware, acceleration, jerk, curvature,
  gap, bounded lead-braking, diagnostics, and fallback constraints.
- Added a reproducible 25-point live MPC weight sweep with constraint filtering and Pareto
  extraction across urban and curved routes.
- Added blended-brake force calibration and a deterministic 3 m/s² lead-braking MPC validation.
- Added an animated low/high RMSE comparison covering road progress, speed, force, and energy.
- Added annotated top-down GIFs for the urban stop-go and curved default-MPC test scenarios.
- Added synchronized split-screen MetaDrive GIFs comparing independent low- and high-RMSE MPC
  rollouts in both the urban stop-go and curved-road scenarios, including a real-time reference
  ghost rendered on the road and a signed longitudinal position-lag readout.
- Added a reproducible visual-validation command that generates an actuator plot, speed/lateral/
  energy dashboard, top-down GIF and PNG, trajectories, and a machine-readable pass/fail report.
- Added a Material for MkDocs website with 18 hierarchical pages, Mermaid architecture/data-flow
  diagrams, equations, implementation links, source-file inventory, and requirement traceability.
- Added core and optional MetaDrive smoke-test commands.
- Added initial unit tests for configuration, interpolation, hardware effects, saturation, and
  energy accounting.
- Added rolling-resistance, aerodynamic-drag, and signed grade road loads plus grade preview in the
  MPC prediction model and MetaDrive force path.
- Added a deterministic ±6% mixed-grade training episode and compact high-speed training ramp.
- Implemented 117-point backward-facing conventional hardware sizing with top-speed, acceleration,
  gradeability, cycle feasibility, and 0.5% energy/mass tie-breaking.
- Added a deterministic shared evaluator with a resumable WAL-enabled SQLite cache.
- Implemented frozen-hardware controller tuning, nested hardware/controller search, fixed-RMSE
  selection, Pareto extraction, and six-step alternating optimization.
- Completed a six-hardware by nine-controller quick co-design run over three scenarios.
- Added a nine-point fixed-controller hardware sensitivity experiment showing capability-boundary,
  motor-mass, final-drive efficiency, tracking, and acceleration effects.
- Added optional battery discharge/charge power limits with traction limiting and regenerative/
  friction brake blending at the electrical-power boundary.
- Added a resettable lumped motor thermal state, loss integration, size-dependent heat capacity and
  cooling, and temperature-dependent torque derating.
- Implemented a four-cycle, 100 s autonomous mountain-shuttle scenario with repeated ±10% grades,
  station stops, terminal progress, stop accuracy, battery-power, thermal, and MPC constraints.
- Completed a 60-point mountain hardware/controller search. Conventional sizing selected
  $g=10.5,s_m=0.6$; closed-loop co-design selected $g=11.5,s_m=0.75$.
- Added a versioned seven-scenario generalization dataset with four training and three disjoint test
  missions varying grade, speed, cycles, payload, drag, temperature, battery limits, and seed.
- Implemented leakage-free hardware selection on training data with independent MPC re-optimization
  for every scenario before and after hardware is frozen.
- Completed 720 training and 90 held-out controller evaluations with a resumable JSON cache after
  rejecting three hardware candidates that violated the shared 120 km/h motor-speed requirement.

## Verification

- Created an isolated Python 3.11 virtual environment and installed MetaDrive 0.4.3 plus the
  development dependencies.
- `ruff check .`: passed.
- `pytest`: 47 tests passed.
- `codesign-smoke --core-only`: passed; nominal mass 1575.0 kg, net energy 2.675 Wh, and
  recovered energy 1.396 Wh for the deterministic four-point core exercise.
- `codesign-smoke --metadrive`: passed; MetaDrive initialized headlessly, downloaded and loaded
  its 0.4.3 assets, completed a 20-step episode, and reported 8.329 Wh net battery energy.
- Live actuator calibration: traction and regeneration gains were 1.000 across all eight force
  levels after selecting the correct MetaDrive varying-dynamics vehicle and signed-force path.
- Live steering validation: exact command handoff, correct and monotonic turn response, 0.387%
  maximum left/right curvature asymmetry, 2.83% maximum bicycle-reference error, and a 0.2 s
  sampled yaw-rate rise time.
- Live urban MPC validation: 0.232 m/s RMSE, 44.09 Wh, 2.463 m/s² peak acceleration,
  3.500 m/s³ peak jerk, and zero fallbacks across 221 solves.
- Live curved-route MPC validation: 0.202 m/s RMSE, 48.47 Wh, 0.279 m lateral RMSE,
  3.500 m/s³ peak jerk, and zero fallbacks across 151 solves.
- MPC sweep: the default $(0,-1)$ gives 0.221 m/s aggregate RMSE and 93.07 Wh; $(-1,-1)$ improves
  both to 0.206 m/s and 92.85 Wh; $(1.5,-1)$ reduces energy 10.51% at 0.694 m/s RMSE.
- Blended braking: total-force error below 0.0023%; deterministic lead braking retains a 5.027 m
  minimum gap and 0.002 m minimum safe-gap margin with zero fallback.
- Deterministic urban profile with the temporary baseline: completed 44 s and 312.59 m; RMSE
  1.108 m/s, net energy 44.41 Wh, 142.08 Wh/km, peak acceleration 1.986 m/s², peak jerk
  2.052 m/s³, and zero powertrain saturation.
- Deterministic traffic test: triggered traffic spawned under seed 7 and the nearest lead vehicle
  was detected with gap and speed measurements.
- Curved-track PID validation from a 1.0 m initial offset: completed 320.70 m with 0.307 m lateral
  RMSE and 1.0 m maximum absolute error, remaining inside the ±1.75 m lane boundaries.
- Urban stop-go PID validation from a 0.5 m initial offset: completed 322.84 m with 0.120 m lateral
  RMSE and 0.500 m maximum absolute error.
- Energy reconstruction residual was 0 Wh and driveline power residual stayed below
  1.1×10⁻¹¹ W in both live scenarios.
- `mkdocs build --strict`: passed; browser QA confirmed nested navigation, rendered Mermaid/math,
  validation assets, detailed-page links, generated search index, and a 390 px responsive layout
  without horizontal overflow.
- Conventional sizing: 97 of 117 candidates feasible; selected final drive 10.5 and motor scale
  0.6, with 9.44 s 0–100 km/h and all top-speed/gradeability checks passing.
- Quick co-design: 54 unique cached evaluations; 0.1 and 0.2 m/s RMSE bounds infeasible, no gain at
  0.4/0.8 m/s, and a preliminary 0.023% saving at the loose 1.5 m/s bound.
- Mountain-shuttle quick search: 40 of 60 hardware/controller samples feasible. Co-design reduced
  energy from 241.40 to 211.03 Wh (12.58%) while RMSE remained effectively unchanged (0.4182 versus
  0.4177 m/s), station distance error stayed below 0.12 m, and no MPC fallback occurred.
- The mountain co-design reduced friction-brake dissipation from 60.21 to 28.02 Wh and increased
  recovered battery energy from 181.02 to 212.99 Wh. Across the hardware grid, best feasible energy
  ranged from 314.7 to 211.0 Wh.
- Corrected generalization training uses a 15-point two-parameter MPC grid and RMSE ≤0.4 m/s.
  Training selected $g=11.5,s_m=0.75$ using only four training scenarios; mean energy was
  274.79 Wh/km versus 311.15 Wh/km for conventional $g=10.5,s_m=0.6$.
- With hardware frozen and MPC re-tuned independently on every held-out scenario, selected hardware
  reduced mean test energy from 344.25 to 312.50 Wh/km (9.22%), won all three test cases, and kept
  all RMSE values below the 0.4 m/s threshold without fallback or mission violations.

## Important limitations

- The included efficiency map is an illustrative synthetic map; a traceable published or measured
  motor map is still required before final experiments.
- MetaDrive 0.4.3 imports slowly on this macOS environment and emits duplicate SDL-class warnings
  because both Pygame and OpenCV bundle SDL. The headless smoke episode still completes normally.
- Full-resolution generalization search, physical traffic-actor validation, broader multi-seed
  robustness experiments, parallel workers, and CARLA export are not implemented yet.

## Optimization boundary

The reduced-grid idea now generalizes from four training missions to three held-out missions with
scenario-specific controller adaptation. The next boundary is a refined hardware grid and broader
unseen traffic, curvature, friction, seed, and parameter distributions.

## Next steps

1. Refine the ratio grid around the top-speed-feasible boundary solution $g=11.5,s_m=0.75$.
2. Expand the dataset with unseen seeds, traffic, curvature, tire friction, and stochastic parameters.
3. Replace the illustrative efficiency and thermal parameters with traceable motor data.
4. Add a rendered, physically controlled MetaDrive traffic-actor safety scenario.
5. Export selected designs for CARLA validation on Windows.
