# Project Status

Last updated: 2026-06-19

## Current milestone

The simulator, hardware-dependent actuator, energy layer, deterministic reference profiles,
trajectory logging, controller-independent metrics, heuristic PID controllers, and visual
validation suite are implemented and verified. A searchable hierarchical MkDocs website now maps
the complete architecture, implementation, tests, evidence, and planned work. Development remains
at the longitudinal MPC design boundary.

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
- Added a reproducible visual-validation command that generates an actuator plot, speed/lateral/
  energy dashboard, top-down GIF and PNG, trajectories, and a machine-readable pass/fail report.
- Added a Material for MkDocs website with 17 hierarchical pages, Mermaid architecture/data-flow
  diagrams, equations, implementation links, source-file inventory, and requirement traceability.
- Added core and optional MetaDrive smoke-test commands.
- Added initial unit tests for configuration, interpolation, hardware effects, saturation, and
  energy accounting.

## Verification

- Created an isolated Python 3.11 virtual environment and installed MetaDrive 0.4.3 plus the
  development dependencies.
- `ruff check .`: passed.
- `pytest`: 19 tests passed.
- `codesign-smoke --core-only`: passed; nominal mass 1575.0 kg, net energy 2.675 Wh, and
  recovered energy 1.396 Wh for the deterministic four-point core exercise.
- `codesign-smoke --metadrive`: passed; MetaDrive initialized headlessly, downloaded and loaded
  its 0.4.3 assets, completed a 20-step episode, and reported 8.329 Wh net battery energy.
- Live actuator calibration: traction and regeneration gains were 1.000 across all eight force
  levels after selecting the correct MetaDrive varying-dynamics vehicle and signed-force path.
- Deterministic urban profile with the temporary baseline: completed 44 s and 312.59 m; RMSE
  1.108 m/s, net energy 44.41 Wh, 142.08 Wh/km, peak acceleration 1.986 m/s², peak jerk
  2.052 m/s³, and zero powertrain saturation.
- Deterministic traffic test: triggered traffic spawned under seed 7 and the nearest lead vehicle
  was detected with gap and speed measurements.
- Curved-track PID validation from a 1.0 m initial offset: completed 318.19 m with 0.377 m lateral
  RMSE and 1.0 m maximum absolute error, remaining inside the ±1.75 m lane boundaries.
- Urban stop-go PID validation from a 0.5 m initial offset: completed 321.96 m with 0.162 m lateral
  RMSE and 0.597 m maximum absolute error.
- Energy reconstruction residual was 0 Wh and driveline power residual stayed below
  1.1×10⁻¹¹ W in both live scenarios.
- `mkdocs build --strict`: passed; browser QA confirmed nested navigation, rendered Mermaid/math,
  validation assets, detailed-page links, generated search index, and a 390 px responsive layout
  without horizontal overflow.

## Important limitations

- The included efficiency map is an illustrative synthetic map; a traceable published or measured
  motor map is still required before final experiments.
- MetaDrive 0.4.3 imports slowly on this macOS environment and emits duplicate SDL-class warnings
  because both Pygame and OpenCV bundle SDL. The headless smoke episode still completes normally.
- Longitudinal MPC, grade-enabled drive cycles, conventional sizing, co-design optimization,
  plots, and CARLA export are not implemented yet.

## MPC design boundary

The next implementation step requires locking the MPC formulation: prediction state, longitudinal
model, lead-vehicle prediction, safe-gap constraint, energy surrogate, force/slew constraints,
horizon, terminal treatment, and infeasibility fallback. No MPC choices have been silently made.

## Next steps

1. Agree on the longitudinal MPC formulation and safety policy.
2. Implement and test MPC against the existing controller protocol and scenario runner.
3. Add the grade-enabled drive-cycle scenarios.
4. Implement conventional hardware sizing and nested/alternating co-design.
