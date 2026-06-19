# Project Status

Last updated: 2026-06-19

## Current milestone

Implementation has started with the first runnable vertical slice: project packaging, validated
configuration records, EV powertrain limits, electrical-energy accounting, and a thin optional
MetaDrive adapter.

## Completed

- Established Python package, dependency groups, default configuration, and test layout.
- Added immutable hardware, vehicle, motor, battery, and project configuration records.
- Implemented final-drive conversion, motor scaling, torque/power/speed saturation, regenerative
  braking, motor-efficiency interpolation, auxiliary load, battery energy, and state of charge.
- Added a MetaDrive wrapper that applies hardware-dependent force limits and records EV energy.
- Added core and optional MetaDrive smoke-test commands.
- Added initial unit tests for configuration, interpolation, hardware effects, saturation, and
  energy accounting.

## Verification

- Created an isolated Python 3.11 virtual environment and installed MetaDrive 0.4.3 plus the
  development dependencies.
- `ruff check .`: passed.
- `pytest`: 13 tests passed.
- `codesign-smoke --core-only`: passed; nominal mass 1575.0 kg, net energy 2.675 Wh, and
  recovered energy 1.396 Wh for the deterministic four-point core exercise.
- `codesign-smoke --metadrive`: passed; MetaDrive initialized headlessly, downloaded and loaded
  its 0.4.3 assets, completed a 20-step episode, and reported 8.329 Wh net battery energy.

## Important limitations

- The included efficiency map is an illustrative synthetic map; a traceable published or measured
  motor map is still required before final experiments.
- MetaDrive action-to-force calibration must be checked against version 0.4.3 in a live episode.
- MetaDrive 0.4.3 imports slowly on this macOS environment and emits duplicate SDL-class warnings
  because both Pygame and OpenCV bundle SDL. The headless smoke episode still completes normally.
- Longitudinal MPC, drive-cycle scenarios, conventional sizing, co-design optimization, plots, and
  CARLA export are not implemented yet.

## Next steps

1. Calibrate the MetaDrive actuator wrapper against measured acceleration and braking response.
2. Add deterministic speed-profile and lead-vehicle scenarios with trajectory logging.
3. Implement the constrained longitudinal MPC and closed-loop metrics.
4. Implement conventional hardware sizing and nested/alternating co-design.
