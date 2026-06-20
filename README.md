# Codesign for Cruise Control

An autonomous-EV illustration of hardware–controller co-design. The project uses MetaDrive for
roads, traffic, vehicle motion, and visualization, while a project-owned EV layer models the
final drive, motor limits, efficiency, regenerative braking, and battery energy.

The comparison is made at matched speed-tracking RMSE limits: minimize energy subject to a fixed
tracking requirement. See [`PLAN.md`](PLAN.md) for the complete experiment design.

## Development setup

```bash
python3.11 -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[dev]'
pytest
codesign-smoke --core-only
```

Run the implemented longitudinal MPC against PID and curved-road scenarios:

```bash
codesign-mpc-validate
```

Sample MPC weights and validate braking with:

```bash
codesign-mpc-sweep
codesign-braking-validate
codesign-trajectory-animation
codesign-scenario-gifs
```

Run conventional hardware sizing and the resumable quick co-design experiment with:

```bash
codesign-size-hardware
codesign-separate-opt --quick
codesign-optimize --quick
codesign-hardware-sensitivity
codesign-mountain-shuttle --quick
codesign-generality-dataset --quick
codesign-matched-rmse-test
```

The full `codesign-optimize` grid is intentionally much larger. Evaluations are cached in SQLite,
so rerunning the command resumes from completed hardware/controller pairs.

Install the MetaDrive backend with:

```bash
python -m pip install -e '.[simulation,dev]'
codesign-smoke --metadrive
```

MetaDrive may download its rendering assets on first use. Optimization will run headlessly;
rendering is reserved for selected comparison episodes.

## Pre-MPC simulator verification

Calibrate the signed wheel-force actuator against MetaDrive's measured acceleration:

```bash
python -m codesign.calibration
```

Run the deterministic urban profile with the temporary proportional baseline:

```bash
python -m codesign.scenario_cli --profile urban
```

The generated calibration, metrics, and trajectory files are written under `artifacts/` and are
intentionally ignored by Git.

Generate the complete longitudinal-force, open-loop steering, energy, PID, and visual validation
evidence with:

```bash
python -m codesign.validation_cli
```

The command fails if force calibration exceeds 0.1% error, steering handoff or response checks fail,
either driving episode terminates early, the vehicle leaves its lane, or the independent
driveline/energy balances do not close.

## Current state

See [`project_status.md`](project_status.md) for implemented features, verification results, and
next work.

## Hierarchical documentation website

Install the development dependencies and run the local documentation site:

```bash
python -m pip install -e '.[dev]'
mkdocs serve
```

Open the displayed local URL to browse architecture, physical models, simulation, controllers,
experiments, optimization, validation, configuration, source files, and requirement traceability.

Build ordinary static HTML with strict link checking:

```bash
mkdocs build --strict
```

The generated website is written to `site/`. Documentation source lives under `docs/`; navigation
and theme settings live in `mkdocs.yml`.
