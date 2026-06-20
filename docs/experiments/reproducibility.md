# Reproducibility

## Environment setup

```bash
python3.11 -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[simulation,optimization,visualization,dev]'
```

MetaDrive downloads its versioned rendering assets on first use.

## Verification commands

```bash
ruff check .
pytest
python -m codesign.smoke --core-only
python -m codesign.smoke --metadrive
python -m codesign.calibration
python -m codesign.steering_validation
python -m codesign.scenario_cli --profile urban
python -m codesign.validation_cli
python -m codesign.mpc_cli
python -m codesign.mpc_sweep
python -m codesign.braking_validation
codesign-size-hardware
codesign-separate-opt --quick
codesign-optimize --quick
```

## Determinism contract

- Python configuration is loaded from `configs/default.yaml`.
- Scenario seed defaults to 7.
- Traffic randomization is disabled.
- Hardware and controller parameters are explicit records.
- Reference profiles are immutable.
- Requested and applied actions are both logged.
- Validation acceptance thresholds are encoded in the command.
- Steering validation uses an 8 m/s constant-speed condition and fixed open-loop commands.

## Generated artifacts

Generated results live under `artifacts/` by default and are ignored by Git. This avoids mixing
machine-specific outputs with source. The documentation contains selected, reviewed validation
assets for human inspection.

## Optimization record

Every closed-loop optimization evaluation persists in a WAL-enabled SQLite database:

- hardware candidate;
- controller candidate;
- scenario and seed;
- feasibility and constraint violations;
- aggregate metrics;
- cache key and completion status.

The cache key hashes hardware, controller, vehicle/motor/battery configuration, scenario profiles,
seed, control interval, and constraint thresholds. Selected designs can be replayed later to write
full trajectories; keeping every trajectory inside the optimization cache would make the database
unnecessarily large.
