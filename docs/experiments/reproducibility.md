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
python -m codesign.scenario_cli --profile urban
python -m codesign.validation_cli
```

## Determinism contract

- Python configuration is loaded from `configs/default.yaml`.
- Scenario seed defaults to 7.
- Traffic randomization is disabled.
- Hardware and controller parameters are explicit records.
- Reference profiles are immutable.
- Requested and applied actions are both logged.
- Validation acceptance thresholds are encoded in the command.

## Generated artifacts

Generated results live under `artifacts/` by default and are ignored by Git. This avoids mixing
machine-specific outputs with source. The documentation contains selected, reviewed validation
assets for human inspection.

## Future optimization record

Every optimization evaluation will persist:

- hardware candidate;
- controller candidate;
- scenario and seed;
- dependency versions;
- feasibility and constraint violations;
- aggregate metrics;
- trajectory location;
- cache key and completion status.

This allows interruption-safe resumption and exact reconstruction of Pareto points.

