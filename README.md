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

Install the MetaDrive backend with:

```bash
python -m pip install -e '.[simulation,dev]'
codesign-smoke --metadrive
```

MetaDrive may download its rendering assets on first use. Optimization will run headlessly;
rendering is reserved for selected comparison episodes.

## Current state

See [`project_status.md`](project_status.md) for implemented features, verification results, and
next work.

