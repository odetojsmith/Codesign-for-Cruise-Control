# MetaDrive integration

## Why MetaDrive

MetaDrive provides a mature, lightweight autonomous-driving environment that runs on the current
macOS development machine. It owns roads, lanes, traffic, collisions, rigid-body motion, sensors,
top-down visualization, and 3D visualization.

CARLA remains the later validation target on the Windows RTX 5060 Ti machine.

## Adapter contract

`MetaDriveEVEnv` exposes a project-specific action:

```text
(normalized steering, requested wheel force in newtons)
```

The adapter performs these operations:

1. Reads current vehicle speed.
2. Evaluates the EV powertrain at the requested force.
3. Clips force using torque, power, speed, and regeneration limits.
4. Converts feasible force to MetaDrive's signed engine action.
5. Advances ten physics steps for one control interval.
6. Returns vehicle state plus battery power, net energy, and saturation status.

## Varying-dynamics vehicle

MetaDrive's default visual vehicle has a fixed physical chassis mass. The project therefore uses
`VaryingDynamicsEnv` and fixes its random-dynamics ranges to one hardware candidate:

- chassis mass equals base vehicle mass plus scaled motor mass;
- engine force equals candidate maximum wheel force divided across four wheels;
- signed negative engine force represents regeneration;
- wheel friction and maximum steering are deterministic.

This choice was validated by comparing requested force with measured $ma$.

## Lane and traffic observations

The adapter provides:

- speed and planar position;
- lateral error from the current reference lane;
- heading error relative to lane direction;
- lane width;
- nearest lead-vehicle gap and speed;
- top-down rendered frames.

Lead vehicles are selected only when ahead of the ego vehicle and within the lateral lane corridor.

## Deterministic settings

| Setting | Default |
|---|---:|
| Scenario seed | 7 |
| Control interval | 0.2 s |
| Physics interval | 0.02 s |
| Traffic randomization | Disabled |
| Rendering during optimization | Disabled |
| Initial map | Straight block sequence |

## Known platform behavior

MetaDrive 0.4.3 works headlessly on this macOS machine. It imports slowly and emits duplicate SDL
class warnings because both Pygame and OpenCV bundle SDL. The warning has not affected completed
headless or top-down validation runs.

## Implementation and verification

- Adapter: [`metadrive_env.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/src/codesign/metadrive_env.py)
- Calibration: [`calibration.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/src/codesign/calibration.py)
- Smoke command: `python -m codesign.smoke --metadrive`
- Full visual validation: `python -m codesign.validation_cli`

