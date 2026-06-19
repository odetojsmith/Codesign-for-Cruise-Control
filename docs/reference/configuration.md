# Configuration reference

The default experiment configuration is stored in
[`configs/default.yaml`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/configs/default.yaml).

## Hardware

| Key | Default | Unit |
|---|---:|---|
| `hardware.final_drive_ratio` | 9.0 | dimensionless |
| `hardware.motor_scale` | 1.0 | relative scale |

## Vehicle

| Key | Default | Unit |
|---|---:|---|
| `vehicle.base_mass_kg` | 1500 | kg |
| `vehicle.wheel_radius_m` | 0.31 | m |
| `vehicle.final_drive_efficiency` | 0.97 | fraction |

The physical MetaDrive chassis mass is base mass plus scaled motor mass.

## Motor

| Key | Default | Unit |
|---|---:|---|
| `motor.base_peak_torque_nm` | 300 | N·m |
| `motor.base_peak_power_kw` | 150 | kW |
| `motor.max_speed_rpm` | 12,000 | rpm |
| `motor.base_mass_kg` | 75 | kg |
| `motor.regenerative_torque_fraction` | 0.60 | fraction |

## Battery

| Key | Default | Unit |
|---|---:|---|
| `battery.capacity_kwh` | 60 | kWh |
| `battery.inverter_efficiency` | 0.97 | fraction |
| `battery.auxiliary_power_w` | 500 | W |

## Simulation

| Key | Default | Unit |
|---|---:|---|
| `simulation.control_interval_s` | 0.2 | s |
| `simulation.seed` | 7 | integer |
| `simulation.use_render` | false | boolean |

## Validation rules

Configuration dataclasses are frozen and validate positivity, efficiency ranges, regeneration
fraction, auxiliary power, and control interval when loaded.

Implementation: [`config.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/src/codesign/config.py).

