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
| `vehicle.maximum_friction_braking_acceleration_mps2` | 6.0 | m/s² |
| `vehicle.rolling_resistance_coefficient` | 0.012 | fraction |
| `vehicle.drag_coefficient` | 0.29 | dimensionless |
| `vehicle.frontal_area_m2` | 2.3 | m² |
| `vehicle.air_density_kg_per_m3` | 1.225 | kg/m³ |

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
| `battery.maximum_discharge_power_kw` | null | kW |
| `battery.maximum_charge_power_kw` | null | kW |

Null power limits preserve the original unconstrained behavior. The mountain-shuttle experiment
overrides them with 90 kW discharge and 45 kW charge limits.

## Motor thermal model

| Key | Default | Unit |
|---|---:|---|
| `thermal.enabled` | false | boolean |
| `thermal.ambient_temperature_c` | 25 | °C |
| `thermal.initial_temperature_c` | 25 | °C |
| `thermal.base_thermal_capacity_j_per_k` | 35,000 | J/K |
| `thermal.base_thermal_conductance_w_per_k` | 120 | W/K |
| `thermal.derating_start_temperature_c` | 90 | °C |
| `thermal.maximum_temperature_c` | 120 | °C |
| `thermal.minimum_torque_fraction` | 0.25 | fraction |

## Simulation

| Key | Default | Unit |
|---|---:|---|
| `simulation.control_interval_s` | 0.2 | s |
| `simulation.seed` | 7 | integer |
| `simulation.use_render` | false | boolean |

## Validation rules

Configuration dataclasses are frozen and validate positivity, efficiency ranges, regeneration
fraction, auxiliary power, battery power limits, thermal ordering, and control interval when loaded.

Implementation: [`config.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/src/codesign/config.py).
