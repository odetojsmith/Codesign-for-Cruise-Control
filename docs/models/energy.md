# EV energy model

## Purpose

MetaDrive supplies vehicle motion but does not calculate the motor/inverter/battery energy needed
for this co-design study. The project-owned energy layer converts feasible motor operation into
battery power and integrates it into Wh.

## Motor efficiency map

The map is indexed by normalized motor speed and absolute normalized torque. Bilinear interpolation
is implemented without a SciPy runtime dependency.

$$
\bar\omega=\frac{\omega_m}{\omega_{\max}},\qquad
\bar T=\frac{|T_m|}{T_{\max}}.
$$

The current map is smooth and illustrative. Its purpose is software validation, not real-world
motor certification.

!!! warning "Absolute accuracy limitation"
    Numerical consistency is verified, but final energy conclusions require a traceable measured
    or published motor-efficiency map.

## Battery power

For motoring,

$$
P_{\mathrm{batt}}
=\frac{T_m\omega_m}{\eta_m\eta_{\mathrm{inv}}}+P_{\mathrm{aux}}.
$$

For regeneration,

$$
P_{\mathrm{batt}}
=T_m\omega_m\eta_m\eta_{\mathrm{inv}}+P_{\mathrm{aux}},
$$

where mechanical power is negative during regeneration.

| Parameter | Value |
|---|---:|
| Inverter efficiency | 0.97 |
| Auxiliary power | 500 W |
| Battery capacity | 60 kWh |

## Integrated quantities

`EnergyState` accumulates:

- gross positive traction energy;
- recovered regenerative energy;
- auxiliary energy;
- net battery energy.

For a fixed interval $\Delta t$,

$$
E_{\mathrm{net,Wh}} \mathrel{+}=
P_{\mathrm{batt}}\frac{\Delta t}{3600}.
$$

State of charge is

$$
\mathrm{SOC}=\mathrm{SOC}_0-
\frac{E_{\mathrm{net,Wh}}}{1000C_{\mathrm{batt,kWh}}}.
$$

## Energy-flow checks

The validation suite independently reconstructs:

1. wheel power from applied force and actuator speed;
2. motor mechanical power using driveline efficiency;
3. battery power using motor/inverter efficiency and auxiliary load;
4. integrated battery energy from the reconstructed power history.

Current live scenarios close with 0 Wh reported integration residual and less than
$1.1\times10^{-11}$ W driveline residual.

## Implementation and tests

- Efficiency map: [`efficiency.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/src/codesign/efficiency.py)
- Energy integration: [`powertrain.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/src/codesign/powertrain.py)
- Visual checks: [`validation_cli.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/src/codesign/validation_cli.py)
- Tests: [`test_efficiency.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/tests/test_efficiency.py) and [`test_powertrain.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/tests/test_powertrain.py)

## Excluded physics

- battery voltage sag and internal resistance;
- battery and motor temperature;
- degradation and cycle aging;
- inverter switching behavior;
- motor thermal derating.

