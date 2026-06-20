# Hardware design

## Decision variables

The outer-loop hardware design is

$$
h=(g,s_m),
$$

where $g$ is final-drive ratio and $s_m$ scales motor torque, power, and mass.

| Variable | Default | Design range | Meaning |
|---|---:|---:|---|
| $g$ | 9.0 | 6–12 | Motor speed divided by wheel speed |
| $s_m$ | 1.0 | 0.6–1.4 | Scale relative to baseline motor |

## Baseline motor

| Property | Value |
|---|---:|
| Peak torque | 300 N·m |
| Peak power | 150 kW |
| Maximum speed | 12,000 rpm |
| Mass | 75 kg |

The scaled properties are

$$
T_{\max}=s_mT_{\max,0},\qquad
P_{\max}=s_mP_{\max,0},\qquad
m_m=s_mm_{m,0}.
$$

Maximum motor speed remains fixed. The normalized efficiency-map shape is preserved.

## Final-drive conversion

$$
\omega_m=g\frac{v}{r_w}.
$$

For motoring,

$$
T_m=\frac{F_wr_w}{\eta_g g}.
$$

For regeneration, the power flow reverses and driveline efficiency is applied in the reverse
direction.

Defaults:

- wheel radius $r_w=0.31$ m;
- final-drive efficiency $\eta_g=0.97$.

## How hardware changes behavior

### Increasing final-drive ratio

- increases low-speed wheel force for the same motor torque;
- increases motor speed at a given vehicle speed;
- reaches motor maximum speed at a lower vehicle speed;
- moves operating points across the efficiency map.

### Increasing motor scale

- raises torque and power limits;
- raises regenerative capability;
- reduces actuator saturation;
- adds vehicle mass and therefore acceleration energy.

The optimum is coupled: a large ratio can compensate for a smaller motor at low speed but can
restrict high-speed operation.

## Limit enforcement

At each step, allowable torque is

$$
T_{\mathrm{allow}}(\omega_m)=
\min\left(T_{\max},\frac{P_{\max}}{\max(\omega_m,\delta)}\right).
$$

Regenerative torque is additionally multiplied by the configured regeneration fraction, currently
0.60. Traction is set to zero beyond maximum motor speed. Negative force is suppressed near zero
vehicle speed to prevent unintended reverse motion.

## Road-load model for sizing

The backward-facing conventional baseline adds rolling, aerodynamic, and grade resistance:

$$
F_{road}=mgC_{rr}\cos\alpha
+\frac{1}{2}\rho C_d A v^2
+mg\sin\alpha,
\qquad \alpha=\tan^{-1}(q),
$$

where $q$ is signed road grade. Required wheel force is $ma+F_{road}$. The full hardware grid is
then filtered by 120 km/h operation, 0–100 km/h in 10 s, 20% gradeability at 30 km/h, and exact
drive-cycle feasibility before energy ranking.

For the complete candidate algorithm, backward-cycle equations, tie-breaking rule, and numerical
trace leading to $g=10.5,s_m=0.6$, see
[Conventional separate design](../optimization/separate-design.md#stage-1-hardware-sizing).

## Implementation and tests

- Implementation: [`config.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/src/codesign/config.py) and [`powertrain.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/src/codesign/powertrain.py)
- Tests: [`test_powertrain.py`](https://github.com/odetojsmith/Codesign-for-Cruise-Control/blob/main/tests/test_powertrain.py)
- Evidence: [Actuator validation](../validation/evidence.md)

Tests verify ratio effects, motor scaling, power-limited torque, overspeed behavior, regeneration,
and reverse prevention.
