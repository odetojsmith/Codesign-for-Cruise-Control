# Conventional separate design

!!! note "Planned"
    This page specifies the baseline workflow; the optimization code is not implemented yet.

## Purpose

Represent a conventional engineering sequence in which hardware is sized from performance and
drive-cycle requirements without closed-loop tracking metrics, then the controller is tuned after
hardware is frozen.

## Stage 1 — hardware sizing

Use backward-facing drive-cycle analysis with exact prescribed-speed following. For each hardware
candidate $(g,s_m)$:

1. Compute required wheel force from prescribed speed, drag, rolling resistance, and grade.
2. Convert wheel operation to motor torque and speed.
3. Reject torque, power, or speed violations.
4. Check vehicle-level requirements.
5. Calculate drive-cycle Wh/km from the same efficiency map used elsewhere.

Requirements:

| Requirement | Threshold |
|---|---:|
| Maximum speed | At least 120 km/h |
| 0–100 km/h acceleration | At most 10 s |
| Gradeability | 20% at 30 km/h |

Choose minimum-Wh/km feasible hardware. If candidates are within 0.5%, choose lower motor mass.

No feedback-controller tracking error is used in this stage.

## Stage 2 — controller tuning

Freeze selected hardware. For every externally specified tracking limit $\epsilon$, tune controller
parameters to solve

$$
\min_\theta E_{\mathrm{net}}(h_{\mathrm{sep}},\theta)
\quad\text{subject to}\quad
\operatorname{RMSE}_v\leq\epsilon.
$$

## Final comparison

Evaluate separate design and co-design with identical scenarios, seeds, constraints, and metrics.
The claim is only supported where co-design uses less energy at the same tracking bound without
additional safety or comfort violations.

## Planned outputs

- selected conventional hardware;
- hardware-feasibility map;
- backward-facing Wh/km table;
- best controller at each RMSE bound;
- separate-design Pareto points;
- validation trajectories.

