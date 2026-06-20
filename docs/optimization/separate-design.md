# Conventional separate design

!!! success "Implemented"
    Backward-facing hardware sizing, persistent evaluation caching, and frozen-hardware MPC tuning
    are implemented. The current evidence uses the quick controller grid; the full grid remains to
    be run.

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

## Outputs

- selected conventional hardware;
- hardware-feasibility map;
- backward-facing Wh/km table;
- best controller at each RMSE bound;
- separate-design Pareto points;
- validation trajectories.

## Current quick result

The 117-point hardware grid has 97 feasible candidates. The conventional rule selects
$g=10.5$ and $s_m=0.6$:

| Quantity | Result |
|---|---:|
| Motor mass | 45.0 kg |
| Backward-cycle energy | 208.18 Wh/km |
| 0–100 km/h time | 9.44 s |
| 120 km/h requirement | Pass |
| 20% grade at 30 km/h | Pass |

![Conventional hardware sizing map](../assets/validation/hardware_sizing_map.png)

The quick closed-loop controller sweep finds no feasible point at 0.1 or 0.2 m/s aggregate RMSE.
At the 0.4 and 0.8 m/s bounds, the selected controller is $(0,0)$ with 0.268 m/s RMSE and
162.69 Wh across the shared urban, highway, and grade training episodes.

```bash
codesign-size-hardware
codesign-separate-opt --quick
```
