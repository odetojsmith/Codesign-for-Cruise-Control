# Evidence and conclusions

This page separates measured results from interpretation. Detailed equations, plots, and raw-file
locations remain on the linked pages.

## Evidence ladder

| Level | Question | Current evidence | Status |
|---|---|---|---|
| 1. Actuation | Does the requested input reach the simulated vehicle? | Traction/regeneration gain 1.000; steering sign, symmetry, scale, and transient checks | Established in MetaDrive |
| 2. Energy | Are force, power, regeneration, and Wh internally consistent? | 0 Wh reconstruction residual; driveline residual below $1.1\times10^{-11}$ W | Established in the software model |
| 3. Control | Can the car track speed and centerline safely? | PID and MPC urban/curved runs; braking-gap and zero-fallback checks | Established for deterministic scenarios |
| 4. Hardware relevance | Do $g$ and $s_m$ materially affect outcomes? | Capability-boundary sweep and repeated mountain mission | Established for constrained missions |
| 5. Co-design benefit | Can closed-loop selection beat traditional sizing? | 12.58% mountain energy reduction at similar RMSE | Supported in the mountain case |
| 6. Generalization | Does training-selected hardware work on unseen missions? | 9.22% mean held-out energy reduction over three test scenarios | Supported on the current split |
| 7. Matched performance | Is the gain more than accepting worse tracking? | 7.12–9.60% energy reduction with lower RMSE in all three held-out cases | Supported at sampled controller resolution |

## Hardware definitions used in comparisons

| Role | Final drive $g$ | Motor scale $s_m$ | How selected |
|---|---:|---:|---|
| Traditional | 10.5 | 0.60 | Vehicle capability and backward exact-speed cycle; no closed-loop RMSE |
| Training-selected | 11.5 | 0.75 | Minimum mean training Wh/km after per-scenario controller tuning and constraints |

## Main numerical results

| Experiment | Traditional | Co-designed/trained | Interpretation |
|---|---:|---:|---|
| Mountain shuttle | 241.40 Wh, 0.4182 m/s RMSE | 211.03 Wh, 0.4177 m/s RMSE | 12.58% less energy at essentially unchanged tracking |
| Mean training energy | 311.15 Wh/km | 274.79 Wh/km | 11.68% reduction |
| Mean held-out energy | 344.25 Wh/km | 312.50 Wh/km | 9.22% reduction |
| Matched-RMSE held-out mean | 336.04 Wh/km | 308.35 Wh/km | 8.24% reduction with lower RMSE in every test case |
| Dense shared-controller point | 311.15 Wh/km, 0.35378 m/s | 276.09 Wh/km, 0.33446 m/s | 11.27% less energy and lower mean RMSE |

## Pareto-plot reading rule

Every plotted sample is an evaluated controller or hardware design. Orange diamonds mark sampled
nondominated points. They are not connected because a line would suggest untested interpolation
between discrete designs.

The dense trained-hardware controller sweep contains 40 weight pairs evaluated on four training
scenarios, for 160 closed-loop runs. Seven samples are nondominated; three also satisfy RMSE
≤0.4 m/s in every scenario.

![Dense trained-hardware controller Pareto samples](../assets/validation/trained_hardware_controller_pareto.png)

## Why the conclusion is credible

- Traditional hardware is selected without access to closed-loop control metrics.
- Training and test missions are disjoint.
- Hardware is frozen before test evaluation.
- Both hardware designs may re-tune control after freezing.
- Energy is minimized only after tracking and mission constraints are applied.
- A stricter matched-RMSE test requires trained hardware to beat the RMSE achieved by traditional
  hardware, rather than merely satisfy a loose common threshold.
- Raw evaluations and controller choices are cached and reproducible.

## Important interpretation nuance

The dense 40-controller plot uses one shared controller weight pair across four training scenarios,
while its traditional reference point came from scenario-specific controller tuning. This is a
conservative reference for the trained hardware, but it is not a complete same-protocol comparison
of two controller frontiers. Producing matched-protocol frontiers for both hardware designs is the
highest-priority next experiment.

## Supported claim

Within this software model and scenario family, controller-aware hardware selection produces a
meaningful energy benefit that is not explained only by accepting larger tracking error.

## Claims not yet supported

- global optimality over continuous hardware and controller spaces;
- robustness across broad traffic, curvature, friction, weather, and seed distributions;
- quantitatively accurate production-motor energy or thermal behavior;
- transfer of rankings from MetaDrive to CARLA or a physical vehicle.

