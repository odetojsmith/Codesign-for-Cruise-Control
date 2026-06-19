# MPC design boundary

!!! warning "Not implemented"
    No MPC structure or tuning decision has been silently committed. This page records the design
    decisions that must be locked before implementation.

## Intended role

The longitudinal MPC will replace the validation speed PID. It must minimize energy while
satisfying tracking, safety, comfort, and hardware constraints.

The external experiment remains

$$
\min E_{\mathrm{net}}
\quad \text{subject to} \quad
\operatorname{RMSE}_v\leq\epsilon.
$$

Internal MPC weights are controller parameters; they are not the final evaluation score.

## Decisions required

| Decision | Candidate default | Why it matters |
|---|---|---|
| State | speed, gap, relative speed, previous force | Defines prediction and safety capability |
| Input | wheel-force request | Matches existing actuator contract |
| Prediction model | longitudinal point mass with local drag/grade | Convexity versus fidelity |
| Horizon | 20 steps at 0.2 s | Four-second look-ahead |
| Lead prediction | constant velocity with bounded braking fallback | Safety robustness |
| Safe gap | standstill gap plus time headway | Converts traffic behavior into constraint |
| Energy surrogate | convex force/speed approximation | OSQP-compatible online optimization |
| Comfort | force-slew, acceleration, and jerk constraints | Passenger comfort and feasibility |
| Fallback | last feasible action, then bounded braking | Handles solver failure |
| Terminal treatment | terminal speed/gap penalty or terminal set | End-of-horizon behavior |

## Planned optimization variables

The plan currently proposes fixing tracking normalization and tuning two internal parameters:

$$
\theta=(\log_{10}\lambda_E,\log_{10}\lambda_{\Delta u}),
\qquad
\log_{10}\lambda_E,\log_{10}\lambda_{\Delta u}\in[-3,3].
$$

This design must be confirmed before coding.

## Acceptance criteria

- Satisfy the same controller protocol as PID.
- Respect hardware-dependent force limits without relying on downstream clipping in nominal runs.
- Report and recover from infeasible optimization problems.
- Meet fixed RMSE, safety-gap, acceleration, and jerk requirements.
- Produce deterministic results for fixed hardware, scenario, and seed.
- Outperform or clearly characterize the PID baseline without changing final metrics.

## Planned implementation stack

- CVXPY for problem construction;
- OSQP for the convex quadratic program;
- the existing MetaDrive adapter and scenario runner;
- Optuna for offline controller-parameter search.

