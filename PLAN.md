# Hardware–Controller Co-Design for an Autonomous Electric Vehicle

## 1. Project objective

Build a reproducible autonomous-driving example that demonstrates when integrated hardware–controller co-design produces a better energy–tracking tradeoff than the conventional sequential workflow.

The vehicle follows planner-provided speed profiles while respecting traffic and safety constraints. The hardware design selects the final-drive ratio and motor size. The controller design tunes a longitudinal model-predictive controller (MPC). Designs are compared at matched tracking requirements rather than through an arbitrary weighted score.

The primary claim will be:

> For the same speed-tracking RMSE limit and safety constraints, co-design can consume less battery energy than conventional separate hardware and controller optimization.

If the resulting Pareto fronts cross, the project will report the operating ranges where each method is preferable instead of claiming universal superiority.

## 2. Simulation architecture

### 2.1 MetaDrive optimization backend

Use [MetaDrive](https://metadrive-simulator.readthedocs.io/en/latest/) as the primary simulator. It supplies:

- Road networks, traffic actors, collisions, and scenario generation.
- Vehicle rigid-body and tire/contact simulation.
- Gym-compatible reset/step interfaces.
- Top-down and 3D rendering.
- Sensor/state access, recording, and replay.
- Fast headless execution for optimization.

Optimization runs will use headless mode. Only baseline runs, selected Pareto designs, and final comparison episodes will be rendered and recorded.

### 2.2 Project-owned EV powertrain and energy layer

MetaDrive does not natively model the complete electric power path needed for this study: motor efficiency maps, inverter/battery losses, regenerative-braking efficiency, state of charge, and electrical energy in Wh. The project will therefore add a compact EV powertrain layer around MetaDrive.

MetaDrive remains the source of vehicle motion and environment state. At every simulation step, the EV layer will:

1. Read vehicle speed, acceleration, road grade, and commanded traction/braking action.
2. Convert wheel speed and force to motor speed and torque using the candidate final-drive ratio.
3. Enforce motor torque, power, speed, and regenerative-braking limits for the candidate motor size.
4. Map feasible motor torque back to the throttle/brake command applied to MetaDrive.
5. Look up motoring or regeneration efficiency from a tabulated motor map.
6. Integrate battery power to obtain net energy consumption and state of charge.

This is not a second vehicle simulator. It is an actuator-limit and energy-accounting model coupled to MetaDrive's vehicle and environment simulation.

### 2.3 CARLA validation backend

After optimization is complete, validate only the conventional baseline and selected Pareto-optimal co-designs in CARLA on the Windows PC with the RTX 5060 Ti. CARLA validation will use its configurable `final_ratio`, torque curve, mass, drag, and vehicle-control APIs.

CARLA will not be placed inside the optimization loop because thousands of CARLA episodes would be unnecessarily expensive. The same speed profiles, hardware records, controller parameters, and evaluation metrics will be exported through a backend-neutral experiment configuration.

## 3. Models and design variables

### 3.1 Hardware design

Define

\[
h=(g,s_m),
\]

where:

- \(g\in[6,12]\) is the fixed motor-to-wheel speed ratio:
  \[
  g=\omega_m/\omega_w.
  \]
- \(s_m\in[0.6,1.4]\) is motor size relative to a baseline motor.

Use a baseline motor with 150 kW peak power, 300 Nm peak torque, 12,000 rpm maximum speed, and 75 kg mass. Scaling changes peak torque, peak power, and motor mass linearly while preserving the normalized efficiency-map shape. Maximum motor speed remains fixed.

Use

\[
\omega_m=g\frac{v}{r_w},\qquad
T_m=\frac{F_w r_w}{\eta_g g}.
\]

The larger motor's mass is added to the MetaDrive vehicle mass. The final-drive efficiency is fixed at 0.97 and wheel radius at 0.31 m.

### 3.2 EV energy model

Use a tabulated, interpolated motor-efficiency map indexed by normalized motor speed and torque. Keep motoring and regenerative efficiencies distinct.

For motoring,

\[
P_{batt}=\frac{T_m\omega_m}{\eta_m\eta_{inv}}+P_{aux}.
\]

For regeneration,

\[
P_{batt}=T_m\omega_m\eta_{regen}\eta_{inv}+P_{aux},
\]

where negative battery power represents recovered energy. Integrate battery power over time and report gross traction energy, recovered energy, net Wh, and Wh/km. Use inverter efficiency 0.97, auxiliary load 500 W, and a 60 kWh battery. State of charge is tracked. Battery voltage sag and degradation remain out of scope, but optional charge/discharge power limits and a lumped motor thermal/torque-derating state are implemented for demanding missions.

### 3.3 Controller design

Use a constrained longitudinal MPC with a 0.2 s control interval and a 20-step prediction horizon. MetaDrive handles lateral lane following with a fixed built-in or project-supplied lateral controller so lateral behavior is not optimized.

The MPC tracks the planner speed command and safe following gap while respecting hardware-dependent traction/braking limits. Normalize every internal objective term, fix the tracking coefficient to one, and tune only

\[
\theta=(\log_{10}\lambda_E,\log_{10}\lambda_{\Delta u}),
\qquad \log_{10}\lambda_E,\log_{10}\lambda_{\Delta u}\in[-3,3].
\]

These are internal controller parameters only. They do not define the final system score. The MPC will be implemented with CVXPY and OSQP, using a convex local energy surrogate; final energy is always calculated by the independent nonlinear EV energy layer.

## 4. Optimization and fair comparison

### 4.1 Final evaluation problem

For tracking limits

\[
\epsilon\in\{0.1,0.2,0.4,0.8\}\ \text{m/s},
\]

solve

\[
\begin{aligned}
\min_{h,\theta}\quad & E_{net}(h,\theta)\\
\text{s.t.}\quad
& \operatorname{RMSE}_v(h,\theta)\leq\epsilon,\\
& d_{min}\geq d_{safe},\\
& |a|\leq 3.0\ \text{m/s}^2,\\
& |j|\leq 4.0\ \text{m/s}^3,\\
& \text{motor, battery, and road constraints are satisfied.}
\end{aligned}
\]

Report RMSE and energy separately and compare methods at the same \(\epsilon\). Also plot the complete energy–RMSE Pareto fronts.

### 4.2 Conventional separate-design baseline

First select hardware without a feedback controller using a backward-facing drive-cycle calculation that assumes exact prescribed-speed following.

For each hardware candidate:

- Check 120 km/h top speed.
- Check 0–100 km/h acceleration within 10 s using maximum feasible torque.
- Check a 20% grade at 30 km/h.
- Reject candidates violating motor torque, power, or speed limits.
- Compute drive-cycle Wh/km from the same efficiency map.

Choose the feasible candidate with minimum Wh/km. If candidates are within 0.5%, choose the one with lower motor mass. Freeze that hardware, then tune the MPC to minimize closed-loop energy for each RMSE bound.

The implemented conventional grid contains 117 candidates and retains 97 after top-speed,
full-power acceleration, 20% gradeability, and exact backward-cycle checks. The minimum cycle
energy is 208.1829 Wh/km. Five $s_m=0.6$ designs lie within the 0.5% near-minimum set; because their
45 kg motor masses tie, energy selects $g=10.5,s_m=0.6$. The detailed equations and intermediate
force margins are maintained in `docs/optimization/separate-design.md`.

### 4.3 Integrated co-design

Sweep the hardware grid

- \(g=6,6.5,\ldots,12\).
- \(s_m=0.6,0.7,\ldots,1.4\).

For each hardware pair, use 60 seeded Optuna trials to tune the two MPC parameters. Cache every simulation result. For each RMSE limit, retain the minimum-energy feasible controller for that hardware and then select the minimum-energy hardware–controller pair globally.

### 4.4 Alternating optimization illustration

Initialize from the conventional hardware and nominal MPC. Then alternate:

1. Optimize controller parameters with hardware fixed.
2. Optimize hardware variables with controller fixed.

Stop after six iterations or when net-energy improvement is below 0.1%. Treat this as an algorithmic illustration, not proof of global optimality; compare its result with the nested hardware-grid result.

## 5. Scenarios and outputs

Optimize over three deterministic training scenarios:

- Urban stop-and-go, 0–50 km/h.
- Highway speed changes, 60–100 km/h.
- Mixed route with road grades from -6% to +6%.

Use a fourth, deliberately hardware-sensitive autonomous mountain-shuttle mission as the decisive
co-design demonstration. It repeats four 25 s station-to-station cycles over ±10% grades, with a
750 m progress requirement, station-stop accuracy, battery charge/discharge limits, and motor
thermal monitoring. Energy is minimized only among designs satisfying identical tracking,
progress, stopping, safety, and actuator constraints.

Validate on unseen mixed traffic, emergency lead-vehicle braking, payload variation of ±10%, drag variation of ±10%, and road-friction variation. Use fixed scenario seeds shared by every method.

Generate:

- Speed command and achieved-speed plots.
- Tracking-error, acceleration, jerk, and following-gap plots.
- Battery power and cumulative-energy plots.
- Motor operating points over the efficiency map.
- Hardware-space objective and feasibility heatmaps.
- Alternating-optimization convergence plots.
- Energy–RMSE Pareto fronts for separate design and co-design.
- MetaDrive top-down and 3D videos for selected designs.
- A CARLA validation table and videos after migration to Windows.

## 6. Software structure and interfaces

Use Python 3.11 with MetaDrive, NumPy, SciPy, Pandas, CVXPY, OSQP, Optuna, Gymnasium, Matplotlib, Plotly, PyYAML, and pytest.

Core immutable records:

- `HardwareDesign(final_drive_ratio, motor_scale)`.
- `ControllerDesign(log_energy_weight, log_slew_weight)`.
- `ScenarioConfig(name, seed, route, speed_profile, traffic_config)`.
- `SimulationResult(rmse_mps, gross_wh, regen_wh, net_wh, wh_per_km, minimum_gap_m, peak_accel_mps2, peak_jerk_mps3, violations, trajectories)`.

Provide commands to:

1. Verify MetaDrive and render a smoke-test episode.
2. Run conventional hardware sizing.
3. Run separate controller optimization.
4. Run integrated co-design.
5. Run alternating optimization.
6. Generate all figures, tables, and MetaDrive recordings.
7. Export selected designs and scenarios for CARLA.
8. Run CARLA validation on Windows.

Every experiment writes its configuration, dependency versions, random seeds, design variables, aggregate metrics, and trajectories to a timestamped result directory. Optimization resumes from cached evaluations after interruption.

## 7. Tests and acceptance criteria

Unit tests will verify:

- Final-drive speed/torque conversion.
- Motor scaling and added vehicle mass.
- Interpolation and bounds of the efficiency map.
- Motoring, auxiliary, and regenerative energy integration.
- Motor torque, power, speed, and regeneration saturation.
- Deterministic scenario generation and result caching.
- RMSE, Wh/km, gap, acceleration, and jerk metrics.

Integration tests will verify:

- MetaDrive can run headlessly and render one saved episode.
- Hardware changes alter feasible acceleration and maximum speed in the expected direction.
- MPC actions obey hardware-dependent limits.
- Infeasible designs are reported without crashing optimization.
- Separate and co-design methods use identical scenarios and evaluation functions.
- A selected experiment can be replayed from its saved configuration.

The initial demonstration succeeds when at least one practically relevant RMSE bound shows lower
Wh/km for co-design without additional safety or comfort violations. The mountain-shuttle quick
grid has met this software-in-the-loop milestone with a 12.58% energy reduction at matched RMSE.
The stronger validation claim still requires qualitatively consistent results across unseen
scenarios and at least five scenario-seed sets. If those conditions are not met, report the
non-dominating Pareto fronts without forcing the intended conclusion.

## 8. Implementation order

1. Create the environment, dependency lock, configuration schema, and smoke test.
2. Integrate MetaDrive scenarios, state extraction, rendering, and replay.
3. Implement and test the EV powertrain and energy layer.
4. Implement the hardware-dependent actuator wrapper.
5. Implement and test longitudinal MPC.
6. Implement conventional hardware sizing and separate controller tuning.
7. Implement nested and alternating co-design with caching and parallel workers.
8. Run training and unseen validation scenarios; generate reports and MetaDrive videos.
9. Export the selected designs through the backend-neutral configuration.
10. On Windows, install CARLA and implement the thin CARLA adapter for final validation.

## 9. Explicit boundaries

- Version one optimizes longitudinal behavior only; lateral control is fixed.
- Battery voltage sag and degradation are excluded. Optional battery power limits and a lumped
  motor thermal/derating model are included; their parameters remain illustrative until calibrated.
- MetaDrive is the authoritative optimization environment; CARLA is a transfer/validation check.
- CARLA disagreement will be reported as a simulator-transfer result, not hidden by retuning hardware.

## 10. Recoverable milestone — autonomous mountain shuttle

Milestone date: 2026-06-20.

Implemented:

- Battery discharge and charge power caps in the hardware-dependent actuator.
- Regenerative/friction braking redistribution when electrical charging power saturates.
- Resettable lumped motor temperature, loss integration, cooling, and torque derating.
- A 100 s, four-cycle autonomous shuttle mission with repeated +10% climbs, −10% descents, and
  station stops.
- Hard feasibility checks for RMSE, terminal progress, station position/speed, motor temperature,
  episode completion, and MPC fallback.
- A reproducible 60-point hardware/controller quick search and MkDocs evidence page.

Measured quick-grid result:

| Method | Hardware | RMSE | Net energy | Friction brake energy |
|---|---|---:|---:|---:|
| Conventional sizing, then MPC tuning | $g=10.5,s_m=0.60$ | 0.4182 m/s | 241.40 Wh | 60.21 Wh |
| Integrated co-design | $g=11.5,s_m=0.75$ | 0.4177 m/s | 211.03 Wh | 28.02 Wh |

The co-designed system saves 12.58% battery energy while completing effectively the same 750 m
mission with the same tracking quality. Recovered battery energy increases from 181.02 to
212.99 Wh. Across sampled hardware, minimum feasible energy spans 314.7–211.0 Wh. Thermal derating
does not activate in the 100 s quick run, so the result is driven primarily by final-drive/motor
effects on regenerative capacity and motor operating points rather than by an imposed thermal
failure.

Recovery commands:

```bash
python -m venv .venv
.venv/bin/pip install -e '.[all]'
.venv/bin/pytest
.venv/bin/python -m codesign.mountain_shuttle --quick
.venv/bin/mkdocs build --strict
```

Primary recovery documents are `PLAN.md`, `project_status.md`, and the MkDocs page
`docs/optimization/mountain-shuttle.md`. Machine-readable experiment outputs are regenerated under
`artifacts/mountain_shuttle/`; documentation copies of the selected plots are version controlled
under `docs/assets/validation/`.

## 11. Recoverable milestone — train/test generalization

Milestone date: 2026-06-20.

The scenario dataset contains four training and three disjoint held-out missions. It varies speed,
positive/negative grade, cycle count, payload, drag, initial motor temperature, battery limits, and
MetaDrive seed. Hardware is optimized using training results only. Once hardware is fixed, MPC
weights remain adaptable and are re-optimized independently for every training or test scenario.

Quick protocol:

- 15 proposed hardware candidates: five final-drive ratios by three motor scales; three candidates
  are rejected before training because they violate the shared 120 km/h motor-speed requirement.
- Fifteen MPC candidates per hardware/scenario pair: five energy weights by three slew weights.
- 720 training evaluations and 90 held-out evaluations.
- Hardware objective: minimum equally weighted mean training Wh/km.
- Hard per-scenario limits: RMSE ≤0.4 m/s, progress ≥98.5%, station accuracy, temperature,
  completion, and zero MPC fallback.

Measured result:

| Stage | Conventional $g=10.5,s_m=0.60$ | Training-selected $g=11.5,s_m=0.75$ | Reduction |
|---|---:|---:|---:|
| Mean training energy | 311.15 Wh/km | 274.79 Wh/km | 11.68% |
| Mean held-out energy | 344.25 Wh/km | 312.50 Wh/km | 9.22% |

The selected hardware uses less energy on every held-out scenario. All selected and conventional
test runs satisfy the 0.4 m/s RMSE threshold and mission constraints. Controller adaptation is
observable: selected hardware uses $\log_{10}\lambda_E\in\{-1.5,-0.5,0.5\}$ across training
scenarios; held-out controllers are independently selected after hardware freezing.

Recovery command:

```bash
.venv/bin/python -m codesign.generality_dataset --quick
```

Primary evidence is documented in `docs/optimization/generality-dataset.md`; machine-readable
manifests, evaluations, selections, cache, and report regenerate under
`artifacts/generality_dataset/`. The selected ratio lies at the shared top-speed-feasible boundary,
so the next experiment should refine that neighborhood without relaxing motor-speed requirements.

## 12. Recoverable milestone — held-out matched-RMSE dominance

Milestone date: 2026-06-20.

To remove ambiguity from comparisons where both designs merely satisfy the same RMSE threshold,
the held-out controller test now runs sequential constraints. Traditional hardware is first tuned
for minimum energy under RMSE ≤0.4 m/s. Its achieved scenario RMSE then becomes the maximum allowed
RMSE for the training-selected hardware. Both designs receive the same 40-point controller grid.

| Scenario | Traditional RMSE / Wh/km | Training-selected RMSE / Wh/km | Energy reduction |
|---|---:|---:|---:|
| Unseen steep | 0.3804 / 397.86 | 0.3541 / 364.99 | 8.26% |
| Heavy descent | 0.3522 / 338.24 | 0.3278 / 314.16 | 7.12% |
| Long fast shift | 0.3943 / 272.03 | 0.3723 / 245.91 | 9.60% |

Training-selected hardware has lower RMSE and lower energy in all three held-out scenarios. Mean
energy decreases from 336.04 to 308.35 Wh/km, an 8.24% reduction. Hardware remains fixed at
$g=11.5,s_m=0.75$ and was not changed using test information; only controller weights adapt per
scenario.

Recovery command:

```bash
.venv/bin/python -m codesign.matched_rmse_test
```

Evidence is documented in `docs/optimization/matched-rmse-test.md` and regenerates under
`artifacts/matched_rmse_test/`.
