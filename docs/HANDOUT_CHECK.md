# Handout check

Every number and rule we rely on, taken from `docs/assignment.pdf` (text extracted with
`pdftotext -layout`; page numbers are the PDF's), against the file and line that implements it. Line numbers refer to
the files as committed. Nothing was changed to make a row match; mismatches and interpretations are listed at the end.

| # | Handout item (page) | Handout value | Where it is in the code | Code value | Status |
|---|---|---|---|---|---|
| 1 | Turret inertia J_turret (p. 2) | 1,200 kg m^2 | `core/tc_params.m:22` | 1200 | match |
| 2 | Motor inertia J_m (p. 2) | 0.20 kg m^2 | `core/tc_params.m:22` | 0.20 | match |
| 3 | Gear ratio, motor spins 100x faster (p. 2) | 100:1 | `core/tc_params.m:22` (`K.N`) | 100 | match |
| 4 | Turret-side damping b (p. 2) | 1,500 N m s/rad | `core/tc_params.m:22` | 1500 | match |
| 5 | Torque constant K_t (p. 2) | 10 N m/A | `core/tc_params.m:23` | 10 | match |
| 6 | Back-EMF constant K_e (p. 2) | 10 V s/rad | `core/tc_params.m:23` | 10 | match |
| 7 | Armature resistance R_m (p. 2) | 1.0 ohm | `core/tc_params.m:23` | 1 | match |
| 8 | Armature inductance L_m (p. 2) | 0.10 H | `core/tc_params.m:23` | 0.1 | match |
| 9 | Plant from first principles (p. 2) | derive G_p(s) | `turret_plant.m:1-21`, `core/tc_params.m:25-28` (J_eq = J_turret + N^2 J_m; den from L, R, b, N^2 Kt Ke) | derived | match (derivation in `docs/DESIGN.md`) |
| 10 | Radar rate (p. 2) | 20 Hz, 50 ms | `core/tc_params.m:13` | 0.05 s | match |
| 11 | Range noise sigma_r (p. 2) | 1 m | `core/tc_params.m:14`, applied at `core/tc_scenario_build.m:35` | 1.0 | match |
| 12 | Angle noise sigma_theta (p. 2) | 0.02 deg | `core/tc_params.m:15`, applied at `core/tc_scenario_build.m:35` | 0.02*pi/180 rad | match |
| 13 | Prediction horizon t_p (p. 4, Sec. 4) | 0.50 s | `core/tc_params.m:10` | 0.5 | match |
| 14 | Engagement radius R_eng (p. 3) | 2 m, hit if distance <= R_eng | `core/tc_params.m:11`; judged at `simulation/tc_run_engagement.m:85-88` (`<= K.r_eng`) | 2.0 | match |
| 15 | Minimum time between pulses t_recharge (p. 3) | 1.0 s | `core/tc_params.m:12`; used in `core/tc_firing_rule.m:26` | 1.0 | match |
| 16 | Pulse rule (p. 3-4) | pulse on a 0 -> 1 transition of `fire_cmd`; holding 1 does nothing; earlier transitions ignored | judged at `simulation/tc_run_engagement.m:85-88` and `tests/eval_adapter.m` (edge and >= 1 s); `turret_controller` holds `fire_cmd` for `K.fire_hold` (`core/tc_params.m:74`) | edge + 1 s | match |
| 17 | Beam geometry (p. 3) | ray from the turret along the instantaneous turret angle | `core/tc_beam_miss_distance.m`; `tests/eval_adapter.m` | ray distance, target behind = range | match |
| 18 | Range limits (p. 3) | 800 m <= r <= 2000 m | `core/tc_params.m:18`; enforced at `core/tc_scenario_build.m:24` and `core/tc_scenario_check.m:8` | 800, 2000 | match |
| 19 | Speed limit (p. 3) | <= 150 m/s | `core/tc_params.m:16`; `core/tc_scenario_build.m:24` | 150 | match |
| 20 | Acceleration limit (p. 3) | <= 20 m/s^2 | `core/tc_params.m:17`; `core/tc_scenario_check.m:9` | 20 | match |
| 21 | Turret stops (p. 2) | -10 deg <= theta <= 85 deg | `core/tc_params.m:19`; hard stops in `simulation/tc_plant_step.m:25`; reference governor `core/tc_stops_governor.m:5` | -10, 85 deg | match |
| 22 | Blackout duration (p. 2-3) | t_bo <= 0.50 s | scenario C: `core/tc_scenario_catalog.m:16` | 0.5 | match (at the bound) |
| 23 | Blackout onset (p. 2-3) | t_boo >= 1.00 s | scenario C: `core/tc_scenario_catalog.m:16` | 1.0 | match (at the bound) |
| 24 | No measurement in blackout; target keeps moving (p. 2-3) | no new radar data | `target_motion_generator.m:30-34` (returns NaN, `meas_valid = 0`) | as stated | match |
| 25 | Run length t_end (p. 3) | 10 s, all results on 0..10 s | `simulation/tc_run_engagement.m:22` (`t_end` 10); scenario truth built to 11 s at `core/tc_scenario_build.m:15`, the envelope test uses 0..10 s | 10 s | match |
| 26 | `target_motion_generator` signature (p. 3) | `[r_meas, theta_meas, meas_valid] = f(t, scenario)` | `target_motion_generator.m:1` | same three outputs plus optional `truth`, `info` | match (extra outputs optional) |
| 27 | `turret_controller` signature (p. 3) | `[V_m, fire_cmd] = f(t, r_meas, theta_meas, meas_valid, theta_turret, theta_dot_turret)` | `turret_controller.m:1` | same inputs; third output `dbg` optional | match (extra output optional) |
| 28 | `turret_plant` signature (p. 3) | `[theta_dot_turret, theta_ddot_turret, i_dot] = f(t, V_m, theta_turret, theta_dot_turret, i)` | `turret_plant.m:1` | identical | match |
| 29 | Units (p. 3-4) | angles in rad, V_m in V, i in A | all of the above use rad, V, A | rad, V, A | match |
| 30 | `meas_valid` meaning (p. 3-4) | "indicates whether a new radar measurement is available" | `target_motion_generator.m:30-39` | 1 only on a new sample | match |

## Things the handout does not say (our assumptions, `docs/ASSUMPTIONS.md`)

Supply voltage limit (`core/tc_params.m:24`, 600 V), motor current budget (`core/tc_params.m:50`, 10 A), initial turret
angle, evaluation step size, initial positions and maneuver shapes of A/B/C (`core/tc_scenario_catalog.m`).

## Mismatches and interpretations to flag

- **No numeric mismatch** between the handout and `core/tc_params.m`.
- **Blackout at the bounds.** Scenario C uses exactly t_boo = 1.0 s and t_bo = 0.5 s. The handout allows t_bo <= 0.5 and
  t_boo >= 1.0, so the instructor's cases may use shorter or later blackouts; the random families (`MC<seed>`, `HB<seed>`)
  draw them inside those bounds.
- **"Elevation angle relative to the turret" (p. 2).** The handout says the radar gives the elevation angle "relative to
  the turret". We read it as the absolute elevation angle theta = atan2(y, x) in the turret's base frame (the radar is
  co-located with the turret), not an angle relative to the turret's current pointing direction. If the evaluator gave
  a relative angle, `theta_meas` would need theta_turret added before use. This is the single most consequential
  interpretation here and is not verifiable without the evaluation script.
- **Interface outputs.** `turret_controller` returns a third output `dbg` and `target_motion_generator` returns `truth` and
  `info`. MATLAB and Octave ignore unrequested outputs, so a two- or three-output call works.
- **Stops are enforced by the simulator, not by `turret_plant`.** `turret_plant` returns derivatives only and clips V_m to
  the voltage limit; the handout's -10..85 deg stops are applied by the integrating script (`simulation/tc_plant_step.m:25`,
  `tests/eval_adapter.m`). An evaluator that does not enforce the stops would let the turret pass them;
  the controller's reference governor (`core/tc_stops_governor.m`) keeps references inside them either way.
- **Handout says evaluation cases come "before the submission deadline" (p. 5).** They have not reached us; the
  random families stand in for them.
