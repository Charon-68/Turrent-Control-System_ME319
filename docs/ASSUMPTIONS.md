# Assumptions

The handout leaves several things open and nothing more is coming from the instructor, so we picked values, wrote them
down here, and built the code so each one can be changed in one place. Each entry gives the assumption, why we made it,
and what changes if it is wrong.

## 1. Voltage limit: |V_m| <= 600 V

- **Assumption.** `K.V_lim = 600` in `core/tc_params.m`; `turret_plant` clips V_m to it.
- **Why.** The handout gives no supply limit. The back-EMF constant N*Ke = 1000 V per rad/s at the turret, so 600 V allows
  a no-load cruise of about 0.6 rad/s (about 34 deg/s). That is enough for the geometry we use and still a real constraint.
- **If wrong.** The cruise rate, the acquisition rate clamp and the feed-forward saturation are all derived from `V_lim`
  (`omega_cruise = V_lim/ff(3)`, `acq_rate`). Change the one number and everything derived follows. A lower limit means
  slower first acquisition (later first hit); a higher one means faster. Tracking after acquisition is not much affected.

## 2. Current budget: 10 A during acquisition

- **Assumption.** `K.i_lim = 10`. The acquisition acceleration clamp is `acq_accel = 0.8*N*Kt*i_lim/J_eq` = 2.5 rad/s^2,
  an 80 % margin on 10 A.
- **Why.** The handout gives no current limit; 10 A is a plausible hardware figure. The plant has no saturation on current
  (only on voltage), so the budget is a design choice, not enforced by `turret_plant`.
- **If wrong.** A larger budget would shorten first-hit time (5 rad/s^2 corresponds to about 16 A and was about 0.21 s
  faster in our probes, `docs/DECISIONS.md`). Peak current in tracking can exceed 10 A in plant-mismatch cases
  (`outputs/robustness.txt`). If the evaluator penalises current, lower `i_lim`.

## 3. `meas_valid` convention

- **Assumption.** `meas_valid = 1` marks a call that carries a NEW radar sample (every 50 ms, outside blackouts); other
  calls have `meas_valid = 0` and NaN values. Our generator behaves this way. The controller also tolerates a second
  convention in which the flag stays 1 on every call with the last sample repeated: samples closer than
  `K.min_gap = 0.4*dt_radar` to the previous accepted one are ignored.
- **Why.** The handout says radar data arrive every 50 ms and that nothing arrives during a blackout, which suggests a flag
  per sample. If the evaluator calls faster than 20 Hz it has to say somehow which calls carry data.
- **If wrong.** Under a "held" flag, the controller can no longer tell a blackout from silence by the flag alone. It
  relies on the repeated values being identical and on the gap rule. `tests/tc_test_all.m` and `outputs/robustness.txt` cover the held
  convention; it works, but with less margin than the "new sample" convention. A third convention (NaN data with the flag
  still 1) is handled by ignoring non-finite values.

## 4. Scenarios A, B, C

- **Assumption.** (Our definitions, `core/tc_scenario_catalog.m`.)
  - **A**: start (1700, 700) m, velocity (-120, 0) m/s, no acceleration, no blackout.
  - **B**: same start; from t = 1 s a smooth acceleration of 8 m/s^2 (x, 0.15 Hz) and 14 m/s^2 (y, 0.30 Hz) switched on
    with a quintic ramp over 0.5 s.
  - **C**: A with one blackout starting at 1.0 s lasting 0.5 s.
  - All trajectories stay inside 800 <= r <= 2000 m, |v| <= 150 m/s, |a| <= 20 m/s^2 on 0 to 10 s
    (checked by `t_envelope_all`).
- **Why.** The handout states the envelope and the event times but not initial positions or maneuver shapes.
- **If wrong.** The instructor's scenarios will differ in geometry. The envelope-based random family (`MC<seed>`) and the
  stress family (`HB<seed>`) are there to cover that. A scenario outside the envelope (for example faster than 150 m/s)
  would break the estimator's gating and projection limits (`env_v`, `env_a`).

## 5. Initial turret angle

- **Assumption.** The simulator default is theta0 = 0 rad (horizontal). Nonzero starts (for example 0.3 rad) are tested
  through `tests/eval_adapter.m`; the controller reads `theta_turret` on its first call and starts from there.
- **Why.** The handout does not specify it.
- **If wrong.** A start far from the target bearing lengthens acquisition by distance / cruise rate (cruise about
  0.58 rad/s) and the first pulse comes later. Starts outside -10 to 85 degrees are not meaningful (the stops).

## 6. Call-rate range

- **Assumption.** The evaluator calls all three functions at a fixed multiple of 20 Hz, between 20 Hz and about 5 kHz,
  and integrates the plant with a step of at most 1 ms (zero-order hold on V_m).
- **Why.** The handout gives no step size. 20 Hz is the radar rate, so it is the lowest rate that makes sense. Forward
  Euler on this plant is unstable above about 3.3 ms; we assume the evaluator is careful.
- **If wrong.** Below 20 Hz the controller would run on stale data. A plant step above 3 ms with forward Euler would
  diverge for any controller. A call rate that is not a multiple of 20 Hz makes radar-tick times fall between calls: the
  generator only returns data when t is within 1e-6 s of a tick, so such a call pattern would lose samples.
  Calling inside `ode45` (variable, rejected steps) is not supported: the controller resets when time goes backwards.

## 7. Evaluation-script protocol

- **Assumption.** The evaluator, at its own step size, (1) calls `target_motion_generator(t, scenario)`, (2) passes the
  result and the current plant state to `turret_controller`, (3) holds V_m over the step and integrates `turret_plant`
  itself, and (4) judges a pulse on each 0 -> 1 edge of `fire_cmd` at least 1 s after the previous pulse, using the
  instantaneous turret angle and the target position at that instant, hit if the beam-to-target distance is <= 2 m.
  `tests/eval_adapter.m` implements exactly this.
- **Why.** It is the most literal reading of the handout's interface and pulse rules.
- **If wrong.** If pulses are judged BETWEEN calls (interpolated), a turret that is sweeping at the instant of the pulse
  can slip out of the 2 m radius; we have a guard for that (`sweep_guard`, off by default because it delayed first hit).
  If `fire_cmd` is sampled more slowly than the controller can change it, a one-tick pulse could be missed, so the
  controller holds `fire_cmd = 1` for `fire_hold = 0.05 s` (harmless according to the handout). If the evaluator
  ends the run at the first hit instead of scoring all pulses, nothing changes.

## 8. Step-response targets in the specification table are our own

- **Assumption.** The targets used in the course-concept specification table (overshoot <= 20 %, rise time <= 0.5 s,
  settling time <= 3 s; `studies/tc_study_specs.m`) are our own assumptions, fixed before the study was run.
- **Why.** The handout states no numeric step-response requirement (checked against the extracted handout text, see
  `docs/HANDOUT_CHECK.md`). A table needs some target to compare against, so we chose round values.
- **If wrong.** The pass/fail marks in that table would change; the measured step numbers (overshoot, rise time,
  settling time of the loop) do not depend on the targets. They must not be presented as requirements from the handout.

## 9. Feed-forward vs feedback: no time-domain ablation, linear-plant comparison only

- **Assumption / limitation.** The time-domain check of "feed-forward only" versus "feedback only" **was not run**
  on the full controller, because no switch exists in `turret_controller.m` / `core/` that disables the feed-forward or
  the feedback path, and we did not change code to create one.
- **What is reported instead.** The comparison in `studies/tc_study_feedforward_vs_feedback.m` uses the linear plant model
  (polynomial transfer functions): feed-forward only, feedback only and both, on the nominal plant and with Kt, Ke, J or b
  off by +-20 % (single parameters and the 16 corners), for a ramp reference at the worst LOS rate through the generator.
  It is a linear-plant analysis; it does not include saturation, the estimator, blackouts or sampling.
- **If wrong.** A closed-loop engagement with the feed-forward disabled could behave differently (for example because of
  saturation or estimator error). That experiment is open.
