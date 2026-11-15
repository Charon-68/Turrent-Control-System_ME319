# Design notes — Guardian elevation fire control (GNU Octave)

Everything here is derived from the assignment and the hand-off notes; no MATLAB is used or required.

## 1. Plant (first principles)

Armature `L di/dt = V − R i − N K_e θ̇`, mechanics `J_eq θ̈ = N K_t i − b θ̇`, `J_eq = J_turret + N² J_motor = 3200 kg m²`.

```
θ/V = N K_t / ( s [ (L s + R)(J_eq s + b) + N² K_t K_e ] ) = 1000 / ( s (320 s² + 3350 s + 1.0015e6) )
```
Poles `0, −5.23 ± 55.70j` (8.90 Hz, ζ = 0.094). Relative degree 3, so the exact inverse is `V = 0.32 y‴ + 3.35 y″ + 1001.5 y′`. In this project those three numbers are **computed** in `core/tc_params.m` (`K.ff = K.den / K.gn`); the back-EMF-limited cruise rate `V_lim / K.ff(3)` and the acquisition clamp (97 % of it) follow automatically.

## 2. Why the beam is NOT aimed at θ(t + t_p)

The laser is instantaneous, so a hit needs the beam on the target's *present* bearing. Aiming t_p ahead misses by r·θ̇·t_p ≈ 37 m. The prescribed prediction `r = A₃(D) θ̂` (Taylor section of e^{t_p s}) is a fixed linear combination of estimated derivatives; the reference generator `(A₃ + K)(D) y_d = r + K(D) θ̂`, with `A₃ + K = (t_p³/6)(s + p)³`, cancels the advance exactly: `(D + p)³ (y_d − θ̂) = 0`. The loop is therefore **independent of t_p**, and radar noise reaches the turret only through the estimator. The useful part of "prediction" is the motion model: it coasts through blackouts, supplies θ̂′…θ̂‴ for the exact inverse, and feeds the fire rule's look-ahead.

## 3. Blocks and where they live

| Block | File | Notes |
|---|---|---|
| Unbiased converted measurement | `core/tc_convert_measurement.m` | Lerro/Bar-Shalom form; keeps the filter linear |
| Markov-jerk transition F(τ), Q(τ) | `core/tc_jerk_model_transition.m` | **Closed form** F, Q by Gauss–Legendre quadrature of the impulse response — no `expm`, any gap |
| Kalman step + gating + adaptive Q | `core/tc_filter_update.m` | duplicate / too-close / non-finite samples rejected; χ² gate; NIS-driven noise scale |
| Envelope projection | `core/tc_envelope_clip.m` | v, a, jerk |
| Exact LOS derivatives | `core/tc_los_derivatives.m` | complex log-derivatives |
| Stops governor | `core/tc_stops_governor.m` | freezes the *whole* derivative tuple beyond the guard band |
| Reference generator | `core/tc_reference_advance.m`, `core/tc_reference_chain.m` | exact matrix-exponential step in the linear zone, clamped cascade during acquisition |
| Controller | `turret_controller.m` | one persistent struct, no classes |
| Hit probability | `core/tc_hit_probability.m` | Rao-Blackwellised Gauss–Hermite mixture over acceleration (baseline: `tc_hit_prob_classes.m`) |
| Calibration | `core/tc_calibrate_probability.m`, `core/tc_calibration_table.m` | isotonic map fitted on tuning seeds only |
| Fire rule | `core/tc_firing_rule.m` | fire-now-or-wait, coasting rule, value-aware after the first pulse |

## 4. Feedback

`C(s) = K_p (1 + 1/(T_i s)) / (τ_f s + 1)`, `K_p = 4000 V/rad`, `T_i = 1 s`, `τ_f = 0.03 s`: crossover ≈ 4 rad/s (a decade under the 55.9 rad/s resonance), PM ≈ 68°, GM ≈ 16 dB; the resonance is gain-stabilised. Anti-windup is *conditional integration* on voltage saturation **and** on the elevation stops (new).

## 5. What differs from the hand-off baseline, and what the evidence said

Outcome column: **adopted** = measured benefit or required; **neutral** = kept, no measurable effect; **rejected** = tried, measured, left off (see `docs/DECISIONS.md` for numbers).

| Item (`future.md` / `vulnerabilities.md`, not included in this repository) | Treatment | Outcome |
|---|---|---|
| Hit-probability calibration (B.1) | isotonic table fitted on MC1–200, scored on MC5000–5199 | **rejected** (Brier 0.00353 -> 0.00353; superseded 2026-11-15 by the re-run: 0.00285 -> 0.00285, `outputs/calibration_study.txt`); identity ships |
| Posterior instead of 17 classes (B.2) | `tc_hit_probability` vs `tc_hit_prob_classes` | **neutral** in time/accuracy (0 s difference on MC201–260), principled (no double-counted variance) |
| Per-shot value awareness (B.4) | `p_sure_after` | **neutral** (exactly 0) |
| IMM/adaptive Q (C.1) | NIS-driven noise scale | **adopted**: null on smooth MC, +0.094 m RMS if removed on the HB family |
| Plant-parameter robustness (D.1) | online velocity-FF scale `adapt_ff` | **rejected**: ineffective in the worst corner (3.74 vs 3.79 m) |
| Friction / dead-zone (D.2) | non-ideal plant in `simulation/tc_plant_step.m` | tested: 0.72 m RMS median with 300 N m Coulomb + 20 V dead-zone |
| ZOH loss at low rates (D.3) | mid-hold reference lead | **rejected**: biases the beam ahead (+0.72 m RMS at 20 Hz) |
| Reference governor at the stops (D.4) | whole-tuple freeze + integrator pause near stops | implemented, **not ablated**: scenario D3 (reaches 83 deg) intercepts 9/9 and `t_lost_track_recovery` passes; no benefit is claimed |
| Time-optimal acquisition (A.1) | braking-curve slew | **rejected**: looked good on tuning seeds, +0.093 s on fresh ones |
| Faster acquisition by current budget | `acq_accel` from `i_lim` (2.5 rad/s^2) | **adopted** as a documented assumption; 5 rad/s^2 would save 0.21 s at ~16 A |
| Acquisition anti-windup | integrator paused while clamps active | **neutral** in the final design (earlier gain was confounded) |
| Slip guard on shots | `sweep_guard` | **rejected** as default (+0.21 s at 100 Hz, +1.1 s at 20 Hz) |
| Held-out seeds (E.1) | `docs/PROTOCOL.md` | **adopted** |
| Harder generators, glint, range noise, bias (E.2, C.3) | `HB<seed>`, `+glint/+range/+bias` | **adopted** (all intercept 30/30 in `outputs/robustness.txt`) |
| Evaluation-script emulator (E.3) | `convention`: `held`, `drift`, `jitter` | **adopted** |
| Closed-loop MC without MATLAB (E.4) | everything runs in Octave | **adopted** |
| CI with Octave (E.5) | `.github/workflows/ci.yml` | **written, never run on GitHub** |
| Performance (E.6) | closed-form F, Q (no `expm` per call), vectorised mixture | **modest**: 0.53 ms mean per call in Octave |
| Single source of constants (F.2) | `core/tc_params.m` only | **adopted** |

The research-only modes (exact-Taylor references, homotopy planner, ambush, re-identification, order 2/3 estimators) are **not** carried over (vulnerabilities C.1, C.7, C.8).
