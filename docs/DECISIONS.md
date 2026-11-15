# Decisions and why

Format: decision, what was consulted, the derivation or measurement that settles it **in this
project**. "Reference" means the friend's public repository and its hand-off notes (see
`ATTRIBUTION.md`, not included in this repository); we use it to find candidate values and known pitfalls, never as evidence.
Seed ranges follow `docs/PROTOCOL.md`: choices are made on 1–200 (calibration) or 201–4999
(exploration); evidence is quoted from 5000+.

## D1  Cached-matrix test was NaN-blind (bug, fixed)
`if abs(S.Phi_dt - dt) > 1e-12` is *false* when the cache is empty (NaN), so the exact
triple-pole transition and the plant's exact ZOH matrices were never built: the reference's linear
zone used the identity and the "exact" plant integrator stood still. Found by the first run of
`tc_test_all` (4 failures). Rewritten `~(abs(.) <= tol)`. Not a tolerance issue; the real
controller was affected. Regression guard: `t_integrators_agree` also asserts the state moved.

## D2  Envelope is an obligation for 0–10 s, not for the 11 s table
Scenario A drifts to r = 796 m at t = 11 s. The assignment fixes t_end = 10 s, so the envelope is
asserted on 0–10 s for every scenario (named, MC, HB); the table runs to 11 s only so a call at the
last step never reads past it.

## D3  Test tolerances replaced by convergence checks
Euler-vs-exact for the reference cascade and RK4-vs-exact for the plant are *numerical-method*
comparisons, so the right test is the order of convergence, not an arbitrary 1e-6/1e-7: Euler error
falls x10 per x10 steps (6.4e-4, 6.4e-5, 6.4e-6), RK4 falls x16.2 and x16.1 per halving of h.

## D4  Harness conventions
Radar samples exist only at t = 0.05 k, so call rates must be integer multiples of 20 Hz (the
brief's own setting): 333 Hz is not a valid test (400 Hz is). For irregular call times the harness
delivers each radar sample once, at the first call after its tick.

## D5  Acquisition anti-windup (`freeze_acq`, default on) — benefit NOT confirmed in the final design
Observed in scenario A: during the clamp-limited slew the turret reached the target with a ~3 m overshoot and
P_hit crossed 0.99 only at t ~ 1.3 s, because the PI integrator had accumulated lag while the reference was not
following the linear law. Pausing integration while the cascade clamps are active looked like a fix: on exploration
seeds MC201–240 mean first hit 1.621 -> 1.545 s (40/40, 0 misses). **That gain was confounded by the slip guard
(D12).** After the guard was removed, the final-design ablation shows no effect: `no_freeze` -0.005 s [-0.022, +0.012],
-0.002 m RMS, Holm p = 1.0 (`outputs/ablations.txt`). The flag stays on (it is the conservative anti-windup choice and
costs nothing) but we do not claim it as an improvement.

## D6  Acceleration clamp from a current budget, not a typed constant
`acq_accel = 0.8 N K_t i_lim / J_eq` with an *assumed* i_lim = 10 A gives 2.5 rad/s^2. The reference
uses the same 2.5 and its notes mention a 10 A hardware figure that the assignment does not state.
Relaxing it to 5 rad/s^2 (about 16 A) cut mean first hit by 0.21 s with no misses on MC201–240, but
it exceeds any current we can justify from the handout, so it is **not** the default. If the
instructor's plant states a current limit, change `K.i_lim` in `core/tc_params.m`: it is the only place.
Peak current is reported (`I_peak`): 8.8 A on D6 at 100 Hz, but up to 10.7-11.4 A in some robustness rows
(50 Hz; R,L,J,b +20 %) and 20 A with Kt -20 % / Ke +20 %: the 20 % margin covers the reference acceleration only, not
feedback or plant mismatch, so the budget is an acquisition guideline, not a hard current limit.

## D7  Braking-curve slew (`acq_decel`) rejected after a confirmation run
Constant-deceleration braking to shave the exponential tail looked good on the exploration seeds
MC201–260 (decel 1.5: -0.127 s mean first hit, but 2 pulse misses; decel 3.0: +0.072 s). Because that
was a selection on the same seeds, it was **confirmed once on fresh seeds MC2000–2099** (outside the
5000+ evidence range): decel 1.5 was *slower*, +0.093 s [+0.080, +0.104], 0 vs 1 pulse misses. The
exploration gain did not replicate, so it stays off (`outputs/probe_decel.log`,
`outputs/probe_brake_confirm.log`).

## D8  Feedback gains and generator pole kept from the reference, re-verified analytically
K_p = 4000 V/rad, T_i = 1 s, tau_f = 0.03 s and p = 20 rad/s are the reference's values. We did
**not** re-optimise them; we re-derived what they imply (`outputs/analysis.txt`): crossover 4.10 rad/s,
PM 68.5 deg, GM 16.1 dB, M_s 1.21, Routh limit for proportional-only 10 484 V/rad, full loop stable to
6.35x the design gain, type-2 loop with K_a = 3.99 1/s^2 (FVT parabola error confirmed by exact
partial-fraction simulation: 0.0150 rad both ways).

## D9  MATLAB compatibility of the submission path (static audit only; never run under MATLAB)
`turret_controller.m`, `turret_plant.m` and every `core/` function they call avoid Octave-only syntax
(`+=`, `printf`, `#`, chained indexing, `}{`) by grep; no MATLAB was available to execute them, so
compatibility is *checked by reading*, not demonstrated. The audit found two Octave-only spots, both fixed:
`(A*B')(:)'` in `tc_session_start.m` and `{'A','B','C'}{k}` in `target_motion_generator.m`.

## D10  Calibration layer: fitted, evaluated, **not adopted**
Isotonic map fitted on tuning seeds MC1–200 (ticks with a ready, tracking filter; outcome = "a pulse
right now would hit"), scored on MC5000–5199 (37 297 ticks, base rate 0.886): Brier 0.00353 raw ->
0.00353 calibrated. Reliability (raw / observed): 0.673/0.617 (n=193), 0.856/0.850, 0.929/0.929,
0.978/0.972, 0.9964/0.9933 (n=1195), 1.0000/1.0000 (n=30 928). The Rao-Blackwellised posterior is
already close to calibrated; the reference's reported mid-range conservatism belongs to its
17-class mixture. The identity table ships (`core/tc_calibration_table.m`); the fitted candidate is
`outputs/calib_fit_candidate.m.txt`. The remaining slight over-confidence at P in [0.99, 0.999) is
the honest residual (about 0.3 points).

> **Superseded 2026-11-15** (numbers above are from the earlier run; the text is kept as the record of that run). Re-run with the final
> design, same seeds and command arguments (`outputs/calibration_study.txt`): 37 297 ticks, base rate 0.888, Brier 0.00285 raw ->
> 0.00285 calibrated. Reliability (raw / observed): 0.670/0.636 (n=176), 0.855/0.921 (n=127), 0.929/0.938, 0.976/0.975,
> 0.9965/0.9977 (n=860), 1.0000/1.0000 (n=31 452). The conclusion (no gain from calibration, identity table ships) is unchanged; the
> "slight over-confidence at P in [0.99, 0.999)" is not seen in the re-run (observed 0.9977 against raw 0.9965). The cause of the
> difference from the earlier run is not established (it is not the renaming).

## D11  Mid-hold reference lead (`lead_hold`) removed — my own idea, refuted
Idea: V is held for one call period, so evaluate the reference half a period ahead ("ZOH
compensation", `future.md` D.3, not included in this repository). It is wrong for this problem: the beam is judged at the sampling
instants and the exact inverse already tracks the nowcast, so the lead simply moves the aim point
ahead of the target by (LOS rate x lead), about 2 m at 20 Hz. Paired MC201–260 at 20 Hz: pointing RMS
0.996 m without the lead vs 1.677 m with it (+0.717 m [+0.572, +0.863]); at 100 Hz +0.07 m; on the HB
family -0.053 m without it. Default `lead_hold = false`; the option stays for the ablation table.

## D12  Fire "slip guard" off by default
`sweep_guard` blocked shots when the beam-vs-target slip over one call period exceeded 0.5 m (a
safeguard for an evaluator that judges between calls). Measured cost with no benefit: first-hit time
0.21 s at 100 Hz and 1.1 s at 20 Hz, pulse misses not increased when removed (MC201–260,
`outputs/probe_guard20.log`, earlier cap/no-cap runs). The harness (and the handout) judge at the call
instant, so the guard penalises the case we model. `sweep_guard = Inf` by default; `guard_window`
(10 ms) is kept so it is rate-independent if an evaluator ever needs it.

## D13  Adaptive process noise kept, because of the adversarial family
On smooth random engagements switching it off changes nothing (+0.006 s, p = 0.73). On the HB family
(bang-bang jerk, maneuvers inside blackouts) removing it costs +0.094 m pointing RMS [+0.062, +0.127]
(exploration seeds HB201–240, `outputs/probe_HB.log`). It stays on.

## D14  Features with a measured null effect (kept, reported)
`p_sure_after` (value-aware bar after the first pulse) and `n_upd_min = 4` produced *exactly* zero
difference on MC201–260; the posterior vs 17-class hit model differs by 9 ms. The reference flagged a
similar no-effect feature (class re-identification) as clutter, so we say plainly: these are neutral
here; `p_sure_after` is kept at 0.995 because it costs nothing and acts only after a first pulse; the
17-class model is kept only as the comparison baseline.

## D15  The 'correlated copies' failure was a harness bug, not a controller weakness
First robustness run: `convention = drift` (meas_valid = 1 on every call, value = current truth + the last radar
sample's noise) gave 14 pulse misses and a median RMS of 2.56 m. Raising `min_gap` (0.4 -> 0.9 radar periods) did not
help, which pointed away from the estimator: the harness kept `meas_valid = 1` *during blackouts*, building the sample
from a stale held measurement minus the truth at the current tick. Fixed (the held sample's own tick supplies the noise;
a blacked-out tick clears it). Re-run MC201–230: 0 misses, median RMS 0.570 m at the default gap, so `min_gap = 0.4 x 50 ms`
stays (`outputs/probe_gap_drift.log`, not included in this repository, is the pre-fix probe; the post-fix numbers are in `outputs/robustness.txt`).

## D16  Adaptive velocity feed-forward is ineffective where it was supposed to help
`adapt_ff` (move integrator content into the velocity feed-forward gain) changes nothing on the nominal plant and in the
worst mismatch corner (Kt -20 %, Ke +20 %) leaves RMS at 3.7 m with a 20 A peak; in the opposite corner it moves 1.57 -> 1.50 m.
It stays **off**. The reference notes the same sensitivity (the velocity feed-forward term is ~ N K_e); a better remedy
(e.g. estimating K_e from the (V, omega) residual with persistent excitation) is future work, not claimed here.

## D17  Controller cost
`outputs/time.txt`: mean 0.527 ms, median 0.518 ms, p99 0.722 ms per call in GNU Octave 11.3 (5000 calls, scenario B,
1 kHz). That is within a 1 ms budget but not faster than the reference's MATLAB figure (0.51 ms mean); removing
`expm` helped, the interpreter cost dominates. The claim "1 kHz real-time headroom" is therefore only modest.

> **Superseded 2026-11-15** for the exact figures: the re-run (`outputs/time.txt`) gives mean 0.528 ms, median 0.515 ms, p99 0.732 ms.
