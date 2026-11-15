# Why the first pulse missed from an 80 degree start (analysis only, nothing was changed)

Case: scenario A, 100 Hz, turret starting at 80 deg. The first pulse (t = 1.84 s) missed by 2.11 m against R_eng = 2 m; the next
pulse hit at 2.85 s, so the first hit slipped by one recharge second. Everything below comes from `outputs/start_angle_miss.txt`
and `plots/start_angle_miss.{png,pdf}`, produced by `make start-angle` (`studies/tc_study_start_angle_miss.m`, about 60 s).
The study replays the closed loop with the controller's debug outputs; its first pulse is identical to `tc_run_engagement`
(t = 1.840 s, miss 2.1086 m). No design or parameter file was touched.

## What the log shows, against the four suspects

| Suspect | Finding |
|---|---|
| Still slewing (acceleration clamp) | **Partly.** The reference had reached the linear zone at 1.63 s, but the turret was still decelerating at the pulse (rate -0.135 rad/s, 22 % of the 0.599 rad/s cruise rate, against -0.388 rad/s 0.1 s earlier; the acquisition clamps were still flagged active). Peak current in the slew was 9.52 A, below the 10 A budget, and the voltage clamp (600 V) was never reached. While the beam sweeps past the target at about 220 m/s the true beam error changes by about 2.5 m per 10 ms call (-5.04, -2.11, +0.40, +2.48 m), so the +-2 m window is only one to two calls wide. This explains why a small error matters, not where the error comes from. |
| Integrator or acquisition freeze | **Active but not shown to matter.** The freeze condition (`freeze_acq` and clamps active) held at the pulse and for 1.82 s of the first 3 s. The integrator content is not exposed, and no run without the freeze was made, so its effect is **not determined**. The feedback voltage was small at the pulse (u_fb = 7 V against u_ff = -120 V). |
| The 85 degree stop | **Ruled out.** The turret starts at 80 deg and moves away from the stop; at the pulse it is 59.6 deg from it; the stop-guard condition was never true after t = 0. |
| Hit probability above the threshold too early | **The rule behaved as designed.** p = 0.997 at the first call at which the rule was ready (n_upd = 37, consistency 1.0), above p_sure = 0.99, so it fired at once (the fire path is inferred from p; the armed state is not exposed). |

## What actually produced the 2.11 m

The true beam error splits exactly into two parts: -2.109 m = -1.241 m (the filter's own bearing error, converted to metres) +
-0.867 m (the offset the controller saw between its estimated target and the turret). The controller's part was inside the 2 m window;
the filter error pushed the truth outside. The posterior sigma at that instant was 0.33 m, so the probability treated the beam error
as N(-0.87, 0.33^2) m and returned 0.997, while the actual filter error was 1.24 m, about 3.8 posterior sigmas.

That filter error does not depend on the start angle: the estimator never sees the turret, and the log shows 1.241 m at t = 1.84 s for
starts of 0, 30, 60 and 80 deg alike. It is a property of the radar noise draw. The 0 deg start fired its first pulse at 0.94 s, when the
filter error was 0.10 m, and hit. So the start angle matters only through **when** the first pulse happens: from 80 deg the turret
arrives at 1.84 s, at the moment this noise draw had a 1.24 m estimation error, while the beam was still sweeping.

## Start angle versus first-pulse miss (same trajectory)

| Start | First pulse t [s] | Miss [m] | Median miss over 10 other noise draws [m] | Range over those draws [m] |
|---|---|---|---|---|
| 0 deg | 0.94 | 0.41 | 0.65 | 0.05-1.12 |
| 30 deg | 0.48 | 0.93 | 0.65 | 0.08-1.13 |
| 60 deg | 1.28 | 0.97 | 0.87 | 0.26-1.22 |
| 80 deg | 1.84 | 2.11 | 1.16 | 0.62-1.71 |

The median first-pulse miss grows with start angle (0.65, 0.65, 0.87, 1.16 m), consistent with firing nearer the end of the slew.
Of the 11 noise draws at 80 deg (the default one plus ten more), one missed by more than 2 m.

## Could and could not be determined

- **Determined:** the pulse missed because a 1.24 m filter error, larger than the posterior sigma implies, coincided with a beam
  that was still sweeping; the 85 deg stop, the voltage clamp and the current budget were not involved; the fire rule did what it is
  designed to do.
- **Also found:** the exact miss distance moves with rounding of the call times alone (2.109 m with times written k/f, 2.151 m with
  k*T, the form `tests/eval_adapter.m` uses). Both are above 2 m, but a distance this close to the limit is not reproducible to
  better than about 0.05 m across harnesses.
- **Not determined:** whether the acquisition freeze or the integrator state contributed (no counterfactual run was made, by design);
  whether the posterior is generally over-confident (one case, plus ten noise draws, cannot say); how often this happens in general
  (the independent check and the sweep count misses but were not analysed for this cause). The log contains one default noise draw
  plus ten others, from one scenario and one range, so the table is an illustration, not a statistic.
- **No design change is proposed or made here.**
