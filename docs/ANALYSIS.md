# Analytical design pages

All numbers are produced by `studies/tc_plot_design_analysis.m` (see `outputs/analysis.txt`), plots in
`plots/design_analysis.*` (pole-zero, Bode, step) and `plots/root_locus.*`. No control toolbox is used.

## 1. Plant G(s)
States θ, ω = θ̇, i; N = 100, J_eq = J_t + N²J_m = 3200 kg m².
```
L di/dt = V − R i − N K_e ω           J_eq ω̇ = N K_t i − b ω
G(s) = θ/V = N K_t / ( s [ (L s + R)(J_eq s + b) + N² K_t K_e ] )
           = 1000 / ( s (320 s² + 3350 s + 1.0015·10⁶) )
```
Poles 0 and −5.23 ± 55.70j: ω_n = 55.9 rad/s (8.90 Hz), ζ = 0.094. Relative degree 3 → the exact inverse
`V = (320 y‴ + 3350 y″ + 1.0015·10⁶ y′)/1000` is a proper realisation when y, y′, y″, y‴ are available (they are:
the reference generator and the estimator supply them without differentiation).

## 2. Compensator and loop
C(s) = K_p (T_i s + 1) / (T_i s (τ_f s + 1)), K_p = 4000 V/rad, T_i = 1 s, τ_f = 0.03 s.
L(s) = C(s)G(s) has a double pole at the origin (plant integrator + PI integrator): **type 2**.

## 3. Routh gain limit
Proportional only, characteristic polynomial `a₂s³ + a₁s² + a₀s + k·N K_t`, a₂ = 320, a₁ = 3350, a₀ = 1.0015·10⁶.
Routh array first column: `a₂, a₁, (a₁a₀ − a₂ k N K_t)/a₁, k N K_t` → stable iff **k < a₁a₀/(a₂ N K_t) = 10 484 V/rad**
(the design K_p = 4000 is 38 % of that). For the full PI + roll-off loop the same test by bisection on the loop-gain scale
gives stability for 0 < k < **6.35** (K_p < 25 391 V/rad), i.e. a gain margin of 16.1 dB.

## 4. Root locus
`plots/root_locus.*` plots the roots of `den(L) + k·num(L) = 0` as k grows. The closed-loop poles at k = 1 are
−1.63, −2.86, −29.7 and −4.82 ± 54.66j. At k = 1 the lightly damped pair has hardly left its open-loop position
(−5.23 ± 55.70j); it is the pair that bends toward the imaginary axis as k grows and reaches it at k = 6.35 (ω ≈ 48.5 rad/s,
the phase-crossover frequency) — this branch sets the gain margin. The roll-off pole (−33 rad/s) and the real pole near the
origin meet and leave the real axis as a second, still-stable pair (≈ −21 ± 25j at k = 6.35). The slow real poles near
−1.6 and −2.9 rad/s come from the PI zero at −1 rad/s and set the ~2.6 s settling of the feedback loop alone.

## 5. Frequency response
Crossover 4.10 rad/s, **PM 68.5°**, phase-crossover 48.5 rad/s, **GM 16.1 dB**, M_s = 1.21 (1.7 dB), closed-loop
bandwidth 5.72 rad/s. The crossover is 13.7× below the resonance, so the resonance is handled by gain stabilisation
(no notch). With the zero-order hold of a sampled controller (continuous-time ZOH equivalent, an *approximation* of the
exact C(z) loop) the margins are 67.3°/17.4 dB at 100 Hz, 66.1°/18.2 dB at 50 Hz and 62.6°/17.6 dB at 20 Hz
(`outputs/design_report.txt`).

## 6. Steady-state error (final value theorem)
E(s) = R(s)/(1 + L(s)). Because L has two poles at the origin:
position constant ∞, velocity constant K_v = ∞, **acceleration constant K_a = lim s²L(s) = K_p·N K_t/(T_i a₀) = 3.994 s⁻²**.
Step error 0, ramp error 0, parabola of acceleration ä gives e_ss = ä/K_a. Worst LOS acceleration 0.060 rad/s² → 0.0150 rad
(22.5 m at 1.5 km) for the *feedback alone*; the exact-inverse feed-forward removes it for a correct model.
Check: exact partial-fraction simulation of the sensitivity S = 1/(1+L) driven by a 0.06 rad/s² parabola gives 0.0150 rad at
60 s; a 0.1 rad/s ramp gives 3·10⁻⁴⁴ rad.

## 7. Step response (feedback loop alone, T = L/(1+L))
Overshoot 15.0 %, 10–90 % rise 0.31 s, 2 % settling 2.63 s. The overall reference response is *not* this: the 2-DOF
structure tracks y_d through the plant inverse, so the reference response is set by the generator poles (−20 rad/s, triple).

## 8. Why the generator poles do not depend on t_p
With r = A₃(D)θ̂ and (A₃ + K)(D) y_d = r + K(D) θ̂: (A₃+K)(D)(y_d − θ̂) = 0, and A₃ + K = (t_p³/6)(s + p)³ by
construction ⇒ (D + p)³ (y_d − θ̂) = 0. K(s) = 1.125 s² + 24.5 s + 165.67 for t_p = 0.5, p = 20.
Taylor sections A_n(s) of e^{t_p s} are Hurwitz only for n ≤ 4 with least damping 1.00, 0.71, 0.36, 0.11 (n = 1…4);
n = 5 is unstable (`outputs/design_report.txt`).
