# Evaluation protocol (read before quoting any number)

| Range | Use | Rule |
|---|---|---|
| `MC1 … MC200` | **tuning / calibration** | The only seeds allowed to influence a constant (isotonic map in `core/tc_calibration_table.m`). |
| `MC201 … MC4999` | free exploration | May be looked at while debugging; never quoted as evidence. |
| `MC5000 … MC5999` | **held-out evaluation** | Never used to choose or fit anything. Every headline statistic comes from here. |
| `HB<seed>` | adversarial family (bang-bang jerk ≈ 450 m/s³, maneuvers inside blackouts, blackout at exactly 1.0 s, back-to-back 0.5 s blackouts) | Reported separately from `MC`; it is a stress family, **not** a failure-probability estimate. |
| `<name>+glint`, `+range`, `+bias` | out-of-spec sensor models | Reported as sensitivity, not as spec compliance. |

Rules

1. Variants are compared on **identical seeds** (same trajectory, blackouts and noise) and tested with a paired bootstrap / sign-flip permutation (`studies/tc_paired_stats.m`). With *k* variants compare against 0.05/*k*.
2. The failure bound is the Wilson lower limit of the success rate on the held-out range: `failure ≤ 1 − lower`. 1000/1000 gives < 0.38 %; 300/300 gives < 1.3 %, so say which one you ran.
3. A run with no hit is **censored**, not averaged: it is counted in the failure statistic and listed separately from first-hit statistics.
4. `tc_settings('clear')` must be called after any experiment; `tc_test_all` does it after every check.
5. Changing a constant after looking at held-out results invalidates that range; open a new range (6000+) and say so in the commit message.
