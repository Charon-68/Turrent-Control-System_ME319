# Results bundle

Every file below lists the exact command that produced it. Commands are run from the repository root with `octave-cli --quiet --eval "..."`
(`make <target>` runs the same expression; see the `Makefile`). Where the program only prints, the file is its captured output with the
command as the first line. Seeds are fixed; only timing varies with the machine.

Re-run on 2026-11-15 with Octave 11.3.0: everything matched the stored result
exactly except `calibration_study.txt` (new Brier 0.00285, stored 0.00353) and the `p99` in `time.txt` (0.732 ms, stored 0.722 ms).
The calibration re-run gives Brier 0.00285 raw = 0.00285 calibrated (the stored, earlier value was 0.00353 = 0.00353); the cause of the
difference was not established. Both runs show no gain from the fitted map, so it is still not adopted and the identity table ships.
Superseded intermediate results from earlier design states are not included in this repository.

| File | Produced by | Content |
|---|---|---|
| `heldout_final.{txt,mat}` | `make heldout-merge` = `tc_merge_heldout({'heldout_MC5000_5299','heldout_MC5300_5649','heldout_MC5650_5999'}, 'heldout_final')` | **headline**: 1000 held-out engagements MC5000–5999 |
| `heldout_MC5000_5299`, `_5300_5649`, `_5650_5999` `.{txt,mat}` | `make heldout` (chunk 1) and `tc_heldout_campaign('MC', 5300:5649, 'heldout_MC5300_5649')`, `tc_heldout_campaign('MC', 5650:5999, 'heldout_MC5650_5999')` | per-chunk results |
| `suite_100Hz.txt` | `make scenarios` = `tc_setup(); T = tc_scenario_suite();` (captured output) |
| `suite_rates.txt` | `tc_setup(); disp('--- 20 Hz'); T = tc_scenario_suite(struct('f_ctrl', 20)); disp('--- 1000 Hz'); T = tc_scenario_suite(struct('f_ctrl', 1000));` (captured output) |
| (note on the row below) | | A–D6 at 100 Hz; 20 and 1000 Hz |
| `design_report.txt` | `make design` = `tc_setup(); tc_design_report()` (captured output) |
| `analysis.txt`, `error_budget.txt` | `make figures` (`tc_plot_design_analysis` writes `analysis.txt`, `tc_plot_error_budget` writes `error_budget.txt`) | analytical design numbers |
| `ablations.txt` / `ablations.mat` | `make ablation` (seeds 201–260) | final-design paired ablations, Holm-adjusted |
| `robustness.{txt,mat}` | `make robustness` (seeds 5000–5029) | rates, conventions, start angle, mismatch, friction, HB, sensors |
| `calibration_study.txt`, `calib_fit_candidate.m.txt` | `tc_setup(); tc_calibration_study(1:200, 5000:5199, false)` (captured output; the `false` keeps it from rewriting `core/tc_calibration_table.m`; `make calibration` passes `true` and was NOT used). `calib_fit_candidate.m.txt` is from the earlier run that wrote the table and was not regenerated | held-out Brier and the (unadopted) fitted map |
| `time.txt` | `make timing` (`tc_timing_study`) | cost per controller call |
| `test_full.txt` | `make test` = `tc_setup(); tc_test_all()` (captured output) | 25/25 checks |
| `independent_check.txt`, `indep_1.mat` ... `indep_5.mat` | `tc_independent_campaign(f, 100)` for f = 1..5 (five processes), then `tc_independent_report()`; `make independent` runs the same calls in one process | independent check: 5 families x 100 cases x (20, 100, 1000 Hz) x (new, held flag) |
| `start_angle_miss.txt`, and `plots/start_angle_miss.{png,pdf}` | `make start-angle` = `tc_setup(); tc_study_start_angle_miss();` | first-pulse analysis for the 80 deg start; see `docs/START_ANGLE_FINDING.md` |
| `report_scenarios.txt`, `plots/report_scenarios.{png,pdf}` | `make report-figures` = `tc_setup(); tc_plot_report_schematic(); tc_plot_report_blockdiagram(); tc_plot_report_scenarios(); tc_plot_report_estimation(); tc_plot_report_final_eval();` (this file: `tc_plot_report_scenarios()` alone) | scenarios A, B, C at 100 Hz: first hit, pointing RMS, beam error after the first hit, peaks (report Fig. 5) |
| `report_estimation.txt`, `plots/report_estimation.{png,pdf}` | `tc_setup(); tc_plot_report_estimation();` (also part of `make report-figures`) | nowcast and t_p-ahead prediction error with the filter's 1 sigma for A, B, C; the replay of the estimator is checked against `turret_controller` (difference 0) (report Fig. 4) |
| `report_final_eval.txt`, `plots/report_final_eval.{png,pdf}` | `tc_setup(); tc_plot_report_final_eval();` (also part of `make report-figures`) | percentiles and per-family means from `heldout_final.mat` and `indep_1..5.mat`; no simulation (report Fig. 6) |
| `plots/report_schematic.{png,pdf}`, `plots/report_blockdiagram.{png,pdf}` | `tc_setup(); tc_plot_report_schematic();` and `tc_plot_report_blockdiagram();` (also part of `make report-figures`) | engineering schematic and block diagram sized for IEEE (report Figs. 1 and 2); drawings, no data |
| `eval_adapter_misses.txt` | `tc_setup(); eval_adapter_misses()` (re-runs the cases flagged MISS in `eval_adapter_sweep.txt`) | every pulse, with miss distance, of the two sweep runs that missed a pulse |
| `eval_adapter_sweep.txt` | `tc_setup(); T = eval_adapter_sweep();` (captured output) | A, B, C, MC201-203 x start 0 / 0.3 rad / 80 deg x 20 / 100 / 1000 Hz |
| `tc_study_los_linearization.{txt,mat}` | `make course-los` | LOS map linearisation, noise to cross-range error; reads (never rewrites) `error_budget.txt` |
| `tc_study_plant_response.txt` | `make course-plant` | impulse/step response of G(s) from the poles, RK4 cross-check; also writes `docs/ANALOGY_TABLE.md` |
| `tc_study_bode_asymptotes.txt` | `make course-bode` | asymptotic Bode of G(s) against the exact response |
| `tc_study_sensitivity.{txt,mat}` | `make course-sens` | S, T, KS of the final loop; bandwidth, peaks, noise argument |
| `tc_study_specs.{txt,mat}` | `make course-specs` | closed-loop step specifications vs dominant-pole estimates and assumed targets |
| `tc_study_uncertainty_margins.{txt,mat}` | `make course-margins` | PM/GM/crossover/Routh limits for +-20 % Kt, Ke, J, b and all 16 corners |
| `tc_study_compensator_compare.{txt,mat}` | `make course-comp` | P vs lecture-procedure lead vs the PI + roll-off (linear plant only) |
| `tc_study_feedforward_vs_feedback.{txt,mat}` | `make course-ff` | feedback only / feed-forward only / both, nominal and +-20 % |
| `tc_study_delay_bandwidth.{txt,mat}` | `make course-delay` | PM and safe crossover versus loop delay |
| `tc_study_sampling.{txt,mat}` | `make course-sampling` | radar Nyquist, LOS content, ZOH, call rate vs resonance |
| `tc_study_design_iterations.txt` | `make course-iter` | verification log for `docs/DESIGN_ITERATIONS.md` |

`make course` runs all eleven `tc_study_*` functions (about 100 s on the machine used here). They use only the polynomial plant and loop
(no Control Toolbox, no simulation), read `core/tc_params.m` without editing it, and write only into `outputs/`, `plots/` and `docs/`.
The paper macros and the two small course tables (`report/generated/course_spec_table.tex`, `course_margin_table.tex`) are produced from the
`.mat` files by `make export` (`tc_export_numbers`) through `studies/tc_course_macros.m`; `make export` also rewrites the other files in `report/generated/`.

## plots/

`make figures` (= `tc_plot_block_diagram(); tc_plot_error_budget(); A = tc_plot_design_analysis(); tc_plot_scenarios({'B','C','D6'})`) rewrote `closed_loop`, `design_analysis`, `error_budget`, `root_locus`, `scenario_B`, `scenario_C`, `scenario_D6` on 2026-11-15. The committed `scenario_B/C/D6` and `closed_loop` images differed from what the current design draws (e.g. old scenario B pulses at about 1.2, 2.2, 3.2 s; current first pulse 0.94 s as in `suite_100Hz.txt`), so the regenerated ones replace them.
