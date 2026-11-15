# All targets run GNU Octave from the repository root.  No MATLAB anywhere.
OCT = octave-cli --quiet --eval

# ---- verification ---------------------------------------------------------------------------------------------
test-quick:   ; $(OCT) "tc_setup(); tc_test_all(true)"
test:         ; $(OCT) "tc_setup(); tc_test_all()"
eval-check:   ; $(OCT) "tc_setup(); eval_syntax_check(); eval_adapter_test()"
timing:       ; $(OCT) "tc_setup(); tc_timing_study()"

# ---- scenarios and design -------------------------------------------------------------------------------------
scenarios:    ; $(OCT) "tc_setup(); T = tc_scenario_suite();"
design:       ; $(OCT) "tc_setup(); R = tc_design_report();"
quick:        ; $(OCT) "tc_setup(); tc_test_all(true); tc_scenario_suite(); R = tc_design_report();"
start-angle:  ; $(OCT) "tc_setup(); tc_study_start_angle_miss();"

# ---- campaigns (long runs; each writes outputs/<name>.{txt,mat}) ----------------------------------------------
heldout:      ; $(OCT) "tc_setup(); tc_heldout_campaign('MC', 5000:5299, 'heldout_MC5000_5299')"
heldout-merge: ; $(OCT) "tc_setup(); tc_merge_heldout({'heldout_MC5000_5299','heldout_MC5300_5649','heldout_MC5650_5999'}, 'heldout_final');"
independent:  ; $(OCT) "tc_setup(); for f = 1:5, tc_independent_campaign(f, 100); end; tc_independent_report();"
calibration:  ; $(OCT) "tc_setup(); tc_calibration_study(1:200, 5000:5199, true)"
ablation:     ; $(OCT) "tc_setup(); tc_ablation_study(201:260)"
robustness:   ; $(OCT) "tc_setup(); tc_robustness_study(5000:5029)"

# ---- course-concept studies (polynomial models only, no simulation; each writes outputs/<name>.txt and plots/<name>.{pdf,png}) ----
course-los:     ; $(OCT) "tc_setup(); tc_study_los_linearization();"
course-plant:   ; $(OCT) "tc_setup(); tc_study_plant_response();"
course-bode:    ; $(OCT) "tc_setup(); tc_study_bode_asymptotes();"
course-sens:    ; $(OCT) "tc_setup(); tc_study_sensitivity();"
course-specs:   ; $(OCT) "tc_setup(); tc_study_specs();"
course-margins: ; $(OCT) "tc_setup(); tc_study_uncertainty_margins();"
course-comp:    ; $(OCT) "tc_setup(); tc_study_compensator_compare();"
course-ff:      ; $(OCT) "tc_setup(); tc_study_feedforward_vs_feedback();"
course-delay:   ; $(OCT) "tc_setup(); tc_study_delay_bandwidth();"
course-sampling: ; $(OCT) "tc_setup(); tc_study_sampling();"
course-iter:    ; $(OCT) "tc_setup(); tc_study_design_iterations();"
course:         ; $(OCT) "tc_setup(); tc_study_los_linearization(); tc_study_plant_response(); tc_study_bode_asymptotes(); tc_study_sensitivity(); tc_study_specs(); tc_study_uncertainty_margins(); tc_study_compensator_compare(); tc_study_feedforward_vs_feedback(); tc_study_delay_bandwidth(); tc_study_sampling(); tc_study_design_iterations();"

# ---- report ---------------------------------------------------------------------------------------------------
figures:      ; $(OCT) "tc_setup(); tc_plot_block_diagram(); tc_plot_error_budget(); A = tc_plot_design_analysis(); tc_plot_scenarios({'B','C','D6'});"
report-figures: ; $(OCT) "tc_setup(); tc_plot_report_schematic(); tc_plot_report_blockdiagram(); tc_plot_report_scenarios(); tc_plot_report_estimation(); tc_plot_report_final_eval();"
export:       ; $(OCT) "tc_setup(); tc_export_numbers();"

# Overleaf bundle: main.tex, generated/*.tex and plots/ holding only the PDFs that main.tex includes
# (main.tex finds them through \graphicspath{{../plots/}{plots/}}).
REPORT_FIGS = $(subst },,$(subst {,,$(shell grep -oE 'includegraphics(\[[^]]*\])?\{[^}]+\}' report/main.tex | grep -oE '\{[^}]+\}$$')))
report-zip:
	rm -rf build/overleaf build/overleaf.zip
	mkdir -p build/overleaf/generated build/overleaf/plots
	cp report/main.tex build/overleaf/
	cp report/generated/*.tex build/overleaf/generated/
	cp $(addprefix plots/,$(REPORT_FIGS)) build/overleaf/plots/
	cd build/overleaf && zip -qrX ../overleaf.zip main.tex generated plots
	rm -rf build/overleaf
	@echo "wrote build/overleaf.zip ($(words $(REPORT_FIGS)) figures)"

.PHONY: test-quick test eval-check timing scenarios design quick start-angle
.PHONY: heldout heldout-merge independent calibration ablation robustness
.PHONY: course course-los course-plant course-bode course-sens course-specs course-margins course-comp course-ff course-delay course-sampling course-iter
.PHONY: figures report-figures export report-zip
