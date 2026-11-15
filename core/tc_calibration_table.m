function T = tc_calibration_table()
% TC_CALIBRATION_TABLE  Map raw -> calibrated hit probability.  SHIPPED AS THE IDENTITY.
% studies/tc_calibration_study.m fitted an isotonic map on tuning seeds 1..200 and scored it on the
% disjoint range 5000..5199: held-out Brier 0.00353 raw vs 0.00353 calibrated (no gain, and the
% 0.99-0.999 bin moved away from the observed rate), so the map is NOT adopted.  The fitted
% candidate and the reliability table are kept in outputs/ (calib_fit_candidate.m.txt,
% calibration_study.txt).  Rerun `make calibration` after any change to the estimator; it overwrites
% this file, so re-check the held-out Brier before keeping the result.
  T.kx = [0 1];
  T.ky = [0 1];
end
