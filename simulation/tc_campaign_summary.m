function s = tc_campaign_summary(R)
% TC_CAMPAIGN_SUMMARY  Headline numbers with Wilson intervals; censored runs (no hit) are listed
% separately instead of being averaged away.
  n = numel(R.success);  k = sum(R.success);
  s.n = n;  s.intercepted = k;  s.success_ci = tc_wilson_interval(k, n);
  s.failure_upper = 1 - s.success_ci(1);                  % one-sided-style bound on failure rate
  s.pulses = sum(R.n_pulses);  s.pulse_hits = sum(R.n_hits);
  s.pulse_hit_rate = s.pulse_hits / max(s.pulses, 1);
  s.pulse_ci = tc_wilson_interval(s.pulse_hits, max(s.pulses, 1));
  tf = R.t_first(R.success);
  s.t_first_mean = mean(tf);  s.t_first_p95 = tc_percentile(tf, 95);  s.t_first_max = max([tf, 0]);
  s.rms_median = median(R.rms);  s.rms_p95 = tc_percentile(R.rms, 95);
  s.in_reng_mean = mean(R.in_reng);  s.V_peak = max(R.V_peak);
end
