function m = tc_engagement_metrics(L, pulses, K)
% TC_ENGAGEMENT_METRICS  Scalar summary of one engagement (all lengths in metres).
%   success      at least one pulse within R_eng            t_first_hit  time of the first such pulse
%   point_rms_m  cross-range pointing error RMS over t >= 2 s   frac_in_Reng  fraction of t >= 2 s inside R_eng
  m.n_pulses = size(pulses, 1);
  m.n_hits = sum(pulses(:,3));
  m.success = m.n_hits > 0;
  m.t_first_hit = NaN;
  if m.success, m.t_first_hit = pulses(find(pulses(:,3) > 0.5, 1), 1); end
  w = L.t >= 2;
  m.point_rms_m = sqrt(mean(L.err_m(w).^2));
  m.frac_in_Reng = mean(abs(L.err_m(w)) <= K.r_eng);
  m.V_peak = max(abs(L.V));
  m.I_peak = max(abs(L.i));
  m.miss_max = 0;  if m.n_pulses > 0, m.miss_max = max(pulses(:,2)); end
end
