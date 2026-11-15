function T = tc_scenario_suite(opts)
% TC_SCENARIO_SUITE  The nine named scenarios, one table row each (printed and returned).
  if nargin < 1, opts = struct(); end
  names = {'A','B','C','D1','D2','D3','D4','D5','D6'};
  T = struct('name', names, 'm', []);
  fprintf('%-4s %9s %7s %9s %9s %8s\n', 'scn', 'first hit', 'hits', 'RMS [m]', 'in R_eng', 'V_peak');
  for q = 1:numel(names)
    o = tc_run_engagement(names{q}, opts);  m = o.metrics;  T(q).m = m;
    fprintf('%-4s %8.2fs %3d/%-3d %9.2f %8.1f%% %8.0f\n', names{q}, m.t_first_hit, m.n_hits, ...
            m.n_pulses, m.point_rms_m, 100*m.frac_in_Reng, m.V_peak);
  end
end
