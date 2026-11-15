function R = tc_campaign_run(family, seeds, opts, verbose)
% TC_CAMPAIGN_RUN  Closed-loop campaign over a list of seeds (identical seeds across variants
% give PAIRED comparisons: same trajectory, blackouts and noise).
%   R = tc_campaign_run('MC', 5000:5299, struct('f_ctrl', 100))
% Returns per-run metrics (R.t_first, R.rms, ...) and a summary R.sum.
  if nargin < 3, opts = struct(); end
  if nargin < 4, verbose = true; end
  opts.keep_log = false;
  n = numel(seeds);
  R = struct('seeds', seeds(:)', 'success', false(1,n), 't_first', NaN(1,n), 'rms', NaN(1,n), ...
             'n_pulses', zeros(1,n), 'n_hits', zeros(1,n), 'in_reng', NaN(1,n), 'V_peak', NaN(1,n), ...
             'miss_max', NaN(1,n));
  tic;
  for q = 1:n
    o = tc_run_engagement(sprintf('%s%d', family, seeds(q)), opts);  m = o.metrics;
    R.success(q) = m.success;  R.t_first(q) = m.t_first_hit;  R.rms(q) = m.point_rms_m;
    R.n_pulses(q) = m.n_pulses;  R.n_hits(q) = m.n_hits;  R.in_reng(q) = m.frac_in_Reng;
    R.V_peak(q) = m.V_peak;  R.miss_max(q) = m.miss_max;
    if verbose && mod(q, 25) == 0, fprintf('  %d/%d  (%.0f s)\n', q, n, toc); fflush(stdout); end
  end
  R.sum = tc_campaign_summary(R);
end
