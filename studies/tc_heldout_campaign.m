function R = tc_heldout_campaign(family, seeds, tag, opts)
% TC_HELDOUT_CAMPAIGN  Closed-loop campaign on a held-out seed range with the FROZEN design; writes
% outputs/<tag>.mat (per-seed data) and outputs/<tag>.txt (summary).  Censored runs (no hit)
% are listed separately; the failure bound is the Wilson limit (docs/PROTOCOL.md).
%   tc_heldout_campaign('MC', 5000:5299, 'heldout_MC5000_5299')
  here = fileparts(mfilename('fullpath'));  addpath(fileparts(here));  tc_setup();
  if nargin < 4, opts = struct(); end
  tc_settings('clear');
  R = tc_campaign_run(family, seeds, opts, true);
  s = R.sum;  rd = fullfile(fileparts(here), 'outputs');
  if ~exist(rd, 'dir'), mkdir(rd); end
  fid = fopen(fullfile(rd, [tag '.txt']), 'w');
  fprintf(fid, 'family %s seeds %d..%d (n = %d)  opts: %s\n', family, seeds(1), seeds(end), s.n, disp_opts(opts));
  fprintf(fid, 'intercepted %d/%d  Wilson95 [%.4f, %.4f]  failure bound < %.3f %%\n', s.intercepted, s.n, s.success_ci(1), s.success_ci(2), 100*s.failure_upper);
  fprintf(fid, 'pulses %d, hits %d, hit rate %.4f  Wilson95 [%.4f, %.4f]\n', s.pulses, s.pulse_hits, s.pulse_hit_rate, s.pulse_ci(1), s.pulse_ci(2));
  fprintf(fid, 'first hit (successful runs): mean %.3f s, p95 %.3f s, max %.3f s\n', s.t_first_mean, s.t_first_p95, s.t_first_max);
  fprintf(fid, 'pointing RMS: median %.3f m, p95 %.3f m;  time in R_eng %.2f %%;  V_peak %.0f V\n', s.rms_median, s.rms_p95, 100*s.in_reng_mean, s.V_peak);
  cens = seeds(~R.success);
  fprintf(fid, 'censored (no hit) seeds: %s\n', mat2str(cens));
  fprintf(fid, 'worst miss distance over all pulses: %.3f m\n', max(R.miss_max));
  fclose(fid);
  save('-text', fullfile(rd, [tag '.mat']), 'R');
  type(fullfile(rd, [tag '.txt']));
end

function s = disp_opts(o)
  f = fieldnames(o);  s = '{}';
  if ~isempty(f), s = strjoin(cellfun(@(k) sprintf('%s=%s', k, mat2str(o.(k))), f', 'UniformOutput', false), ', '); end
end
