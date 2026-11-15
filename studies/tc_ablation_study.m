function T = tc_ablation_study(seeds, which, tag)
% TC_ABLATION_STUDY  One-factor PAIRED ablations against the frozen design on identical seeds.
%   T = tc_ablation_study(201:260)                  all variants (exploration range, see docs/PROTOCOL.md)
%   T = tc_ablation_study(201:260, {'classes'})
% Each variant is a tc_settings override, cleared afterwards.  For first-hit time and pointing RMS
% the paired difference (variant - frozen) comes with a percentile-bootstrap 95 % CI and a
% sign-flip permutation p; p-values are Holm-adjusted within each metric family.
% Runs with no hit are censored at 10 s for the first-hit comparison.
  here = fileparts(mfilename('fullpath'));  addpath(fileparts(here));  tc_setup();
  if nargin < 1, seeds = 201:260; end
  if nargin < 3, tag = 'ablations'; end
  V = {'classes',     struct('phit_model', 'classes'); ...
       'no_adapt_q',  struct('adapt_q', false); ...
       'lead_on',     struct('lead_hold', true); ...
       'guard_on',    struct('sweep_guard', 0.5); ...
       'no_freeze',   struct('freeze_acq', false); ...
       'brake_slew',  struct('acq_decel', 1.5); ...
       'fast_fire',   struct('n_upd_min', 4); ...
       'psure_flat',  struct('p_sure_after', 0.99); ...
       'p_dev_10',    struct('p_dev', 10); ...
       'p_dev_30',    struct('p_dev', 30); ...
       'adapt_ff',    struct('adapt_ff', true); ...
       'aim_tp',      struct('aim_ahead', 0.5); ...
       'rate_20Hz',   struct()};
  if nargin > 1 && ~isempty(which), V = V(ismember(V(:,1), which), :); end
  tc_settings('clear');
  base = tc_campaign_run('MC', seeds, struct('f_ctrl', 100), false);
  fprintf('frozen design on %d seeds: %d/%d intercepted, first hit %.3f s, RMS median %.3f m, pulse misses %d\n', ...
          numel(seeds), base.sum.intercepted, base.sum.n, base.sum.t_first_mean, base.sum.rms_median, sum(base.n_pulses - base.n_hits));
  nv = size(V, 1);  Rv = cell(1, nv);  st = cell(1, nv);  sr = cell(1, nv);
  for q = 1:nv
    tc_settings('set', V{q,2});  opts = struct('f_ctrl', 100);
    if strcmp(V{q,1}, 'rate_20Hz'), opts.f_ctrl = 20; end
    Rv{q} = tc_campaign_run('MC', seeds, opts, false);  tc_settings('clear');
    st{q} = tc_paired_stats(cens(base.t_first), cens(Rv{q}.t_first));
    sr{q} = tc_paired_stats(base.rms, Rv{q}.rms);
    fprintf('  done %s\n', V{q,1});  fflush(stdout);
  end
  pt = holm(cellfun(@(x) x.p, st));  pr = holm(cellfun(@(x) x.p, sr));
  fprintf('\n%-11s %5s %4s | %9s %19s %7s | %9s %19s %7s\n', 'variant', 'icpt', 'miss', 'd t_first', '95% CI', 'p_Holm', 'd RMS [m]', '95% CI', 'p_Holm');
  T = struct('name', {}, 'R', {}, 'dt', {}, 'dr', {}, 'pt', {}, 'pr', {});
  for q = 1:nv
    R = Rv{q};
    fprintf('%-11s %2d/%-2d %4d | %+9.3f [%+7.3f,%+7.3f] %7.3f | %+9.3f [%+7.3f,%+7.3f] %7.3f\n', V{q,1}, R.sum.intercepted, R.sum.n, ...
            sum(R.n_pulses - R.n_hits), st{q}.mean, st{q}.ci(1), st{q}.ci(2), pt(q), sr{q}.mean, sr{q}.ci(1), sr{q}.ci(2), pr(q));
    T(end+1) = struct('name', V{q,1}, 'R', R, 'dt', st{q}, 'dr', sr{q}, 'pt', pt(q), 'pr', pr(q)); %#ok<AGROW>
  end
  rd = fullfile(fileparts(here), 'outputs');  if ~exist(rd, 'dir'), mkdir(rd); end
  save('-text', fullfile(rd, [tag '.mat']), 'T', 'base', 'seeds');
end

function x = cens(t), x = t;  x(~isfinite(x)) = 10; end      % no hit within the 10 s run

function pa = holm(p)
% Holm step-down adjustment (controls the family-wise error rate).
  m = numel(p);  [ps, ix] = sort(p);  adj = zeros(1, m);  run = 0;
  for k = 1:m, run = max(run, (m - k + 1)*ps(k));  adj(k) = min(run, 1); end
  pa = zeros(1, m);  pa(ix) = adj;
end
