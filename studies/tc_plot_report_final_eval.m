function R = tc_plot_report_final_eval()
% TC_PLOT_REPORT_FINAL_EVAL  Final-evaluation figure from STORED data only (no simulation):
%   (a) empirical CDF of the first-hit time over the 1000 held-out seeds (outputs/heldout_final.mat)
%   (b) empirical CDF of the pointing RMS per engagement over the same seeds
%   (c) independent campaign (outputs/indep_1..5.mat): mean first-hit time per family at 20, 100 and 1000 Hz
%       (the held-flag configurations are identical to the new-sample ones, see outputs/independent_check.txt)
% Writes outputs/report_final_eval.txt (the numbers quoted in the report) and plots/report_final_eval.{pdf,png}.
% Command: make report-figures
  here = fileparts(mfilename('fullpath'));  root = fileparts(here);  addpath(root);  tc_setup();
  rd = fullfile(root, 'outputs');
  A = load(fullfile(rd, 'heldout_final.mat'));  H = A.R;  tf = H.t_first(H.success > 0.5);  rm = H.rms;
  fams = {'weave', 'step', 'bang-bang', 'arc', 'pop-up'};  I = cell(1, 5);
  for q = 1:5, B = load(fullfile(rd, sprintf('indep_%d.mat', q)));  I{q} = B.R;  end
  rates = I{1}.rates;  convs = I{1}.convs;
  fid = fopen(fullfile(rd, 'report_final_eval.txt'), 'w');  w = @(varargin) fprintf(fid, varargin{:});
  w('octave-cli --quiet --eval "tc_setup(); tc_plot_report_final_eval();"   (make report-figures)\n');
  w('Numbers behind the final-evaluation figure; data: outputs/heldout_final.mat and outputs/indep_1..5.mat (no simulation run here).\n\n');
  w('held-out MC5000-5999 (n = %d): first hit  min %.3f  median %.3f  mean %.3f  p95 %.3f  max %.3f s;  share with first hit <= 1.5 s: %.1f %%\n', ...
    numel(H.seeds), min(tf), median(tf), mean(tf), tc_percentile(tf, 95), max(tf), 100*mean(tf <= 1.5));
  w('held-out pointing RMS per run: median %.3f  p95 %.3f  max %.3f m;  seeds with RMS > 1 m: %d of %d\n\n', median(rm), tc_percentile(rm, 95), max(rm), sum(rm > 1), numel(rm));
  w('independent campaign, mean first-hit time [s] and intercepts per family (new-sample flag; held flag identical)\n');
  w('%-10s', 'family');  for r = 1:numel(rates), w(' %8s %9s', sprintf('%dHz', rates(r)), 'icpt'); end;  w('\n');
  M = zeros(5, numel(rates));  N = zeros(5, numel(rates));
  for q = 1:5
    w('%-10s', fams{q});
    for r = 1:numel(rates)
      j = (r-1)*numel(convs) + 1;  s = I{q}.success(j, :);  tfq = I{q}.t_first(j, s);  M(q, r) = mean(tfq);  N(q, r) = sum(s);
      w(' %8.3f %6d/%-3d', M(q, r), N(q, r), numel(s));
    end
    w('\n');
  end
  w('\nindependent campaign pooled over the five families, first hit per call rate [s]:\n');
  PM = zeros(1, numel(rates));  PP = PM;  PX = PM;
  for r = 1:numel(rates)
    j = (r-1)*numel(convs) + 1;  v = [];  for q = 1:5, v = [v, I{q}.t_first(j, I{q}.success(j, :))]; end %#ok<AGROW>
    PM(r) = mean(v);  PP(r) = tc_percentile(v, 95);  PX(r) = max(v);
    w('  %5d Hz: n %d  mean %.3f  p95 %.3f  max %.3f\n', rates(r), numel(v), PM(r), PP(r), PX(r));
  end
  fclose(fid);
  % ---- figure ----
  h = figure('visible', 'off', 'position', [50 50 700 190], 'color', 'w');
  set(h, 'defaultaxesfontsize', 7, 'defaulttextfontsize', 7);
  ax = axes('position', [0.07 0.23 0.24 0.60]);  hold on;
  [x, y] = ecdf_(tf);  stairs(x, y, 'k-', 'linewidth', 1.0);  plot(mean(tf)*[1 1], [0 1], 'b--', 'linewidth', 0.9);  plot(tc_percentile(tf, 95)*[1 1], [0 1], 'r:', 'linewidth', 1.1);
  xlabel('first-hit time [s]');  ylabel('fraction of seeds');  title('(a) held-out, 1000 seeds', 'fontsize', 7);  grid on;  box on;  xlim([0.4 2.6]);  ylim([0 1.02]);
  legend({'CDF', 'mean', '95th pct'}, 'location', 'northwest', 'fontsize', 6);  set(ax, 'layer', 'top');
  ax = axes('position', [0.40 0.23 0.24 0.60]);  hold on;
  [x, y] = ecdf_(rm);  stairs(x, y, 'k-', 'linewidth', 1.0);  plot(median(rm)*[1 1], [0 1], 'b--', 'linewidth', 0.9);  plot(tc_percentile(rm, 95)*[1 1], [0 1], 'r:', 'linewidth', 1.1);
  xlabel('pointing RMS per run [m]');  title('(b) held-out, 1000 seeds', 'fontsize', 7);  grid on;  box on;  xlim([0.3 1.3]);  ylim([0 1.02]);
  legend({'CDF', 'median', '95th pct'}, 'location', 'southeast', 'fontsize', 6);  set(ax, 'layer', 'top');
  ax = axes('position', [0.74 0.23 0.24 0.60]);  hold on;
  mk = {'o', 's', '^'};  fc = {'w', [0.6 0.6 0.6], 'k'};
  for r = 1:numel(rates), plot(1:5, M(:, r), mk{r}, 'color', 'k', 'markerfacecolor', fc{r}, 'markersize', 4, 'linestyle', '-', 'linewidth', 0.6); end
  set(ax, 'xtick', 1:5, 'xticklabel', {'weave', 'step', 'bang', 'arc', 'pop-up'}, 'layer', 'top');  ylabel('mean first hit [s]');  title('(c) independent, 5 x 100 cases', 'fontsize', 7);
  grid on;  box on;  xlim([0.6 5.4]);  ylim([0.8 2.5]);  legend(arrayfun(@(r) sprintf('%d Hz', r), rates, 'uniformoutput', false), 'location', 'northeast', 'fontsize', 6);
  text(0.65, 0.84, 'all 500/500 intercepted at every rate', 'fontsize', 6);
  tc_save_figure(h, 'report_final_eval');
  R = struct('heldout_first', tf, 'heldout_rms', rm, 'indep_mean', M, 'indep_pooled_mean', PM, 'indep_pooled_p95', PP);
  fprintf('wrote outputs/report_final_eval.txt and plots/report_final_eval.{pdf,png}\n');
end

function [x, y] = ecdf_(v)
  v = sort(v(:))';  n = numel(v);  x = [v(1), v];  y = [0, (1:n)/n];
end
