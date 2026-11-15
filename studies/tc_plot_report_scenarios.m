function R = tc_plot_report_scenarios()
% TC_PLOT_REPORT_SCENARIOS  Scenarios A, B, C at 100 Hz in one report figure (three columns).
%   row 1: target and turret elevation (deg), pulses marked on the turret curve
%   row 2: cross-range beam error r*sin(theta_target - theta_turret) with +-R_eng, pulses marked (filled = hit)
%   row 3: motor voltage V_m (V) with the +-V_lim clamp
% Blackout of scenario C shaded.  Uses tc_run_engagement only (deterministic named scenarios).
% Writes outputs/report_scenarios.txt, plots/report_scenarios.{pdf,png}.   Command: make report-figures
  here = fileparts(mfilename('fullpath'));  root = fileparts(here);  addpath(root);  tc_setup();
  tc_settings('clear');  K = tc_params();  names = {'A', 'B', 'C'};
  rd = fullfile(root, 'outputs');  fid = fopen(fullfile(rd, 'report_scenarios.txt'), 'w');  w = @(varargin) fprintf(fid, varargin{:});
  w('octave-cli --quiet --eval "tc_setup(); tc_plot_report_scenarios();"   (make report-figures)\n');
  w('Scenarios A, B, C, 100 Hz, turret starts at 0 deg, frozen design (tc_run_engagement).\n\n');
  w('%-3s %9s %6s %6s %9s %9s %9s %8s %8s %9s\n', 'scn', 'first_hit', 'pulses', 'hits', 'rms_m', 'max_after', 'in_Reng%', 'V_peak', 'I_peak', 'miss_max');
  h = figure('visible', 'off', 'position', [50 50 700 250], 'color', 'w');
  set(h, 'defaultaxesfontsize', 7, 'defaulttextfontsize', 7);
  R = struct();
  for c = 1:3
    scn = names{c};  o = tc_run_engagement(scn, struct('f_ctrl', 100));  L = o.log;  P = o.pulses;  m = o.metrics;
    [~, ~, ~, ~, info] = target_motion_generator(0, scn);  bo = info.blackouts;
    after = L.t >= m.t_first_hit + 0.5;
    w('%-3s %9.2f %6d %6d %9.3f %9.3f %9.1f %8.0f %8.1f %9.3f\n', scn, m.t_first_hit, m.n_pulses, m.n_hits, m.point_rms_m, max(abs(L.err_m(after))), 100*m.frac_in_Reng, m.V_peak, m.I_peak, m.miss_max);
    R.(scn) = struct('first_hit', m.t_first_hit, 'rms', m.point_rms_m, 'pulses', P);
    x0 = 0.075 + (c-1)*0.315;  wd = 0.27;
    ax1 = axes('position', [x0 0.65 wd 0.20]);  hold on;  shade(bo, [0 65]);
    plot(L.t, L.th_true*180/pi, 'k-', 'linewidth', 0.9);  plot(L.t, L.th*180/pi, 'r--', 'linewidth', 0.9);
    if ~isempty(P), kk = arrayfun(@(t) find(L.t >= t - 1e-9, 1), P(:,1)); plot(P(:,1), L.th(kk)*180/pi, 'kv', 'markerfacecolor', 'y', 'markersize', 4); end
    ylim([0 65]);  xlim([0 10]);  grid on;  box on;  set(ax1, 'xticklabel', [], 'layer', 'top');  title(sprintf('Scenario %s', scn), 'fontsize', 8);
    if c == 1, ylabel('elev., deg'); end
    ax2 = axes('position', [x0 0.41 wd 0.20]);  hold on;  shade(bo, [-6 6]);
    plot(L.t, L.err_m, 'b-', 'linewidth', 0.8);  hline(K.r_eng);  hline(-K.r_eng);
    if ~isempty(P)
      hit = P(:,3) > 0.5;  plot(P(hit,1), P(hit,4).*P(hit,2), 'kv', 'markerfacecolor', 'y', 'markersize', 4);
      if any(~hit), plot(P(~hit,1), P(~hit,4).*P(~hit,2), 'kv', 'markerfacecolor', 'w', 'markersize', 4); end
    end
    ylim([-6 6]);  xlim([0 10]);  grid on;  box on;  set(ax2, 'xticklabel', [], 'layer', 'top', 'ytick', [-6 -3 0 3 6]);  if c == 1, ylabel('error, m'); end
    ax3 = axes('position', [x0 0.17 wd 0.20]);  hold on;  shade(bo, [-700 700]);
    plot(L.t, L.V, 'm-', 'linewidth', 0.8);  hline(K.V_lim);  hline(-K.V_lim);
    ylim([-700 700]);  xlim([0 10]);  grid on;  box on;  set(ax3, 'layer', 'top', 'ytick', [-400 0 400]);  xlabel('t [s]');  if c == 1, ylabel('V_m, V'); end
  end
  fclose(fid);
  tc_save_figure(h, 'report_scenarios');
  fprintf('wrote outputs/report_scenarios.txt and plots/report_scenarios.{pdf,png}\n');
end

function shade(bo, yl)
  for j = 1:size(bo, 1)
    patch([bo(j,1) bo(j,1)+bo(j,2) bo(j,1)+bo(j,2) bo(j,1)], [yl(1) yl(1) yl(2) yl(2)], [0.85 0.85 0.85], 'edgecolor', 'none');
  end
end

function hline(y)
  plot([0 10], [y y], '--', 'color', [0.8 0 0], 'linewidth', 1.0);
end
