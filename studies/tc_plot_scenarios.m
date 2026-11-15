function tc_plot_scenarios(names, opts)
% TC_PLOT_SCENARIOS  Four-panel closed-loop trace per scenario -> plots/scenario_<name>.{pdf,png}
%   panels: elevation (truth / estimate / reference / turret) | cross-range error with +-R_eng and pulses
%           motor voltage | hit probability.   Blackouts are shaded.
  here = fileparts(mfilename('fullpath'));  addpath(fileparts(here));  tc_setup();
  if nargin < 1, names = {'B', 'C', 'D6'}; end
  if nargin < 2, opts = struct(); end
  K = tc_params();
  for q = 1:numel(names)
    o = tc_run_engagement(names{q}, opts);  L = o.log;  P = o.pulses;
    [~, ~, ~, ~, info] = target_motion_generator(0, names{q});
    h = figure('visible', 'off', 'position', [50 50 1000 760], 'color', 'w');
    subplot(4,1,1);  plot(L.t, L.th_true*180/pi, 'k', L.t, L.th_hat*180/pi, 'b--', L.t, L.y_d*180/pi, 'g', L.t, L.th*180/pi, 'r');
    shade(info.blackouts);  grid on;  ylabel('elevation [deg]');  title(sprintf('Scenario %s (%.0f Hz controller)', names{q}, o.opts.f_ctrl));
    legend({'target', 'estimate', 'reference y_d', 'turret'}, 'location', 'eastoutside');
    subplot(4,1,2);  plot(L.t, L.err_m, 'b');  hold on;  plot([0 10], K.r_eng*[1 1], 'r:', [0 10], -K.r_eng*[1 1], 'r:');
    if ~isempty(P), plot(P(:,1), P(:,4), 'kv', 'markerfacecolor', 'y', 'markersize', 8); end
    hold off;  ylim([-6 6]);  shade(info.blackouts);  grid on;  ylabel('cross-range error [m]');
    legend({'target - beam', '+-R_{eng}', '', 'pulses'}, 'location', 'eastoutside');
    subplot(4,1,3);  plot(L.t, L.V, 'm');  ylim([-650 650]);  shade(info.blackouts);  grid on;  ylabel('V_m [V]');
    subplot(4,1,4);  plot(L.t, L.p_hit, 'k');  ylim([0 1.05]);  shade(info.blackouts);  grid on;  ylabel('P_{hit}');  xlabel('t [s]');
    tc_save_figure(h, ['scenario_' names{q}]);
  end
end

function shade(bo)
  yl = ylim;
  for j = 1:size(bo, 1)
    patch([bo(j,1) bo(j,1)+bo(j,2) bo(j,1)+bo(j,2) bo(j,1)], [yl(1) yl(1) yl(2) yl(2)], 'k', 'facecolor', [0.82 0.82 0.82], 'facealpha', 0.6, 'edgecolor', 'none');
  end
  ylim(yl);
end
