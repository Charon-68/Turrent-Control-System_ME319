function R = tc_plot_report_estimation()
% TC_PLOT_REPORT_ESTIMATION  Nowcast and t_p-ahead prediction error versus time for scenarios A, B and C (report figure).
%
% The estimator does not depend on the turret, so it is replayed here with the same public functions the controller calls
% (tc_session_start, tc_filter_update, tc_jerk_model_transition, tc_los_derivatives), at the controller's 100 Hz call times.
% The replay is checked against turret_controller's own nowcast (its dbg.x_hat) before anything is plotted.
%   row 1: cross-range error of the nowcast bearing, r*(theta_hat - theta_true), with the filter's own +-1 sigma
%   row 2: error of the t_p-ahead prediction r*(theta_pred(t+t_p) - theta_true(t+t_p)),  theta_pred = [1 t_p t_p^2/2 t_p^3/6]*D
%   row 3: velocity error norm |v_hat - v_true| with the filter's 1 sigma  sqrt(P_vx + P_vy)
% Writes outputs/report_estimation.txt, plots/report_estimation.{pdf,png}.   Command: make report-figures
  here = fileparts(mfilename('fullpath'));  root = fileparts(here);  addpath(root);  tc_setup();
  tc_settings('clear');  K = tc_params();  names = {'A', 'B', 'C'};  f = 100;  T = 1/f;  n = round(10*f) + 1;
  R = struct();
  rd = fullfile(root, 'outputs');  fid = fopen(fullfile(rd, 'report_estimation.txt'), 'w');
  w = @(varargin) fprintf(fid, varargin{:});
  w('octave-cli --quiet --eval "tc_setup(); tc_plot_report_estimation();"   (make report-figures)\n');
  w('Nowcast and t_p-ahead prediction error, scenarios A, B, C, 100 Hz calls (the estimator does not see the turret).\n');
  w('Replay check: max |x_hat(replay) - x_hat(turret_controller dbg)| over the run, per scenario (state units m, m/s, m/s^2, m/s^3).\n\n');
  w('%-3s %12s | %9s %9s %9s | %9s %9s %9s | %9s %9s\n', 'scn', 'replay_diff', 'now_rms', 'now_max', 'sig_mean', 'pred_rms', 'pred_max', 'pred_blk', 'vel_rms', 'vel_sigmean');
  h = figure('visible', 'off', 'position', [50 50 700 250], 'color', 'w');
  set(h, 'defaultaxesfontsize', 7, 'defaulttextfontsize', 7);
  for c = 1:3
    scn = names{c};  [~, ~, ~, ~, info] = target_motion_generator(0, scn);  bo = info.blackouts;
    % --- controller's own nowcast (reference) ---
    clear turret_controller
    xc = zeros(8, n);
    for k = 1:n
      t = (k-1)/f;  [r, th, v] = target_motion_generator(t, scn);
      [~, ~, dbg] = turret_controller(t, r, th, v, 0, 0);  xc(:, k) = dbg.x_hat;
    end
    % --- replay with the public estimator functions ---
    S = tc_session_start(0, 0, 0);  q_cont = S.q_cont;
    tt = zeros(1, n);  ex = zeros(1, n);  sg = NaN(1, n);  ep = NaN(1, n);  ev = NaN(1, n);  sv = NaN(1, n);  xr = zeros(8, n);  on = false(1, n);
    for k = 1:n
      t = (k-1)/f;  tt(k) = t;  [r, th, v, tru] = target_motion_generator(t, scn);
      [S, ~] = tc_filter_update(S, t, r, th, v);
      tr = S.tr;
      if tr.on && (t - tr.t_upd) <= K.lost_gap
        tau = t - tr.t_upd;  [F1, Q1] = tc_jerk_model_transition(tau, 1/K.tau_j, q_cont, S.Wgl);
        F = kron(eye(2), F1);  xn = F*tr.x;  Pn = F*tr.P*F' + tr.rho*kron(eye(2), Q1);  xr(:, k) = xn;  on(k) = true;
        D = tc_los_derivatives(xn);  rr = hypot(tru(1), tru(2));  tht = atan2(tru(2), tru(1));
        ex(k) = rr*(D(1) - tht);
        px = xn(1);  py = xn(5);  rh = hypot(px, py);  g = [-py/rh^2, px/rh^2];            % d theta / d (x, y)
        Pxy = [Pn(1,1) Pn(1,5); Pn(5,1) Pn(5,5)];  sg(k) = rr*sqrt(g*Pxy*g');
        [~, ~, ~, trf] = target_motion_generator(min(t + K.tp, 10.99), scn);                % truth t_p ahead (table runs to 11 s)
        thp = [1, K.tp, K.tp^2/2, K.tp^3/6]*D;  ep(k) = rr*(thp - atan2(trf(2), trf(1)));
        ev(k) = hypot(xn(2) - tru(3), xn(6) - tru(4));  sv(k) = sqrt(Pn(2,2) + Pn(6,6));
      end
    end
    dif = max(max(abs(xr(:, on) - xc(:, on))));
    if dif > 1e-9, error('tc_plot_report_estimation: replay differs from turret_controller by %.3g for %s', dif, scn); end
    inbo = false(1, n);  for j = 1:size(bo, 1), inbo = inbo | (tt >= bo(j,1) & tt < bo(j,1) + bo(j,2)); end
    ss = on & tt >= 2 & ~inbo;                                   % steady tracking, outside blackouts
    pb = on & inbo;                                              % inside a blackout
    nr = sqrt(mean(ex(ss).^2));  nm = max(abs(ex(on & tt >= 2)));  sm = mean(sg(ss));
    pr = sqrt(mean(ep(ss).^2));  pm = max(abs(ep(on & tt >= 2)));  pbm = NaN;  if any(pb), pbm = max(abs(ep(pb))); end
    vr = sqrt(mean(ev(ss).^2));  vs = mean(sv(ss));
    w('%-3s %12.2e | %9.3f %9.3f %9.3f | %9.3f %9.3f %9.3f | %9.3f %9.3f\n', scn, dif, nr, nm, sm, pr, pm, pbm, vr, vs);
    R.(scn) = struct('t', tt, 'ex', ex, 'sg', sg, 'ep', ep, 'ev', ev, 'sv', sv, 'on', on, 'blackouts', bo, ...
                     'now_rms', nr, 'now_max', nm, 'sig_mean', sm, 'pred_rms', pr, 'pred_max', pm, 'pred_blk', pbm, 'vel_rms', vr, 'vel_sig', vs);
    if any(pb)                                                    % growth through the blackout (scenario C)
      kb = find(pb);  R.(scn).blk_t = tt(kb);  R.(scn).blk_sig_end = sg(kb(end));  R.(scn).blk_err_end = abs(ex(kb(end)));  R.(scn).blk_sig_start = sg(kb(1));
      R.(scn).blk_pred_end = abs(ep(kb(end)));
    end
    % ---- plot column ----
    x0 = 0.075 + (c-1)*0.315;  wd = 0.27;
    ax1 = axes('position', [x0 0.65 wd 0.20]);  hold on;  shade(bo, [-3.5 3.5]);  plot(tt, ex, 'k-', 'linewidth', 0.7);
    plot(tt, sg, 'b--', 'linewidth', 0.7);  plot(tt, -sg, 'b--', 'linewidth', 0.7);  hline(K.r_eng);  hline(-K.r_eng);
    ylim([-3.5 3.5]);  xlim([0 10]);  grid on;  box on;  set(ax1, 'xticklabel', [], 'layer', 'top', 'ytick', [-3 -1 0 1 3]);  title(sprintf('Scenario %s', scn), 'fontsize', 8);
    if c == 1, ylabel('nowcast, m'); end
    ax2 = axes('position', [x0 0.41 wd 0.20]);  hold on;  shade(bo, [-8 8]);
    plot(tt, ep, 'k-', 'linewidth', 0.7);  hline(K.r_eng);  hline(-K.r_eng);
    ylim([-8 8]);  xlim([0 10]);  grid on;  box on;  set(ax2, 'xticklabel', [], 'layer', 'top');  if c == 1, ylabel('pred., m'); end
    ax3 = axes('position', [x0 0.17 wd 0.20]);  hold on;  shade(bo, [0 12]);
    plot(tt, ev, 'k-', 'linewidth', 0.7);  plot(tt, sv, 'b--', 'linewidth', 0.7);
    ylim([0 12]);  xlim([0 10]);  grid on;  box on;  set(ax3, 'layer', 'top');  xlabel('t [s]');  if c == 1, ylabel('vel., m/s'); end
  end
  if isfield(R.C, 'blk_sig_end')
    w('\nscenario C blackout (t = %.2f ... %.2f s): nowcast 1-sigma %.3f m at its first call, %.3f m at its last call; |nowcast error| at the last call %.3f m; |t_p-ahead error| at the last call %.3f m\n', ...
      R.C.blk_t(1), R.C.blk_t(end), R.C.blk_sig_start, R.C.blk_sig_end, R.C.blk_err_end, R.C.blk_pred_end);
  end
  fclose(fid);
  tc_save_figure(h, 'report_estimation');
  fprintf('wrote outputs/report_estimation.txt and plots/report_estimation.{pdf,png}\n');
end

function shade(bo, yl)
  for j = 1:size(bo, 1)
    patch([bo(j,1) bo(j,1)+bo(j,2) bo(j,1)+bo(j,2) bo(j,1)], [yl(1) yl(1) yl(2) yl(2)], [0.85 0.85 0.85], 'edgecolor', 'none');
  end
end

function hline(y)
  plot([0 10], [y y], '--', 'color', [0.8 0 0], 'linewidth', 1.0);
end
