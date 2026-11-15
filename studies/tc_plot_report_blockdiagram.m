function tc_plot_report_blockdiagram()
% TC_PLOT_REPORT_BLOCKDIAGRAM  Closed-loop block diagram sized for a two-column IEEE page (7 in wide, text >= 7 pt).
% Same structure as plots/closed_loop (tc_plot_block_diagram) but redrawn with larger text; the numbers shown are
% read from core/tc_params.m (plant polynomial, feed-forward coefficients, PI gains).  Pure drawing, no simulation.
% Writes plots/report_blockdiagram.{pdf,png}.   Command: make report-figures
  here = fileparts(mfilename('fullpath'));  root = fileparts(here);  addpath(root);  tc_setup();  K = tc_params();
  h = figure('visible', 'off', 'position', [50 50 700 275], 'color', 'w');
  set(h, 'defaultaxesfontsize', 7, 'defaulttextfontsize', 7);
  axes('position', [0 0 1 1]);  hold on;  axis off;  xlim([0 7]);  ylim([0 3]);
  navy = [0.1 0.2 0.5];
  % ---- row 0: fire control (above the estimator) ----
  box_(2.45, 2.45, 1.30, 0.48, {'Hit probability', 'calibrated posterior'});
  box_(0.80, 2.45, 1.25, 0.48, {'Fire rule', 'fire now or wait'});
  arrow_([3.10 2.20], [3.10 2.45]);  text(3.14, 2.31, 'x, P', 'fontsize', 7);
  arrow_([2.45 2.69], [2.05 2.69]);  arrow_([0.80 2.69], [0.25 2.69]);  text(0.02, 2.80, 'fire\_cmd', 'fontsize', 7);
  % ---- row 1: sensing, estimation, prediction, reference ----
  box_(0.05, 1.65, 0.95, 0.55, {'Target', 'maneuver a(t)'});
  box_(1.30, 1.65, 1.00, 0.55, {'Radar 20 Hz', 'noise, blackout'});
  box_(2.45, 1.65, 1.30, 0.55, {'Converted-meas.', 'Kalman filter', 'Markov jerk'});
  box_(4.00, 1.65, 1.35, 0.55, {'LOS nowcast', 'Taylor prediction', 'r = A_3(D)\theta'});
  box_(5.60, 1.65, 1.35, 0.55, {'Reference generator', '(A_3+K)y_d = r+K\theta'});
  arrow_([1.00 1.925], [1.30 1.925]);  text(1.02, 2.00, '(x,y)', 'fontsize', 6);
  arrow_([2.30 1.925], [2.45 1.925]);  arrow_([3.75 1.925], [4.00 1.925]);  arrow_([5.35 1.925], [5.60 1.925]);
  text(2.31, 2.02, '(r,\theta)', 'fontsize', 6);  text(3.77, 2.02, 'D', 'fontsize', 6);  text(5.40, 2.02, 'r', 'fontsize', 6);
  % ---- row 2: feed-forward, summing junction, saturation, plant ----
  box_(5.60, 0.85, 1.35, 0.55, {'Feed-forward G_p^{-1}(D)', sprintf('(%.2f, %.2f, %.1f)', K.ff(1), K.ff(2), K.ff(3))});
  line_([6.78 6.78], [1.65 1.125]);  arrow_([6.78 1.125], [6.95 1.125]);  text(6.55, 1.50, 'y_d', 'fontsize', 7);
  arrow_([5.60 1.125], [5.07 1.125]);  text(5.28, 1.20, 'V_{ff}', 'fontsize', 6);
  circ_(4.95, 1.125);
  arrow_([4.83 1.125], [4.50 1.125]);
  box_(3.55, 0.85, 0.95, 0.55, {'Saturation', sprintf('\\pm %g V', K.V_lim)});
  arrow_([3.55 1.125], [3.30 1.125]);  text(3.33, 1.20, 'V_m', 'fontsize', 6);
  box_(1.30, 0.85, 2.00, 0.55, {'Plant G_p(s)', '1000 / [s(320s^2+3350s+1.0015e6)]'});
  arrow_([1.30 1.125], [0.15 1.125]);  text(0.50, 1.20, '\theta', 'fontsize', 8);
  % ---- row 3: PI feedback ----
  box_(4.30, 0.10, 1.40, 0.50, {'PI + roll-off C(s)', sprintf('K_p = %g V/rad, T_i = %g s', K.kp, K.ti)});
  arrow_([4.95 0.60], [4.95 1.065]);  text(5.00, 0.82, 'V_{fb}', 'fontsize', 6);
  circ_(6.30, 0.35);
  arrow_([6.18 0.35], [5.70 0.35]);  text(5.85, 0.43, 'e', 'fontsize', 7);
  line_([6.78 6.78], [1.125 0.35]);  arrow_([6.78 0.35], [6.42 0.35]);  text(6.45, 0.50, '+', 'fontsize', 8);
  line_([0.80 0.80], [1.125 0.04]);  line_([0.80 6.30], [0.04 0.04]);  arrow_([6.30 0.04], [6.30 0.23]);  text(6.35, 0.12, '-', 'fontsize', 9);
  text(1.20, 0.12, 'encoder: \theta, d\theta/dt (also to the hit probability)', 'fontsize', 6);
  tc_save_figure(h, 'report_blockdiagram');
  fprintf('wrote plots/report_blockdiagram.{pdf,png}\n');

  function box_(x, y, w, hh, lines)
    patch([x x+w x+w x], [y y y+hh y+hh], [0.94 0.96 1.0], 'edgecolor', navy, 'linewidth', 1.0);
    n = numel(lines);
    for q = 1:n
      text(x + w/2, y + hh/2 + (n+1)/2*0.115 - q*0.115 - 0.0, lines{q}, 'horizontalalignment', 'center', 'verticalalignment', 'middle', 'fontsize', 7);
    end
  end
  function arrow_(p, q)
    plot([p(1) q(1)], [p(2) q(2)], 'k-', 'linewidth', 0.9);
    d = [q(1)-p(1), q(2)-p(2)];  d = d/norm(d);  nrm = [-d(2), d(1)];  s = 0.05;
    patch([q(1), q(1) - s*2*d(1) + s*nrm(1), q(1) - s*2*d(1) - s*nrm(1)], [q(2), q(2) - s*2*d(2) + s*nrm(2), q(2) - s*2*d(2) - s*nrm(2)], [0 0 0], 'edgecolor', 'k');
  end
  function line_(p, q), plot(p, q, 'k-', 'linewidth', 0.9); end
  function circ_(x, y)
    c = linspace(0, 2*pi, 40);  patch(x + 0.12*cos(c), y + 0.12*sin(c), [1 1 1], 'edgecolor', 'k', 'linewidth', 0.9);
    plot(x + [-0.07 0.07], [y y], 'k-', 'linewidth', 0.7);  plot([x x], y + [-0.07 0.07], 'k-', 'linewidth', 0.7);
  end
end
