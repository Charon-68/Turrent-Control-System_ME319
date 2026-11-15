function tc_plot_block_diagram()
% TC_PLOT_BLOCK_DIAGRAM  Closed-loop block diagram -> plots/closed_loop.{pdf,png}
% Radar with noise and dropout, estimator, predictor, reference generator, 2-DOF
% (feed-forward + PI) loop, saturation, plant, sensor feedback, disturbance inputs, fire control.
  here = fileparts(mfilename('fullpath'));  addpath(fileparts(here));  tc_setup();
  h = figure('visible', 'off', 'position', [50 50 1250 640], 'color', 'w');
  ax = axes('position', [0 0 1 1]);  axis([0 125 0 64]);  axis off;  hold on;
  navy = [0.10 0.20 0.45];  grey = [0.35 0.35 0.35];  red = [0.70 0.10 0.10];
  % ---- row 1: sensing and estimation (left to right) ----
  box(ax, 2, 46, 14, 9, {'Target', 'maneuver a(t)', '|v|<=150, |a|<=20'}, navy);
  box(ax, 23, 46, 15, 9, {'Radar 20 Hz', 'r+n_r, \theta+n_\theta', 'dropout <= 0.5 s'}, navy);
  box(ax, 46, 46, 19, 9, {'Converted-meas. KF', 'Markov-jerk, 8 states', '\chi^2 gate, envelope clip'}, navy);
  box(ax, 73, 46, 20, 9, {'LOS nowcast + Taylor', '\theta, \theta'', \theta'''', \theta''''''', 'r = A_3(D) \theta  (t_p = 0.5 s)'}, navy);
  box(ax, 101, 46, 21, 9, {'Reference generator', '(A_3+K)(D) y_d = r + K(D) \theta', 'deviation poles -p (x3)'}, navy);
  arrow(ax, 16, 50.5, 23, 50.5);  arrow(ax, 38, 50.5, 46, 50.5);
  arrow(ax, 65, 50.5, 73, 50.5);  arrow(ax, 93, 50.5, 101, 50.5);
  text(19.5, 52.5, '(x,y)', 'fontsize', 8, 'horizontalalignment', 'center');
  text(42, 52.5, '(r,\theta) meas.', 'fontsize', 8, 'horizontalalignment', 'center');
  text(69, 52.5, 'x_{est}, P', 'fontsize', 8, 'horizontalalignment', 'center');
  text(97, 52.5, 'r', 'fontsize', 8, 'horizontalalignment', 'center');
  % disturbance inputs (red)
  arrow(ax, 9, 60, 9, 55, red);  text(9, 62, 'disturbance: target maneuver', 'fontsize', 8, 'color', red, 'horizontalalignment', 'center');
  arrow(ax, 30.5, 60, 30.5, 55, red);  text(30.5, 62, 'noise, dropout', 'fontsize', 8, 'color', red, 'horizontalalignment', 'center');
  % ---- row 2: control path (right to left) ----
  box(ax, 101, 28, 21, 9, {'Reference y_d', 'y_d, y_d'', y_d'''', y_d''''''', 'no differentiation'}, navy);
  arrow(ax, 111.5, 46, 111.5, 37);
  box(ax, 76, 28, 19, 9, {'Feed-forward G_p^{-1}(D)', '0.32 y''''''+3.35 y''''+1001.5 y'''}, navy);
  arrow(ax, 101, 32.5, 95, 32.5);
  circle(ax, 58, 32.5);  arrow(ax, 76, 32.5, 59.4, 32.5);  text(66, 34, 'V_{ff}', 'fontsize', 8, 'horizontalalignment', 'center');
  box(ax, 36, 28, 14, 9, {'Saturation', '\pm V_{lim}'}, navy);
  arrow(ax, 56.6, 32.5, 50, 32.5);
  box(ax, 8, 28, 22, 9, {'Plant G_p(s)', '1000/(s(320s^2+3350s+1.0015e6))', 'motor + gear + turret'}, navy);
  arrow(ax, 36, 32.5, 30, 32.5);  text(33, 34, 'V_m', 'fontsize', 8, 'horizontalalignment', 'center');
  arrow(ax, 8, 32.5, 2, 32.5);  text(4.5, 34, '\theta', 'fontsize', 10, 'horizontalalignment', 'center');
  arrow(ax, 19, 41, 19, 37, red);  text(19, 42.6, 'plant mismatch', 'fontsize', 8, 'color', red, 'horizontalalignment', 'center');
  % ---- row 3: error junction and PI (right to left), fire control ----
  line([108 108], [28 16.5], 'color', 'k', 'linewidth', 1.2);  arrow(ax, 108, 16.5, 89.4, 16.5);  text(98, 18, 'y_d', 'fontsize', 8);
  circle(ax, 88, 16.5);  text(89.6, 13.2, '-', 'fontsize', 12, 'fontweight', 'bold');  text(85.6, 19.5, 'e = y_d - \theta', 'fontsize', 8);
  box(ax, 66, 12, 16, 9, {'PI + roll-off C(s)', 'K_p(1+1/(T_i s))/(\tau_f s+1)', 'cond. integration'}, navy);
  arrow(ax, 86.6, 16.5, 82, 16.5);
  line([66 58], [16.5 16.5], 'color', 'k', 'linewidth', 1.2);  arrow(ax, 58, 16.5, 58, 31.1);  text(60, 24, 'V_{fb}', 'fontsize', 8);
  % sensor feedback: theta -> error junction and fire control
  line([2 2], [32.5 5], 'color', grey, 'linewidth', 1.2);  line([2 88], [5 5], 'color', grey, 'linewidth', 1.2);  arrow(ax, 88, 5, 88, 15.1, grey);
  text(45, 3, 'sensor feedback of \theta_{turret} (and rate): error junction and fire control', 'fontsize', 8, 'color', grey, 'horizontalalignment', 'center');
  % ---- fire control ----
  box(ax, 30, 12, 18, 9, {'Hit probability', 'calibrated posterior'}, navy);
  box(ax, 8, 12, 16, 9, {'Fire rule', 'fire-now-or-wait'}, navy);
  line([53 53], [46 24.5], 'color', 'k', 'linewidth', 1.2);  line([53 39], [24.5 24.5], 'color', 'k', 'linewidth', 1.2);  arrow(ax, 39, 24.5, 39, 21);
  text(54, 40, 'x_{est}, P', 'fontsize', 8);
  line([39 39], [5 12], 'color', grey, 'linewidth', 1.2);  arrow(ax, 39, 10.5, 39, 12, grey);
  arrow(ax, 30, 16.5, 24, 16.5);  arrow(ax, 8, 16.5, 4.5, 16.5);  text(4.5, 14.2, 'fire\_cmd', 'fontsize', 9);
  hold off;
  tc_save_figure(h, 'closed_loop');
end

function box(ax, x, y, w, hgt, lines, col)
  rectangle('position', [x y w hgt], 'edgecolor', col, 'linewidth', 1.5, 'facecolor', [0.95 0.97 1]);
  n = numel(lines);
  for k = 1:n
    fs = 8.5; if k == 1, fs = 9.5; end
    wt = 'normal'; if k == 1, wt = 'bold'; end
    text(x + w/2, y + hgt - (k - 0.5)*hgt/n, lines{k}, 'fontsize', fs, 'fontweight', wt, 'horizontalalignment', 'center', 'verticalalignment', 'middle');
  end
end

function circle(ax, x, y)
  rectangle('position', [x-1.3 y-1.3 2.6 2.6], 'curvature', [1 1], 'edgecolor', 'k', 'linewidth', 1.2, 'facecolor', 'w');
  text(x, y, '+', 'fontsize', 8, 'horizontalalignment', 'center');
end

function arrow(ax, x1, y1, x2, y2, col)
  if nargin < 6, col = 'k'; end
  if x1 ~= x2 || y1 ~= y2, line([x1 x2], [y1 y2], 'color', col, 'linewidth', 1.2); end
  d = [x2 - x1, y2 - y1];  if norm(d) == 0, d = [1 0]; end, d = d/norm(d);  nrm = [-d(2), d(1)];
  tip = [x2 y2];  b1 = tip - 1.2*d + 0.5*nrm;  b2 = tip - 1.2*d - 0.5*nrm;
  patch([tip(1) b1(1) b2(1)], [tip(2) b1(2) b2(2)], col, 'edgecolor', col);
end
