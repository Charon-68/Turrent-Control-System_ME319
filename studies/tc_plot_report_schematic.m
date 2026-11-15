function tc_plot_report_schematic()
% TC_PLOT_REPORT_SCHEMATIC  Engineering schematic of the elevation-plane engagement geometry (report figure).
% Purely geometric drawing (no simulation, no stored data): turret and co-located radar at the origin, the allowed elevation
% sector -10 ... +85 deg, a target path with the present and the t_p-ahead position, line of sight (range r, elevation
% theta), turret/beam angle theta_T and the cross-range error e, and the R_eng ring.  Angles and distances are NOT to scale
% where the labels say so (the ring and the t_p advance are exaggerated so that they are visible).
% Writes plots/report_schematic.{pdf,png}.   Command: make report-figures
  here = fileparts(mfilename('fullpath'));  root = fileparts(here);  addpath(root);  tc_setup();  K = tc_params();
  h = figure('visible', 'off', 'position', [50 50 340 345], 'color', 'w');
  set(h, 'defaultaxesfontsize', 7, 'defaulttextfontsize', 7);
  ax = axes('position', [0.01 0.01 0.98 0.98]);  hold on;  axis equal;  axis off;  %#ok<NASGU>
  d2r = pi/180;
  % allowed sector and stops
  a = linspace(K.th_min, K.th_max, 60);
  for q = linspace(K.th_min, K.th_max, 400), plot([0 1.95*cos(q)], [0 1.95*sin(q)], '-', 'color', [0.93 0.93 0.93], 'linewidth', 2.5); end   % light sector (lines: patch colours were unreliable)
  for s = [K.th_min, K.th_max]
    plot([0 2.0*cos(s)], [0 2.0*sin(s)], 'k-', 'linewidth', 1.6);
  end
  text(2.0*cos(K.th_min) - 0.03, 2.0*sin(K.th_min) - 0.09, 'stop -10^{\circ}', 'horizontalalignment', 'right');
  text(0.08, 2.05, 'stop +85^{\circ}');
  plot([0 2.3], [0 0], 'k:', 'linewidth', 0.8);  text(2.32, 0.02, 'x');  plot([0 0], [0 1.9], 'k:', 'linewidth', 0.8);  text(0.03, 1.92, 'y');
  text(0.30, 1.50, 'allowed elevation sector', 'color', [0.35 0.35 0.35]);
  % geometry (angles exaggerated for legibility)
  tT = 22*d2r;  th = 30*d2r;  r = 1.50;  P = r*[cos(th), sin(th)];  Pf = [1.02 0.95];  thp = atan2(Pf(2), Pf(1));
  % target path: past (solid), present, predicted (dashed)
  tt = linspace(0, 1, 40);  xs = interp1(linspace(0,1,4), [2.20 1.90 1.62 P(1)], tt, 'pchip');  ys = interp1(linspace(0,1,4), [0.58 0.64 0.70 P(2)], tt, 'pchip');
  plot(xs, ys, 'k-', 'linewidth', 1.1);  plot([P(1) Pf(1)], [P(2) Pf(2)], 'k--', 'linewidth', 1.1);
  plot([2.05 1.80], [0.60 0.655], 'k-', 'linewidth', 1.2);  patch([1.78 1.86 1.84], [0.66 0.685 0.625], [0 0 0], 'edgecolor', 'k');
  text(1.55, 0.33, 'path (v, a)');
  plot(P(1), P(2), 'ko', 'markerfacecolor', 'k', 'markersize', 5);  plot(Pf(1), Pf(2), 'ko', 'markerfacecolor', 'w', 'markersize', 5);
  text(P(1) + 0.08, P(2) + 0.13, 'target at t');
  text(0.20, 1.27, 'predicted target at t + t_p'); plot([0.95 Pf(1) - 0.03], [1.23 Pf(2) + 0.04], 'k-', 'linewidth', 0.5);
  % R_eng ring (exaggerated) with a leader
  c = linspace(0, 2*pi, 80);  plot(P(1) + 0.07*cos(c), P(2) + 0.07*sin(c), 'k-', 'linewidth', 0.9);
  plot([P(1) + 0.05, 1.50], [P(2) + 0.05, 1.05], 'k-', 'linewidth', 0.5);  text(1.12, 1.12, 'R_{eng} = 2 m (not to scale)');
  % line of sight, predicted bearing, beam
  plot([0 P(1)], [0 P(2)], 'b-', 'linewidth', 1.3);
  plot([0 Pf(1)], [0 Pf(2)], 'b--', 'linewidth', 1.0);
  plot([0 2.05*cos(tT)], [0 2.05*sin(tT)], 'r-', 'linewidth', 1.8);
  text(2.05*cos(tT) + 0.02, 2.05*sin(tT) - 0.08, 'beam ray', 'color', 'r');
  text(0.62, 0.43, 'LOS, range r', 'color', 'b', 'rotation', 30);
  % angle arcs and labels
  rad = [0.46 0.58 0.70];  ang = [tT th thp];  col = {'r', 'b', 'b'};  lab = {'\theta_T', '\theta', '\theta(t+t_p)'};
  for q = 1:3
    aa = linspace(0, ang(q), 30);  plot(rad(q)*cos(aa), rad(q)*sin(aa), '-', 'color', col{q}, 'linewidth', 0.8);
  end
  text(0.50, 0.04, '\theta_T', 'color', 'r');  text(0.66, 0.26, '\theta', 'color', 'b');  text(0.14, 0.60, '\theta(t+t_p)', 'color', 'b');
  % cross-range error e
  Q = (P*[cos(tT); sin(tT)])*[cos(tT), sin(tT)];
  plot([Q(1) P(1)], [Q(2) P(2)], 'k-', 'linewidth', 1.3);  text(Q(1) + 0.05, Q(2) - 0.02, 'e = r sin(\theta-\theta_T)', 'fontsize', 6);
  % turret and radar
  patch([-0.05 0.05 0.05 -0.05], [-0.05 -0.05 0.05 0.05], [0 0 0]);  text(-0.05, -0.17, 'turret and radar (co-located)');
  xlim([-0.12 2.42]);  ylim([-0.50 2.18]);
  tc_save_figure(h, 'report_schematic');
  fprintf('wrote plots/report_schematic.{pdf,png}\n');
end
