function T = tc_plot_error_budget()
% TC_PLOT_ERROR_BUDGET  Allowed pointing error versus range -> plots/error_budget.{pdf,png}
%   allowed:  atan(R_eng / r)  (0.143 deg at 800 m ... 0.057 deg at 2 km)
%   raw radar angle noise: sigma_theta = 0.02 deg (range-independent in angle, r*sigma_theta in metres)
%   filtered nowcast: steady-state Kalman cross-range sigma (Riccati iteration of the SAME filter,
%                     converted-measurement covariance, plant of Markov jerk), as an angle
%   after a 0.5 s blackout: the same sigma propagated 0.5 s with no data (worst coasting point)
  here = fileparts(mfilename('fullpath'));  addpath(fileparts(here));  tc_setup();
  K = tc_params();  W = tc_quadrature_rule('legendre', 8);  th = 30*pi/180;
  rr = 800:100:2000;  T = zeros(numel(rr), 5);
  [F1, Q1] = tc_jerk_model_transition(K.dt_radar, 1/K.tau_j, 2*K.sigma_j^2/K.tau_j, W);
  F = kron(eye(2), F1);  Q = kron(eye(2), Q1);  H = zeros(2, 8);  H(1,1) = 1;  H(2,5) = 1;
  [Fb, Qb] = tc_jerk_model_transition(0.5, 1/K.tau_j, 2*K.sigma_j^2/K.tau_j, W);
  Fb = kron(eye(2), Fb);  Qb = kron(eye(2), Qb);
  nb = [-sin(th); cos(th)];
  for q = 1:numel(rr)
    r = rr(q);  [~, Rz] = tc_convert_measurement(r, th, K.sig_r, K.sig_th);
    P = diag([100 K.prior_v^2 K.prior_a^2 K.sigma_j^2 100 K.prior_v^2 K.prior_a^2 K.sigma_j^2]);
    for n = 1:600
      Pp = F*P*F' + Q;  S = H*Pp*H' + Rz;  Kg = (Pp*H')/S;  P = (eye(8) - Kg*H)*Pp;
    end
    sig_f = sqrt(nb'*P([1 5],[1 5])*nb);
    Pc = Fb*P*Fb' + Qb;  sig_c = sqrt(nb'*Pc([1 5],[1 5])*nb);
    T(q,:) = [r, atand(K.r_eng/r), K.sig_th*180/pi, sig_f/r*180/pi, sig_c/r*180/pi];
  end
  h = figure('visible', 'off', 'position', [50 50 760 460], 'color', 'w');
  plot(T(:,1), T(:,2), 'k-', 'linewidth', 2);  hold on;
  plot(T(:,1), T(:,3), 'r--', 'linewidth', 1.6);
  plot(T(:,1), T(:,4), 'b-', 'linewidth', 1.6);
  plot(T(:,1), T(:,5), 'm-.', 'linewidth', 1.6);  hold off;
  grid on;  xlabel('range r [m]');  ylabel('angle [deg]');
  title('Pointing-error budget: allowed atan(R_{eng}/r) versus radar and estimator error (1\sigma)');
  legend({'allowed: atan(2 m / r)', 'raw radar angle noise 0.02 deg', 'filtered nowcast (steady state)', 'filter after 0.5 s blackout'}, 'location', 'northeast');
  tc_save_figure(h, 'error_budget');
  rd = fullfile(fileparts(here), 'outputs');  if ~exist(rd, 'dir'), mkdir(rd); end
  fid = fopen(fullfile(rd, 'error_budget.txt'), 'w');
  fprintf(fid, '%8s %10s %12s %12s %14s   (angles in degrees, theta = 30 deg)\n', 'r [m]', 'allowed', 'raw radar', 'filtered', '0.5 s blackout');
  fprintf(fid, '%8.0f %10.4f %12.4f %12.4f %14.4f\n', T');
  fprintf(fid, 'cross-range sigma at 2000 m: raw %.2f m, filtered %.2f m, after 0.5 s blackout %.2f m\n', 2000*K.sig_th, T(end,4)*pi/180*2000, T(end,5)*pi/180*2000);
  fclose(fid);  type(fullfile(rd, 'error_budget.txt'));
end
