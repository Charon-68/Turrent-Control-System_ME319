function S = tc_session_start(t, theta_turret, theta_dot_turret)
% TC_SESSION_START  Fresh controller state (one struct, no globals, no closures).
  K = tc_params();
  S.cfg  = K;
  S.q_cont = 2*K.sigma_j^2/K.tau_j;                  % continuous jerk-noise intensity
  S.Wgl  = tc_quadrature_rule('legendre', 8);
  Wgh    = tc_quadrature_rule('hermite', 5);
  [Z1, Z2] = meshgrid(Wgh.x, Wgh.x);
  S.Zgh  = [Z1(:)'; Z2(:)'];                         % 2 x 25 standard-normal nodes
  Wtensor = Wgh.w * Wgh.w';                          % (no chained indexing: keep the file MATLAB-valid)
  S.Wgh  = Wtensor(:)';                              % 1 x 25 weights
  S.H    = zeros(2,8);  S.H(1,1) = 1;  S.H(2,5) = 1;
  S.cal  = tc_calibration_table();
  if isfield(K, 'calib_off') && K.calib_off, S.cal = struct('kx', [0 1], 'ky', [0 1]); end   % identity (used while FITTING the map)
  S.t_first = t;  S.t_prev = t;  S.n_calls = 0;  S.dt_est = 0.01;
  S.tr = struct('on', false, 'x', zeros(8,1), 'P', eye(8), 't_upd', t, 'n_upd', 0, ...
                'n_out', 0, 'rho', 1, 'nis_ema', 1, 'last_r', NaN, 'last_th', NaN);
  % reference generator: deviation [y y' y''] - [th^ th^' th^''] from the nowcast tuple
  S.dev = [0; theta_dot_turret; 0];                  % reference starts on the turret state
  S.Dnow = [theta_turret; 0; 0; 0];                  % last effective nowcast tuple
  S.Phi_dt = NaN;  S.Phi = eye(3);                   % cached exact deviation transition
  % feedback
  S.xi = 0;  S.xf = 0;  S.aff = 1;  S.sat = 0;
  % fire
  S.t_fire = -Inf;  S.n_fired = 0;  S.armed = false;  S.t_arm = 0;  S.p_max = 0;
  S.p_prev = 0;  S.hold_until = -Inf;
end
