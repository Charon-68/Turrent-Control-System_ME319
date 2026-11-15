function [V_m, fire_cmd, dbg] = turret_controller(t, r_meas, theta_meas, meas_valid, theta_turret, theta_dot_turret)
% TURRET_CONTROLLER  Predictive elevation controller and fire control (the submission).
%
%   [V_m, fire_cmd] = turret_controller(t, r_meas, theta_meas, meas_valid, ...
%                                       theta_turret, theta_dot_turret)
%
% One call = one controller tick; the call rate is free (20 Hz ... 5 kHz tested).  The
% controller measures its own dt.  A pulse is announced for fire_hold seconds (spec: holding
% fire_cmd = 1 does nothing), so a slower evaluator sampling cannot miss it.
%
% Signal flow (assignment Fig. 2):
%   radar -> converted-measurement KF (Markov jerk) -> LOS nowcast th^..th^'''
%         -> Taylor prediction r = A3(D) th^  -> deviation reference generator y_d..y_d'''
%         -> V = Gp^-1(D) y_d + C(s)(y_d - th)   [feed-forward + PI with roll-off], |V| <= V_lim
%   estimator -> calibrated hit probability -> fire-now-or-wait -> fire_cmd
%
% Why the reference follows the NOWCAST and not th(t+t_p): the laser is instantaneous, so a
% hit needs the beam on the target's PRESENT bearing.  The prescribed t_p = 0.5 s prediction
% r = A3(D) th^ is a fixed linear combination of the estimated derivatives; the generator
% (A3 + K)(D) y_d = r + K(D) th^ cancels the advance exactly, leaving (D + p)^3 (y_d - th^) = 0.
%
% State lives in one persistent struct; reset on the first call, when t decreases, when t
% returns to 0, or after a stale gap (a new run).  `clear turret_controller` also resets.
% Do not call from inside an adaptive integrator (rejected steps move t backwards).
  persistent S
  tc_setup();
  if isempty(S) || tc_session_expired(S, t)
    S = tc_session_start(t, theta_turret, theta_dot_turret);
  end
  K  = S.cfg;
  dt = max(t - S.t_prev, 0);
  if S.n_calls == 1, S.dt_est = dt; elseif dt > 0, S.dt_est = 0.8*S.dt_est + 0.2*dt; end

  % 1 ---- advance the reference with the PREVIOUS nowcast model (keeps y continuous)
  Dend = tc_taylor_advance(S.Dnow, dt);
  S = tc_reference_advance(S, Dend, dt);
  y_abs = Dend(1:3) + S.dev;

  % 2 ---- estimator
  [S, ~] = tc_filter_update(S, t, r_meas, theta_meas, meas_valid);

  % 3 ---- nowcast of the target (state propagated from the last update)
  tr = S.tr;  mode = 'acquire';  tau = 0;  xn = zeros(8,1);  Pn = eye(8);
  if tr.on && (t - tr.t_upd) <= K.lost_gap
    tau = t - tr.t_upd;
    [F1, Q1] = tc_jerk_model_transition(tau, 1/K.tau_j, S.q_cont, S.Wgl);
    F  = kron(eye(2), F1);
    xn = F*tr.x;
    Pn = F*tr.P*F' + tr.rho*kron(eye(2), Q1);
    Dn = tc_los_derivatives(xn);
    mode = 'track';  if tau > K.coast_gap, mode = 'coast'; end
  else
    Dn = [Dend(1); 0; 0; 0];                 % no usable track: hold the reference where it is
  end
  Dn = tc_stops_governor(Dn, K);
  Dr = Dn;                                   % tuple the reference follows: the PRESENT bearing ...
  if K.aim_ahead ~= 0                        % ... unless the t_p-ahead comparison mode is on
    Dr = tc_stops_governor(tc_taylor_advance(Dn, K.aim_ahead), K);
  end
  S.dev  = y_abs - Dr(1:3);                  % re-express the continuous reference vs the new tuple
  S.Dnow = Dr;

  % 4 ---- reference top derivative and the t_p prediction (monitor / reference input r)
  [d3, lin] = tc_reference_chain(S.dev, Dr, K);
  yv = [y_abs; Dr(4) + d3];
  r_pred = [1, K.tp, K.tp^2/2, K.tp^3/6] * Dn;
  lead = 0;
  if K.lead_hold && S.n_calls >= 2, lead = min(0.5*S.dt_est, K.lead_max); end
  yl = tc_taylor_advance(yv, lead);            % evaluate at mid-hold: ZOH compensation

  % 5 ---- feed-forward (exact plant inverse) + PI with roll-off
  aff = S.aff;
  u_ff = K.ff(1)*yl(4) + K.ff(2)*yl(3) + aff*K.ff(3)*yl(2);
  e = yl(1) - theta_turret;
  deg = pi/180;
  at_stop = (theta_turret >= K.th_max - 0.5*deg && e > 0) || (theta_turret <= K.th_min + 0.5*deg && e < 0);
  if dt > 0
    pause_acq = K.freeze_acq && ~lin;                           % reference is clamp-limited, not tracking the linear law
    if ~(S.sat*e > 0) && ~at_stop && ~pause_acq, S.xi = S.xi + e*dt; end   % conditional integration
    a = exp(-dt/K.tau_f);
    S.xf = a*S.xf + (1 - a)*K.kp*(e + S.xi/K.ti);
    if K.adapt_ff && strcmp(mode, 'track') && S.sat == 0 && abs(yl(2)) > 0.03
      tgt  = min(max((K.kp*S.xi/K.ti)/(K.ff(3)*yl(2)), -0.5), 0.5);
      aff2 = min(max(S.aff + K.aff_gamma*dt*tgt, K.aff_lo), K.aff_hi);
      S.xi = S.xi - (aff2 - S.aff)*K.ff(3)*yl(2)*K.ti/K.kp;     % move integrator content into the gain
      S.aff = aff2;
    end
  end
  u = u_ff + S.xf;
  V_m = min(max(u, -K.V_lim), K.V_lim);
  S.sat = 0;  if abs(u) > K.V_lim, S.sat = sign(u); end

  % 6 ---- fire control
  fire = false;  info = struct('p', 0, 'p_raw', 0, 'cons', 0, 'sig', NaN);
  if any(strcmp(mode, {'track', 'coast'}))
    [S, fire, info] = tc_firing_rule(S, t, mode, xn, Pn, theta_turret, tau, Dn(2) - theta_dot_turret);
  end
  fire_cmd = double(fire || (t <= S.hold_until));

  S.t_prev = t;  S.n_calls = S.n_calls + 1;
  if nargout > 2
    dbg = struct('mode', mode, 'theta_hat', Dn(1), 'r_pred', r_pred, 'y_d', yl(1), 'y_d1', yl(2), ...
                 'u_ff', u_ff, 'u_fb', S.xf, 'p_hit', info.p, 'p_raw', info.p_raw, 'cons', info.cons, ...
                 'sig', info.sig, 'n_upd', S.tr.n_upd, 'rho', S.tr.rho, 'aff', S.aff, 'linear', lin, ...
                 'x_hat', xn);
  end
end
