function K = tc_params()
% TC_PARAMS  Single source of truth for every constant in the project.
%
% The plant constants are the assignment's; everything else (transfer-function
% coefficients, feed-forward gains, cruise rate, cascade clamps) is DERIVED here,
% so changing a plant number can never leave a stale hand-typed inverse behind.
% Every other file reads this struct; nothing else defines a physical constant.

  % ---- specification (assignment Sec. 2) ------------------------------------
  K.tp         = 0.5;            % s   prescribed prediction horizon
  K.r_eng      = 2.0;            % m   engagement radius
  K.t_recharge = 1.0;            % s   minimum pulse spacing
  K.dt_radar   = 0.05;           % s   radar period
  K.sig_r      = 1.0;            % m
  K.sig_th     = 0.02*pi/180;    % rad
  K.v_tgt_max  = 150;            % m/s
  K.a_tgt_max  = 20;             % m/s^2
  K.r_min      = 800;  K.r_max = 2000;
  K.th_min     = -10*pi/180;  K.th_max = 85*pi/180;

  % ---- plant (derived from first principles, see docs/DESIGN.md Sec. 2) ------
  K.J_turret = 1200;  K.J_motor = 0.20;  K.N = 100;  K.b = 1500;
  K.Kt = 10;  K.Ke = 10;  K.R = 1;  K.L = 0.1;
  K.V_lim = 600;                 % V   ASSUMED drive limit (spec is silent)
  K.J_eq = K.J_turret + K.N^2*K.J_motor;
  K.gn   = K.N*K.Kt;             % torque constant seen at the turret
  % theta/V = gn / ( s*(c3' s^2 + ...) ):  den = L*J s^2 + (L b + R J) s + (R b + N^2 Kt Ke)
  K.den  = [K.L*K.J_eq, K.L*K.b + K.R*K.J_eq, K.R*K.b + K.N^2*K.Kt*K.Ke];
  K.ff   = K.den / K.gn;         % [c3 c2 c1]  exact inverse, V = c3 y''' + c2 y'' + c1 y'
  K.omega_cruise = K.V_lim / K.ff(3);   % rad/s  back-EMF limited cruise rate

  % ---- estimator --------------------------------------------------------------
  K.sigma_j   = 20;              % m/s^3  Gauss-Markov jerk std
  K.tau_j     = 0.4;             % s
  K.prior_v   = 90;              % m/s   per-axis velocity prior std (target speed <= 150 m/s)
  K.prior_a   = 12;              % m/s^2
  K.env_v     = 1.05*K.v_tgt_max;      % envelope projection limits (5 %, 15 % margins)
  K.env_a     = 1.15*K.a_tgt_max;
  K.env_j     = 400;
  K.chi2_gate = 40;              % two consecutive implausible innovations => new track
  K.min_gap   = 0.4*K.dt_radar;  % ignore "valid" samples closer than this (held / oversampled flags)
  K.lost_gap  = 1.5;             % s without data: forget the track
  K.coast_gap = 1.5*K.dt_radar;  % older than this the track is "coasting"
  K.adapt_q   = true;            % NIS-driven process-noise scaling
  K.adapt_lam = 0.15;  K.adapt_eta = 0.25;  K.adapt_lo = 0.4;  K.adapt_hi = 8;

  % ---- reference generator ------------------------------------------------------
  K.p_dev     = 20;              % rad/s  triple deviation pole
  K.acq_rate  = 0.97*K.omega_cruise;   % rad/s  absolute rate clamp (97 % of back-EMF limit)
  K.i_lim     = 10;              % A  ASSUMED motor-current budget (spec is silent; hardware-typical)
  K.acq_accel = 0.8*K.gn*K.i_lim/K.J_eq;   % rad/s^2 acceleration-demand clamp = 2.5 for 10 A (20 % margin)
  K.acq_decel = 0;               % rad/s^2 braking curve beyond the linear zone (0 = plain clamp)
  K.acq_zone  = 0.05;            % rad     linear zone where the exact triple-pole law applies
  K.aim_ahead = 0;               % s  COMPARISON ONLY: aim the beam this far along the predicted path (0 = present bearing)
  K.lim_margin = 0.3*pi/180;     % rad     reference kept this far inside the stops

  % ---- feedback ------------------------------------------------------------------
  K.kp = 4000;  K.ti = 1.0;  K.tau_f = 0.03;
  K.freeze_acq = true;           % pause the PI integrator while the acquisition clamps are active
  K.lead_hold = false;           % OFF: shifting the reference by half a call period biases the beam ahead by rate*lead (docs/DECISIONS.md D11)
  K.lead_max  = 0.025;           % s
  K.adapt_ff  = false;           % online scale on the velocity feed-forward (robustness option)
  K.aff_gamma = 0.6;  K.aff_lo = 0.6;  K.aff_hi = 1.6;

  % ---- fire control -----------------------------------------------------------------
  K.phit_model = 'posterior';    % 'posterior' (Rao-Blackwellised) or 'classes' (17 hand-picked)
  K.p_sure   = 0.99;  K.p_arm = 0.90;  K.t_wait = 0.30;  K.peak_drop = 0.02;
  K.p_coast  = 0.95;
  K.p_sure_after = 0.995;        % once a pulse is out only accuracy/consistency are at stake
  K.n_upd_min = 8;
  K.q_cons_min = 0.80;           % posterior mass of futures that are on target, required to fire
  K.sweep_guard = Inf;           % m  OFF by default: it cost 0.21 s (100 Hz) to 1.1 s (20 Hz) of first-hit time and removed no miss (D12).
  K.guard_window = 0.01;         % s  slip window if an evaluator that judges BETWEEN calls makes the guard worthwhile (set sweep_guard ~ 0.5)
  K.fire_hold = 0.05;            % s  fire_cmd stays 1 this long (harmless per spec; survives slow samplers)
  K.recharge_margin = 1e-6;
  K.a_env_hyp = 1.05*K.a_tgt_max;% acceleration cut-off for the posterior mixture

  % ---- user / experiment overrides ---------------------------------------------------
  o = tc_settings('get');
  f = fieldnames(o);
  for k = 1:numel(f), K.(f{k}) = o.(f{k}); end
  % keep derived values consistent if the experiment changed their sources
  if isfield(o, 'V_lim') && ~isfield(o, 'omega_cruise'), K.omega_cruise = K.V_lim / K.ff(3); end
  if isfield(o, 'V_lim') && ~isfield(o, 'acq_rate'),     K.acq_rate = 0.97*K.omega_cruise;   end
end
