function out = tc_run_engagement(scenario, opts)
% TC_RUN_ENGAGEMENT  Evaluation-style closed loop for ONE scenario (the harness our tests use).
%
%   out = tc_run_engagement('B')                       defaults below
%   out = tc_run_engagement('MC5001', struct('f_ctrl', 20))
%
% opts (all optional)
%   f_ctrl 100       controller call rate [Hz]          theta0 0   initial turret angle [rad]
%   integrator 'exact'|'rk4'|'euler'   dt_plant 1e-3    plant_scale struct (R L J b Kt Ke)
%   friction struct('coulomb', N m, 'deadzone', V)      t_end 10
%   convention 'ref' | 'held' | 'drift' | 'jitter'      keep_log true
%     ref    radar value only on ticks, NaN/0 between (the assumed evaluation convention)
%     held   last sample held, meas_valid = 1 on EVERY call
%     drift  valid on every call, value = interpolated truth + the last tick's noise (correlated copies)
%     jitter irregular call times (uniform 0.5..1.5 x nominal period); each radar sample is
%            delivered once, at the first call after its tick
% Pulses are judged on the 0 -> 1 edge of fire_cmd, >= 1 s after the previous pulse, at the call time,
% with the beam as a ray along the MEASURED turret angle.
  tc_setup();
  o = struct('f_ctrl', 100, 'theta0', 0, 'integrator', 'exact', 'dt_plant', 1e-3, ...
             'plant_scale', struct(), 'friction', struct('coulomb', 0, 'deadzone', 0), ...
             't_end', 10, 'convention', 'ref', 'keep_log', true, 'seed', 1);
  if nargin > 1, f = fieldnames(opts); for q = 1:numel(f), o.(f{q}) = opts.(f{q}); end, end
  K = tc_params();
  M = tc_plant_model(o.plant_scale, o.friction);
  clear turret_controller
  % call times
  T = 1/o.f_ctrl;
  if strcmp(o.convention, 'jitter')
    u = tc_uniform_draws(o.seed + 91, ceil(o.t_end*o.f_ctrl*1.6) + 10);
    tk = [0; cumsum(T*(0.5 + u))];  tk = tk(tk <= o.t_end + 1e-12)';
  else
    tk = (0:round(o.t_end*o.f_ctrl))/o.f_ctrl;
  end
  n = numel(tk);
  s = [o.theta0; 0; 0];
  L = struct('t', tk, 'th', zeros(1,n), 'thd', zeros(1,n), 'i', zeros(1,n), 'V', zeros(1,n), ...
             'th_true', zeros(1,n), 'r_true', zeros(1,n), 'th_hat', NaN(1,n), 'y_d', NaN(1,n), ...
             'p_hit', NaN(1,n), 'p_raw', NaN(1,n), 'hit_now', NaN(1,n), 'ready', false(1,n), 'valid', zeros(1,n), 'err_m', zeros(1,n), 'mode', {cell(1,n)});
  pulses = zeros(0, 5);                                % [t miss hit signed p_hit]
  t_last = -Inf;  fire_prev = 0;  held = [NaN NaN];  held_t = NaN;  tick_done = -1;
  for k = 1:n
    t = tk(k);
    [r, th, v, tru] = target_motion_generator(t, scenario);
    if strcmp(o.convention, 'jitter')
      % call times do not coincide with radar ticks: deliver each tick's sample ONCE, at the first
      % call at or after the tick (what a sample-and-hold evaluation script would do)
      kt = floor(t/K.dt_radar + 1e-9);  r = NaN;  th = NaN;  v = 0;
      if kt > tick_done
        [rq, tq, vq] = target_motion_generator(kt*K.dt_radar, scenario);
        if vq, r = rq; th = tq; v = 1; end
        tick_done = kt;
      end
    end
    switch o.convention
      case 'held'
        if v, held = [r th]; end
        if all(isfinite(held)), r = held(1); th = held(2); v = 1; end
      case 'drift'
        % valid on EVERY call; value = current truth + the noise of the last real radar sample
        % (correlated copies).  A blacked-out radar tick clears the held sample: no data in a blackout.
        kk = round(t/K.dt_radar);
        if abs(t - kk*K.dt_radar) < 1e-6 && ~v, held = [NaN NaN];  held_t = NaN; end
        if v, held = [r th];  held_t = t; end
        if all(isfinite(held))
          [rt, tt] = tc_truth_at_tick(scenario, held_t, K.dt_radar);   % truth when `held` was measured
          r  = hypot(tru(1), tru(2)) + (held(1) - rt);                 % current truth + that sample's noise
          th = atan2(tru(2), tru(1)) + (held(2) - tt);
          v = 1;
        else
          r = NaN;  th = NaN;  v = 0;
        end
    end
    th_now = s(1);  thd_now = s(2);
    if o.keep_log
      [V, fire, dbg] = turret_controller(t, r, th, v, th_now, thd_now);
      L.th_hat(k) = dbg.theta_hat;  L.y_d(k) = dbg.y_d;  L.p_hit(k) = dbg.p_hit;  L.p_raw(k) = dbg.p_raw;  L.mode{k} = dbg.mode;
      L.ready(k) = strcmp(dbg.mode, 'track') && dbg.n_upd >= K.n_upd_min;
    else
      [V, fire] = turret_controller(t, r, th, v, th_now, thd_now);
    end
    L.th(k) = th_now;  L.thd(k) = thd_now;  L.i(k) = s(3);  L.V(k) = V;  L.valid(k) = v;
    L.th_true(k) = atan2(tru(2), tru(1));  L.r_true(k) = hypot(tru(1), tru(2));
    L.err_m(k) = L.r_true(k)*sin(L.th_true(k) - th_now);
    L.hit_now(k) = tc_beam_miss_distance(th_now, tru(1), tru(2)) <= K.r_eng;     % would a pulse right now hit?
    if fire > 0.5 && fire_prev <= 0.5 && (t - t_last) >= 1 - 1e-9
      [miss, sg] = tc_beam_miss_distance(th_now, tru(1), tru(2));
      pulses(end+1,:) = [t, miss, miss <= K.r_eng, sg, L.p_hit(k)]; %#ok<AGROW>
      t_last = t;
    end
    fire_prev = fire;
    if k < n
      dt = tk(k+1) - t;  ns = max(1, ceil(dt/o.dt_plant - 1e-9));  h = dt/ns;
      for q = 1:ns, [s, M] = tc_plant_step(M, s, V, h, o.integrator); end
      if ~all(isfinite(s)), error('tc_run_engagement: plant diverged at t = %.3f', t); end
    end
  end
  out.log = L;  out.pulses = pulses;  out.scenario = scenario;  out.opts = o;
  out.metrics = tc_engagement_metrics(L, pulses, K);
end

function [r, th] = tc_truth_at_tick(scenario, t, dtr)
% truth polar coordinates at the most recent radar tick (to form the correlated-copy noise)
  tk = floor(t/dtr + 1e-9)*dtr;
  [~, ~, ~, tru] = target_motion_generator(tk, scenario);
  r = hypot(tru(1), tru(2));  th = atan2(tru(2), tru(1));
end
