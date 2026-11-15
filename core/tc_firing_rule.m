function [S, fire, info] = tc_firing_rule(S, t, mode, x, Pc, th_T, tau, th_dot_rel)
% TC_FIRING_RULE  Fire-now-or-wait rule on the calibrated hit probability.
%
%   mode        'track' | 'coast'   (anything else never fires)
%   x, Pc       nowcast state and covariance at time t
%   th_T        measured turret angle;  tau = data age (s)
%   th_dot_rel  LOS rate minus turret rate (rad/s), used by the sweep guard
%
% Every pulse blocks the next one for t_recharge and nothing reports a kill, so the
% rule is a one-step optimal-stopping heuristic:
%   tracking : fire at P >= p_sure; else arm at P >= p_arm and fire when P has
%              fallen peak_drop below its armed maximum, or after t_wait.
%   coasting : certainty can only decay in a blackout, so fire while P >= p_coast
%              and falling.
% After the first pulse the interception time is already decided, so only
% accuracy/consistency remain: the bar rises to p_sure_after (no time cost).
  K = S.cfg;  fire = false;
  if strcmp(K.phit_model, 'classes')
    [praw, sig, cons] = tc_hit_prob_classes(S, x, Pc, th_T, tau);
  else
    [praw, sig, cons] = tc_hit_probability(S, x, Pc, th_T);
  end
  p = tc_calibrate_probability(S.cal, praw);
  info = struct('p', p, 'p_raw', praw, 'cons', cons, 'sig', sig);

  recharged = (t - S.t_fire) >= (K.t_recharge + K.recharge_margin);
  slip = hypot(x(1), x(5)) * abs(th_dot_rel) * min(max(S.dt_est, 0), K.guard_window);   % beam-vs-target slip over a rate-independent window
  ready = recharged && S.tr.n_upd >= K.n_upd_min && slip <= K.sweep_guard ...
          && cons >= K.q_cons_min;
  p_sure = K.p_sure;  if S.n_fired > 0, p_sure = K.p_sure_after; end

  if ready && any(strcmp(mode, {'track', 'coast'}))
    if strcmp(mode, 'coast')
      fire = (p >= K.p_coast) && (p < S.p_prev);
    elseif p >= p_sure
      fire = true;
    elseif p >= K.p_arm
      if ~S.armed
        S.armed = true;  S.t_arm = t;  S.p_max = p;
      else
        S.p_max = max(S.p_max, p);
        fire = (p < S.p_max - K.peak_drop) || (t - S.t_arm >= K.t_wait);
      end
    else
      S.armed = false;
    end
  elseif ~recharged
    S.armed = false;
  end

  if fire
    S.t_fire = t;  S.n_fired = S.n_fired + 1;  S.armed = false;
    S.hold_until = t + K.fire_hold;
  end
  S.p_prev = p;
end
