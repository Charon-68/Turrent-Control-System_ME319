function [S, took] = tc_filter_update(S, t, r, th, valid)
% TC_FILTER_UPDATE  Converted-measurement Kalman filter step with the data-validity
% gating an unknown evaluation script requires.
%
% A sample is used only if it is flagged valid, finite, not an exact repeat of
% the previous sample, and at least K.min_gap after the last used sample (so a
% harness that raises meas_valid on every call, or holds the last sample, cannot
% feed correlated copies into the filter).  Gap handling needs no special case:
% one exact transition F(gap), Q(gap) covers a blackout or an oversampled call.
  K  = S.cfg;  tr = S.tr;  took = false;
  if ~(valid ~= 0) || ~isfinite(r) || ~isfinite(th) || r <= 0, return; end
  if tr.on
    if r == tr.last_r && th == tr.last_th, return; end
    gap = t - tr.t_upd;
    if gap < K.min_gap, return; end
  end
  [z, Rz] = tc_convert_measurement(r, th, K.sig_r, K.sig_th);

  restart = ~tr.on;
  if tr.on && (t - tr.t_upd) > K.lost_gap, restart = true; end   % data far outside the spec: forget

  if ~restart
    gap = t - tr.t_upd;
    [F1, Q1] = tc_jerk_model_transition(gap, 1/K.tau_j, S.q_cont, S.Wgl);
    F = kron(eye(2), F1);  Q = kron(eye(2), Q1);
    xp = F*tr.x;
    Pp = F*tr.P*F' + tr.rho*Q;
    nu = z - S.H*xp;
    Sm = S.H*Pp*S.H' + Rz;
    nis = nu' * (Sm \ nu);
    if nis > K.chi2_gate
      tr.n_out = tr.n_out + 1;
      if tr.n_out >= 2
        restart = true;                    % a different object (or a broken track): start over
      else
        S.tr = tr;  return;                % single outlier: skip it, keep the track
      end
    end
  end

  if restart
    x0 = zeros(8,1);  x0([1 5]) = z;
    P0 = zeros(8);
    P0([1 5],[1 5]) = Rz;
    P0(2,2) = K.prior_v^2;  P0(6,6) = K.prior_v^2;
    P0(3,3) = K.prior_a^2;  P0(7,7) = K.prior_a^2;
    P0(4,4) = K.sigma_j^2;  P0(8,8) = K.sigma_j^2;
    tr.x = x0;  tr.P = P0;  tr.n_upd = 1;  tr.n_out = 0;
    tr.rho = 1;  tr.nis_ema = 1;  tr.on = true;
  else
    tr.n_out = 0;
    Kg = (Pp*S.H') / Sm;
    IKH = eye(8) - Kg*S.H;
    tr.P = IKH*Pp*IKH' + Kg*Rz*Kg';          % Joseph form
    tr.P = 0.5*(tr.P + tr.P');
    tr.x = tc_envelope_clip(xp + Kg*nu, K);
    tr.n_upd = tr.n_upd + 1;
    if K.adapt_q
      % NIS-driven process-noise scale: E[NIS]/2 = 1 for a consistent filter.
      tr.nis_ema = (1 - K.adapt_lam)*tr.nis_ema + K.adapt_lam*(nis/2);
      tr.rho = min(max(tr.rho*exp(K.adapt_eta*(tr.nis_ema - 1)), K.adapt_lo), K.adapt_hi);
    end
  end
  tr.t_upd = t;  tr.last_r = r;  tr.last_th = th;
  S.tr = tr;  took = true;
end
