function tf = tc_session_expired(S, t)
% TC_SESSION_EXPIRED  Decide whether this call belongs to a NEW run.
% Reset when time went backwards, returned to (numerically) zero after earlier calls
% (tolerance, not ==, so an evaluator whose first time is 1e-15 still resets), or the
% gap since the last call is far longer than any real controller period.
  tf = (t < S.t_prev - 1e-9) ...
    || (S.n_calls > 0 && abs(t) < 1e-9 && S.t_prev > 1e-9) ...
    || (t - S.t_prev > 3.0);
end
