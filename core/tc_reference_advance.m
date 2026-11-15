function S = tc_reference_advance(S, Dpre, dt)
% TC_REFERENCE_ADVANCE  Advance the deviation state S.dev over dt seconds, relative to
% the (old-estimate) nowcast tuple Dpre valid at the END of the interval.
%
% Linear region: exact transition expm(A_d dt) of the triple-pole companion
% matrix (cached per dt).  Acquisition region: sub-stepping (<= 2 ms) of the
% clamped cascade, reading the nowcast along its own cubic Taylor model.
  K = S.cfg;  d = S.dev;
  if dt <= 0, return; end
  D0 = tc_taylor_advance(Dpre, -dt);
  [~, lin0] = tc_reference_chain(d, D0, K);
  if lin0
    if ~(abs(S.Phi_dt - dt) <= 1e-12)          % written so a NaN (cache empty) also recomputes
      p = K.p_dev;
      A = [0 1 0; 0 0 1; -p^3 -3*p^2 -3*p];
      S.Phi = expm(A*dt);  S.Phi_dt = dt;
    end
    dE = S.Phi*d;
    [~, lin1] = tc_reference_chain(dE, Dpre, K);
    if lin1, S.dev = dE; return; end
  end
  n = max(1, ceil(dt/0.002));  h = dt/n;
  for k = 1:n
    Dk = tc_taylor_advance(Dpre, (k-1)*h - dt);
    d3 = tc_reference_chain(d, Dk, K);
    d(3) = d(3) + d3*h;
    d(2) = d(2) + d(3)*h;
    d(1) = d(1) + d(2)*h;
  end
  S.dev = d;
end
