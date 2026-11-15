function [F, Q] = tc_jerk_model_transition(tau, alpha, q, W)
% TC_JERK_MODEL_TRANSITION  Exact discretisation of the per-axis Markov-jerk model.
%
%   state s = [pos; vel; acc; jerk],  jerk' = -alpha*jerk + w,  E[w w'] = q delta
%
% Closed form for F (no matrix exponential).  Q = q * int_0^tau g(s) g(s)' ds,
% where g is the impulse response of the state to the driving noise; the
% integrand is smooth, so an n-point Gauss-Legendre rule (W.x, W.w on [-1,1])
% is exact to ~1e-12.  Verified against Van Loan expm in tests/.
  e  = exp(-alpha*tau);
  em = -expm1(-alpha*tau);                  % 1 - e, accurate for small tau
  c1 = em/alpha;                            % int_0^tau e^{-alpha s} ds
  c2 = (tau - c1)/alpha;
  c3 = (tau^2/2 - c2)/alpha;
  F = [1, tau, tau^2/2, c3;
       0, 1,   tau,     c2;
       0, 0,   1,       c1;
       0, 0,   0,       e ];
  if nargout > 1
    s  = 0.5*tau*(W.x + 1);                 % quadrature abscissae in [0, tau]
    es = -expm1(-alpha*s);
    gj = exp(-alpha*s);
    ga = es/alpha;
    gv = (s - ga)/alpha;
    gp = (s.^2/2 - gv)/alpha;
    G  = [gp'; gv'; ga'; gj'];              % 4 x n
    Q  = q*0.5*tau * (G .* W.w') * G';
    Q  = 0.5*(Q + Q');
  end
end
