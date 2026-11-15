function [P, sig, cons] = tc_hit_probability(S, x, Pc, th_T)
% TC_HIT_PROBABILITY  Probability that a pulse fired NOW lands within R_eng.
%
% Rao-Blackwellised mixture over the target's CURRENT acceleration a:
%   a ~ N(mu_a, P_aa) truncated to the physical envelope |a| <= a_env,
%   position | a ~ N(mu_p + K (a - mu_a), P_pp - K P_ap),   K = P_pa P_aa^-1.
% The signed beam-to-target offset is the projection of the position on the
% BEAM normal n_b = [-sin th_T, cos th_T].  P = sum_i w_i [Phi((R - d_i)/s) -
% Phi((-R - d_i)/s)] with a 25-node tensor Gauss-Hermite rule.  Nothing is
% hand-picked and no variance is counted twice (the conditional covariance
% removes the part of P_pp that the acceleration already explains).
  K = S.cfg;
  ib = [cos(th_T); sin(th_T)];  nb = [-sin(th_T); cos(th_T)];
  ip = [1 5];  ia = [3 7];
  mp = x(ip);  ma = x(ia);
  P = 0;  sig = 1e3;  cons = 0;
  if mp'*ib <= 0, return; end                       % target behind the beam
  Ppp = Pc(ip,ip);  Ppa = Pc(ip,ia);  Paa = Pc(ia,ia);
  Paa = 0.5*(Paa + Paa') + 1e-9*eye(2);
  Kg  = Ppa/Paa;
  Pcond = Ppp - Kg*Ppa';
  s2 = max(nb'*Pcond*nb, 1e-6);  sig = sqrt(s2);
  d0 = nb'*mp;
  A  = ma + chol(Paa, 'lower')*S.Zgh;               % 2 x n acceleration hypotheses
  w  = S.Wgh .* (sum(A.^2, 1) <= K.a_env_hyp^2);
  if sum(w) < 1e-9                                  % posterior entirely outside the envelope
    w = S.Wgh;  nrm = sqrt(sum(A.^2, 1));  sc = min(1, K.a_env_hyp./max(nrm, eps));  A = A.*sc;
  end
  w  = w/sum(w);
  di = d0 + (nb'*Kg)*(A - ma);                      % 1 x n offsets
  R  = K.r_eng;
  P  = sum(w .* (tc_normal_cdf((R - di)/sig) - tc_normal_cdf((-R - di)/sig)));
  cons = sum(w .* (abs(di) <= R));                  % posterior mass of futures whose centre offset is on target
end
