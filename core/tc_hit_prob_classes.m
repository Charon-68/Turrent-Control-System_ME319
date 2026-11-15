function [P, sig, cons] = tc_hit_prob_classes(S, x, Pc, th_T, tau)
% TC_HIT_PROB_CLASSES  Baseline hit probability with 17 hand-picked maneuver classes
% (nominal, weight 1/2, plus magnitudes {a_max/2, a_max} in 8 directions), each
% displacing the nominal position by (a_i - a_hat) tau^2 / 2 since the last
% radar sample.  Kept only as the comparison baseline for tc_hit_probability.
  K = S.cfg;
  nb = [-sin(th_T); cos(th_T)];  ib = [cos(th_T); sin(th_T)];
  mp = x([1 5]);  ma = x([3 7]);
  P = 0;  cons = 0;
  s2 = max(nb'*Pc([1 5],[1 5])*nb, 1e-6);  sig = sqrt(s2);
  if mp'*ib <= 0, return; end
  ang = (0:7)*pi/4;
  mag = K.a_tgt_max*[0.5; 1];
  Ac  = [0, reshape((mag*cos(ang))', 1, []); 0, reshape((mag*sin(ang))', 1, [])];
  w   = [0.5, repmat(0.5/16, 1, 16)];
  di  = nb'*mp + nb'*((Ac - ma)*(0.5*tau^2));
  di(1) = nb'*mp;                                   % nominal class: no displacement
  R = K.r_eng;
  P = sum(w .* (tc_normal_cdf((R - di)/sig) - tc_normal_cdf((-R - di)/sig)));
  cons = mean(abs(di) <= R);                        % fraction of the 17 classes on target
end
