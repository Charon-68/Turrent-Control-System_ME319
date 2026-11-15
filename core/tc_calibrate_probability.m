function pc = tc_calibrate_probability(T, p)
% TC_CALIBRATE_PROBABILITY  Piecewise-linear isotonic map of the raw probability.
  kx = T.kx;  ky = T.ky;
  if p <= kx(1), pc = ky(1); return; end
  if p >= kx(end), pc = ky(end); return; end
  j = find(kx <= p, 1, 'last');
  pc = ky(j) + (ky(j+1) - ky(j))*(p - kx(j))/(kx(j+1) - kx(j));
end
