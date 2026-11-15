function p = tc_normal_cdf(x)
% TC_NORMAL_CDF  Standard normal CDF (vectorised, no toolbox).
  p = 0.5*erfc(-x/sqrt(2));
end
