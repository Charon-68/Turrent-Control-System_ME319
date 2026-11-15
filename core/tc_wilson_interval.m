function ci = tc_wilson_interval(k, n, z)
% TC_WILSON_INTERVAL  Wilson score interval for a binomial proportion (default 95 %).
  if nargin < 3, z = 1.959964; end
  p = k/n;  den = 1 + z^2/n;
  c = (p + z^2/(2*n))/den;
  h = z*sqrt(p*(1-p)/n + z^2/(4*n^2))/den;
  ci = [max(c - h, 0), min(c + h, 1)];
end
