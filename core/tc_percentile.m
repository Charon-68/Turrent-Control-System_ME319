function v = tc_percentile(x, p)
% TC_PERCENTILE  Percentile by linear interpolation of the sorted sample (no statistics package).
  x = sort(x(:));  n = numel(x);
  if n == 0, v = NaN; return; end
  pos = 1 + (n-1)*p/100;  lo = floor(pos);  hi = ceil(pos);
  v = x(lo) + (pos - lo)*(x(hi) - x(lo));
end
