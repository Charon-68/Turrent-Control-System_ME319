function [kx, ky] = tc_isotonic_fit(p, y, nknots)
% TC_ISOTONIC_FIT  Pool-adjacent-violators fit of E[y | p] (non-decreasing), reduced to at most
% nknots piecewise-linear knots.  Anchored at (0,0) and (1,1) so the map stays a probability.
  [p, ix] = sort(p(:));  y = y(ix);  n = numel(p);
  val = y(:)';  wt = ones(1, n);  px = p(:)';
  % PAV with block merging
  v = zeros(1, n);  w = zeros(1, n);  xs = zeros(1, n);  m = 0;
  for i = 1:n
    m = m + 1;  v(m) = y(i);  w(m) = 1;  xs(m) = p(i);
    while m > 1 && v(m-1) > v(m)
      v(m-1) = (w(m-1)*v(m-1) + w(m)*v(m))/(w(m-1) + w(m));
      xs(m-1) = (w(m-1)*xs(m-1) + w(m)*xs(m))/(w(m-1) + w(m));   % weighted mean abscissa
      w(m-1) = w(m-1) + w(m);  m = m - 1;
    end
  end
  v = v(1:m);  w = w(1:m);  xs = xs(1:m);
  % thin to nknots by merging the smallest-weight neighbours
  while numel(v) > nknots - 2
    [~, j] = min(w(1:end-1) + w(2:end));
    v(j) = (w(j)*v(j) + w(j+1)*v(j+1))/(w(j) + w(j+1));
    xs(j) = (w(j)*xs(j) + w(j+1)*xs(j+1))/(w(j) + w(j+1));
    w(j) = w(j) + w(j+1);  v(j+1) = [];  xs(j+1) = [];  w(j+1) = [];
  end
  kx = [0, xs, 1];  ky = [0, v, 1];
  [kx, iu] = unique(kx);  ky = ky(iu);
  ky = cummax(ky);  ky = min(max(ky, 0), 1);
end
