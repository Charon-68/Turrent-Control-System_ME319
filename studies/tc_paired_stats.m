function s = tc_paired_stats(a, b, nboot, nperm)
% TC_PAIRED_STATS  Paired comparison of two per-seed metric vectors (same seeds, same noise).
%   mean difference b - a, percentile-bootstrap 95 % CI, two-sided sign-flip permutation p.
  d = b(:) - a(:);  d = d(isfinite(d));  n = numel(d);
  if nargin < 3, nboot = 4000; end
  if nargin < 4, nperm = 4000; end
  u = tc_uniform_draws(2044, nboot*n);  idx = floor(reshape(u, nboot, n)*n) + 1;
  bm = mean(reshape(d(idx), nboot, n), 2);
  s.mean = mean(d);  s.ci = [tc_percentile(bm, 2.5), tc_percentile(bm, 97.5)];  s.n = n;
  u2 = tc_uniform_draws(7, nperm*n);  sg = 2*(reshape(u2, nperm, n) < 0.5) - 1;
  pm = abs(mean(sg .* d', 2));
  s.p = (1 + sum(pm >= abs(s.mean) - 1e-15)) / (nperm + 1);
end
