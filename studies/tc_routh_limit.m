function [kmax, kfirst] = tc_routh_limit(Ln, Ld)
% TC_ROUTH_LIMIT  Largest loop-gain scale k such that den(L) + k num(L) passes the Routh test.
%   Bisection on the sign pattern of the Routh first column (tc_routh_first_column), not on roots;
%   kfirst is the same limit found from the roots (max real part), as a cross-check.
%   Returns Inf if still stable at k = 1e6.
  stable = @(k) all(tc_routh_first_column(tc_poly_add(Ld, k*Ln)) > 0);
  if ~stable(1e-6), kmax = 0;  kfirst = 0;  return; end
  if stable(1e6), kmax = Inf;  kfirst = Inf;  return; end
  lo = 1e-6;  hi = 1e6;
  for it = 1:200, mid = sqrt(lo*hi);  if stable(mid), lo = mid; else, hi = mid; end, end
  kmax = lo;
  lo = 1e-6;  hi = 1e6;
  for it = 1:200, mid = sqrt(lo*hi);  if max(real(roots(tc_poly_add(Ld, mid*Ln)))) < 0, lo = mid; else, hi = mid; end, end
  kfirst = lo;
end
