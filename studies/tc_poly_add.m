function p = tc_poly_add(a, b)
% TC_POLY_ADD  Sum of two polynomials given as coefficient rows of different length.
  n = max(numel(a), numel(b));  p = [zeros(1, n - numel(a)), a] + [zeros(1, n - numel(b)), b];
end
