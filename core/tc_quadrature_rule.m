function W = tc_quadrature_rule(kind, n)
% TC_QUADRATURE_RULE  Gauss-Legendre ('legendre') or probabilists' Gauss-Hermite ('hermite')
% nodes and weights by Golub-Welsch.  W.x, W.w are column vectors; Hermite
% weights sum to 1 (standard-normal expectation).
  k = (1:n-1)';
  switch kind
    case 'legendre'
      beta = k ./ sqrt(4*k.^2 - 1);
      mass = 2;
    case 'hermite'
      beta = sqrt(k);
      mass = 1;
    otherwise
      error('tc_quadrature_rule: unknown kind');
  end
  J = diag(beta, 1) + diag(beta, -1);
  [V, D] = eig(J);
  [x, ix] = sort(diag(D));
  W.x = x;
  W.w = mass * (V(1, ix)').^2;
end
