function [N, D] = tc_pade_delay(Td, n)
% TC_PADE_DELAY  Pade approximation of exp(-s Td): N(s)/D(s), order n (default 8), coefficient rows, highest power first.
  if nargin < 2, n = 8; end
  c = zeros(1, n + 1);
  for k = 0:n, c(k+1) = factorial(2*n - k)*factorial(n)/(factorial(2*n)*factorial(k)*factorial(n - k)); end
  D = fliplr(c.*Td.^(0:n));  N = fliplr(c.*(-Td).^(0:n));
end
