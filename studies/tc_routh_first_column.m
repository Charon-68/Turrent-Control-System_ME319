function c = tc_routh_first_column(p)
% TC_ROUTH_FIRST_COLUMN  First column of the Routh array of p(s) (coefficients, highest power first).
%   The polynomial is Hurwitz iff all entries are > 0 (after making the leading coefficient positive).
%   A zero first element is replaced by a tiny epsilon (regular special case); the result is only
%   used with bisection on the gain, where that case does not occur.
  p = p(:).';  p = p(find(p ~= 0, 1):end);  if p(1) < 0, p = -p; end
  n = numel(p) - 1;  w = ceil((n + 1)/2);
  R = zeros(n + 1, w);  R(1, 1:numel(p(1:2:end))) = p(1:2:end);  R(2, 1:numel(p(2:2:end))) = p(2:2:end);
  for i = 3:n+1
    if R(i-1, 1) == 0, R(i-1, 1) = 1e-12*max(abs(p)); end
    for j = 1:w-1
      R(i, j) = (R(i-1, 1)*R(i-2, j+1) - R(i-2, 1)*R(i-1, j+1))/R(i-1, 1);
    end
  end
  c = R(:, 1);
end
