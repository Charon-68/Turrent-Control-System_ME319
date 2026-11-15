function y = tc_lin_response(num, den, kind, amp, t)
% TC_LIN_RESPONSE  Exact time response of num/den by partial fractions (residue), no state space.
%   kind: 'imp' (impulse), 'step', 'ramp', 'par' (parabola amp*t^2/2).  Repeated poles are handled
%   through residue's multiplicities (a companion form is hopelessly ill-conditioned: coefficients
%   span ten decades).  Same method as lin_sim in tc_plot_design_analysis.m.
  m = find(strcmp(kind, {'imp', 'step', 'ramp', 'par'})) - 2;      % extra integrators
  if m < 0, dtot = den; else, dtot = conv(den, [1 zeros(1, m + 1)]); end
  [r, p, kd, ee] = residue(amp*num, dtot);
  y = zeros(size(t));
  for q = 1:numel(r), y = y + real(r(q) * t.^(ee(q) - 1) / factorial(ee(q) - 1) .* exp(p(q)*t)); end
  if ~isempty(kd), y = y + kd(end)*(t == 0); end
end
