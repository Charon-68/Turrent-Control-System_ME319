function M = tc_course_model(scale)
% TC_COURSE_MODEL  Polynomial model of the final loop, built from tc_params (read only).
%   M = tc_course_model()                 nominal plant and the final PI + roll-off compensator
%   M = tc_course_model(struct('Kt',0.8)) multipliers on Kt, Ke, J (J_eq), b, R, L (missing = 1)
% Same formulas as studies/tc_plot_design_analysis.m (no Control Toolbox):
%   G_p = gn / (s a(s)),  a = [L J, L b + R J, R b + N^2 Kt Ke],  C = Kp (Ti s + 1) / (Ti s (tau_f s + 1)).
% Fields: gn, a, Gn, Gd (plant num/den), Cn, Cd, Ln, Ld (loop), cl (closed-loop char. polynomial).
  K = tc_params();
  if nargin < 1, scale = struct(); end
  f = @(n) get_scale(scale, n);
  L = K.L*f('L');  R = K.R*f('R');  J = K.J_eq*f('J');  b = K.b*f('b');  Kt = K.Kt*f('Kt');  Ke = K.Ke*f('Ke');
  M.gn = K.N*Kt;
  M.a  = [L*J, L*b + R*J, R*b + K.N^2*Kt*Ke];
  M.Gn = M.gn;  M.Gd = conv([1 0], M.a);
  M.Cn = K.kp*[K.ti 1];  M.Cd = conv([K.ti 0], [K.tau_f 1]);
  M.Ln = conv(M.Gn, M.Cn);  M.Ld = conv(M.Gd, M.Cd);
  M.cl = tc_poly_add(M.Ld, M.Ln);
  M.K = K;
end

function v = get_scale(s, n)
  if isfield(s, n), v = s.(n); else, v = 1; end
end
