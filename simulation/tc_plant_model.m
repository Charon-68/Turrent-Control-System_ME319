function M = tc_plant_model(scale, fric)
% TC_PLANT_MODEL  Simulation-side plant (the ground truth the controller does NOT see).
%   scale  struct of multipliers on R, L, J, b, Kt, Ke (default all 1): plant-mismatch studies
%   fric   struct('coulomb', N m, 'deadzone', V) for the optional non-ideal plant (default none)
% Returns constants and the continuous-time linear matrices [theta; omega; i].
  K = tc_params();
  if nargin < 1 || isempty(scale), scale = struct(); end
  if nargin < 2 || isempty(fric),  fric  = struct('coulomb', 0, 'deadzone', 0); end
  sc = @(f) ifelse_field(scale, f);
  M.R = K.R*sc('R');  M.L = K.L*sc('L');  M.J = K.J_eq*sc('J');  M.b = K.b*sc('b');
  M.N = K.N;  M.Kt = K.Kt*sc('Kt');  M.Ke = K.Ke*sc('Ke');  M.Vlim = K.V_lim;
  M.fric = fric;
  M.A = [0 1 0; 0 -M.b/M.J M.N*M.Kt/M.J; 0 -M.N*M.Ke/M.L -M.R/M.L];
  M.B = [0; 0; 1/M.L];
  M.th_lo = K.th_min;  M.th_hi = K.th_max;
  M.exact_h = NaN;  M.Ad = eye(3);  M.Bd = zeros(3,1);
end

function v = ifelse_field(s, f)
  if isfield(s, f), v = s.(f); else, v = 1; end
end
