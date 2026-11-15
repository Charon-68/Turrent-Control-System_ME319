function [s, M] = tc_plant_step(M, s, V, h, method)
% TC_PLANT_STEP  Advance the plant state s = [theta; omega; i] by h seconds, voltage held
% (zero-order hold), then apply the mechanical stops.
%   'exact'  exact ZOH discretisation (any h; valid only without friction/dead-zone)
%   'rk4'    classical RK4 through turret_plant semantics (use h <= 1 ms)
%   'euler'  forward Euler (stable only for h < ~3.3 ms; kept as the worst-case integrator)
  V = min(max(V, -M.Vlim), M.Vlim);
  if M.fric.deadzone > 0 && abs(V) <= M.fric.deadzone, V = 0; end
  nonlin = M.fric.coulomb > 0 || M.fric.deadzone > 0;
  if strcmp(method, 'exact') && ~nonlin
    if ~(abs(M.exact_h - h) <= 1e-15)          % NaN-safe: empty cache must recompute
      E = expm([M.A, M.B; zeros(1,4)]*h);
      M.Ad = E(1:3,1:3);  M.Bd = E(1:3,4);  M.exact_h = h;
    end
    s = M.Ad*s + M.Bd*V;
  elseif strcmp(method, 'euler')
    s = s + h*tc_plant_rate(M, s, V);
  else
    k1 = tc_plant_rate(M, s, V);
    k2 = tc_plant_rate(M, s + 0.5*h*k1, V);
    k3 = tc_plant_rate(M, s + 0.5*h*k2, V);
    k4 = tc_plant_rate(M, s + h*k3, V);
    s = s + (h/6)*(k1 + 2*k2 + 2*k3 + k4);
  end
  if s(1) >= M.th_hi && s(2) > 0, s(1) = M.th_hi; s(2) = 0; end   % hard stops
  if s(1) <= M.th_lo && s(2) < 0, s(1) = M.th_lo; s(2) = 0; end
end

function ds = tc_plant_rate(M, s, V)
  % Same equations as turret_plant.m, with optional Coulomb friction (smoothed sign).
  i_dot = (V - M.R*s(3) - M.N*M.Ke*s(2))/M.L;
  tq = M.N*M.Kt*s(3) - M.b*s(2) - M.fric.coulomb*tanh(s(2)/1e-3);
  ds = [s(2); tq/M.J; i_dot];
end
