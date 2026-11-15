function [p, v, a] = tc_scenario_integrate(d, tg)
% TC_SCENARIO_INTEGRATE  Trapezoidal integration of the acceleration profile on grid tg.
  a = d.acc(tg);
  v = d.v0 + cumtrapz(tg, a, 2);
  p = d.p0 + cumtrapz(tg, v, 2);
end
