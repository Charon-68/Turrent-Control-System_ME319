function [theta_dot_turret, theta_ddot_turret, i_dot] = turret_plant(t, V_m, theta_turret, theta_dot_turret, i) %#ok<INUSL>
% TURRET_PLANT  Elevation drive from first principles (state derivative form).
%
%   Armature :  L di/dt   = V - R i - N Ke thdot            (back-EMF is N*Ke*thdot)
%   Mechanics:  J_eq thdd = N Kt i - b thdot                (J_eq = J_turret + N^2 J_motor)
%
% The gear ratio N reflects the motor (which spins N times faster) onto the
% turret shaft: inertia scales by N^2, torque by N, back-EMF speed by N.
% theta_dot_turret is returned unchanged as d(theta)/dt, so the state vector
% is [theta; theta_dot; i].  The supply limit |V| <= V_lim is applied here.
% Integrate with a step <= 1 ms (forward Euler is unstable above ~3.3 ms) or RK4.
% theta_turret and t are unused by the (time-invariant, linear) dynamics but
% kept so the signature matches the assignment interface.
% Constants come from tc_params (cached; use `clear turret_plant` after changing them).

  persistent c
  if isempty(c)
    tc_setup();
    K = tc_params();                       % single source of constants
    c = struct('J', K.J_eq, 'b', K.b, 'N', K.N, 'Kt', K.Kt, 'Ke', K.Ke, ...
               'R', K.R, 'L', K.L, 'Vlim', K.V_lim);
  end
  V = min(max(V_m, -c.Vlim), c.Vlim);
  i_dot             = (V - c.R*i - c.N*c.Ke*theta_dot_turret) / c.L;
  theta_ddot_turret = (c.N*c.Kt*i - c.b*theta_dot_turret) / c.J;
end
