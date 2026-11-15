function [d3, lin] = tc_reference_chain(d, D, K)
% TC_REFERENCE_CHAIN  Deviation-reference law with acquisition clamps.
%
% d = [y-th^, y'-th^', y''-th^'']  (deviation of the reference from the nowcast),
% D = nowcast tuple [th^ th^' th^'' th^'''].  Returns d3 = y''' - th^'''.
%
% Cascade (gains p/3, p, 3p) built so that, while no clamp is active,
%     d''' = -3p d'' - 3p^2 d' - p^3 d      i.e.  (D+p)^3 d = 0   (exactly linear).
% The two clamps only bite during acquisition (large slew): the ABSOLUTE rate
% demand is limited to the back-EMF-limited cruise rate and the acceleration
% demand to acq_accel.  Optional braking curve (acq_decel > 0): beyond the
% linear zone the rate demand follows a constant-deceleration parabola
% v = sqrt(v_b^2 + 2 a_b (|e| - e_b)) so the turret brakes late instead of
% creeping in on the exponential tail of the linear law.
  p  = K.p_dev;
  g  = (p/3)*d(1);
  zb = K.acq_zone;
  braking = K.acq_decel > 0 && abs(d(1)) > zb;
  if braking
    vb = (p/3)*zb;
    g  = sign(d(1))*sqrt(vb^2 + 2*K.acq_decel*(abs(d(1)) - zb));
  end
  w_u = D(2) - g;
  w   = min(max(w_u, -K.acq_rate), K.acq_rate);
  a_u = D(3) - p*((D(2) + d(2)) - w);
  a   = min(max(a_u, -K.acq_accel), K.acq_accel);
  d3  = -3*p*((D(3) + d(3)) - a);
  lin = (w == w_u) && (a == a_u) && ~braking;
end
