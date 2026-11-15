function d = tc_los_derivatives(x)
% TC_LOS_DERIVATIVES  Elevation angle and its first three time derivatives, exact.
%   x = [px vx ax jx  py vy ay jy]'   ->   d = [theta; theta'; theta''; theta''']
% With z = px + j py: theta = arg z and theta^(k) = Im( (log z)^(k) ), where the
% log-derivatives follow from w_k = z^(k)/z:
%   L1 = w1,  L2 = w2 - w1^2,  L3 = w3 - 3 w1 w2 + 2 w1^3.
% Valid for any trajectory (no small-angle step), no numerical differentiation.
  z0 = complex(x(1), x(5));
  w1 = complex(x(2), x(6))/z0;
  w2 = complex(x(3), x(7))/z0;
  w3 = complex(x(4), x(8))/z0;
  d = [atan2(x(5), x(1));
       imag(w1);
       imag(w2 - w1^2);
       imag(w3 - 3*w1*w2 + 2*w1^3)];
end
