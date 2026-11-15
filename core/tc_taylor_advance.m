function d2 = tc_taylor_advance(d, s)
% TC_TAYLOR_ADVANCE  Move a derivative 4-tuple [f f' f'' f'''] by s seconds along
% its own cubic Taylor model (s may be negative).  Used for the Taylor
% prediction (s = t_p), the ZOH lead, and sub-stepping inside a call.
  d2 = [d(1) + d(2)*s + d(3)*s^2/2 + d(4)*s^3/6;
        d(2) + d(3)*s + d(4)*s^2/2;
        d(3) + d(4)*s;
        d(4)];
end
