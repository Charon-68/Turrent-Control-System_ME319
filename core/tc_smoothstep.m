function s = tc_smoothstep(u)
% TC_SMOOTHSTEP  Quintic 6u^5 - 15u^4 + 10u^3 on [0,1] (C2: finite jerk at the joins).
  u = min(max(u, 0), 1);
  s = u.^3 .* (10 - 15*u + 6*u.^2);
end
