function ok = tc_scenario_check(d, tg, el_lo, el_hi)
% TC_SCENARIO_CHECK  Envelope check of a scenario definition on the grid tg:
% 800 <= r <= 2000, |v| <= 150, |a| <= 20, elevation within [el_lo, el_hi] degrees.
  tg = tg(tg <= 10 + 1e-9);                  % the envelope is an obligation for the 10 s engagement
  [p, v, a] = tc_scenario_integrate(d, tg);
  r = hypot(p(1,:), p(2,:));
  el = atan2(p(2,:), p(1,:))*180/pi;
  ok = all(r >= 800) && all(r <= 2000) && all(hypot(v(1,:), v(2,:)) <= 150) ...
       && all(hypot(a(1,:), a(2,:)) <= 20 + 1e-9) && all(el >= el_lo) && all(el <= el_hi);
end
