function d = tc_scenario_random(family, seed)
% TC_SCENARIO_RANDOM  Reproducible random ADMISSIBLE engagements by rejection sampling.
%   'MC' : smooth sinusoid-sum maneuvers (peak jerk <= ~170 m/s^3), 1-2 blackouts.
%   'HB' : bang-bang acceleration reversals with 0.15 s transitions (jerk up to ~450 m/s^3),
%          maneuver onsets that can fall INSIDE a blackout, blackout starting exactly at
%          1.0 s and back-to-back 0.5 s blackouts (spec limit, 0.5 s apart).
% Seed ranges are a PROTOCOL, not a convention: tuning/calibration uses 1..200 only; every
% reported statistic uses a disjoint, never-fitted range (5000+).  See docs/PROTOCOL.md.
  tg = (0:1e-3:11);
  for attempt = 0:499
    u = tc_uniform_draws(seed*10007 + 50021*attempt + (family(1) == 'H')*977, 64);
    r0 = 1500 + 500*u(1);  th0 = (8 + 47*u(2))*pi/180;  sp = 70 + 80*u(3);
    hd = pi + th0 + (2*u(4) - 1)*40*pi/180;
    p0 = r0*[cos(th0); sin(th0)];  v0 = sp*[cos(hd); sin(hd)];
    if family(1) == 'M'
      [acc, bo] = mc_profile(u, tg);
    else
      [acc, bo] = hb_profile(u, tg, seed);
    end
    a = acc(tg);
    pk = max(hypot(a(1,:), a(2,:)));
    if pk < 1e-9, continue; end
    scale = (0.4 + 0.6*u(29))*0.97*20/pk;                % peak between 40 % and 100 % of 0.97 a_max
    if family(1) == 'H', scale = (0.9 + 0.07*u(29))*20/pk; end
    d.p0 = p0;  d.v0 = v0;  d.acc = @(t) scale*acc(t);  d.bo = bo;
    if tc_scenario_check(d, tg, 8, 83), return; end
  end
  error('tc_scenario_random: no admissible draw for %s%d', family, seed);
end

function [acc, bo] = mc_profile(u, tg) %#ok<INUSD>
  f  = 0.1 + 0.9*reshape(u(5:10), 2, 3);   ph = 2*pi*reshape(u(11:16), 2, 3);
  am = 0.2 + 0.8*reshape(u(17:22), 2, 3);
  t_on = 0.5 + 2.5*u(23);
  brk = u(24) < 0.5;  t_b = 1 + 6*u(25);  len = 0.5 + 1.5*u(26);  rmp = 0.15 + 0.45*u(27);
  dirb = [cos(2*pi*u(28)); sin(2*pi*u(28))];
  acc = @(t) mc_acc(t, f, ph, am, t_on, brk, t_b, len, rmp, dirb);
  b1 = 1 + 7.5*u(30);  l1 = 0.1 + 0.4*u(31);
  bo = [b1 l1];
  if u(32) < 0.3
    s2 = b1 + l1 + 0.5 + (9.5 - (b1 + l1 + 0.5))*u(33);  l2 = 0.1 + 0.4*u(34);
    if s2 + l2 < 9.5, bo = [bo; s2 l2]; end
  end
end

function a = mc_acc(t, f, ph, am, t_on, brk, t_b, len, rmp, dirb)
  t = t(:)';  a = zeros(2, numel(t));
  for ax = 1:2
    for k = 1:3
      a(ax,:) = a(ax,:) + am(ax,k)*sin(2*pi*f(ax,k)*t + ph(ax,k));
    end
  end
  a = a .* tc_smoothstep((t - t_on)/0.5);
  if brk, a = a + 1.5*dirb*tc_smooth_pulse(t, t_b, t_b + len, rmp); end
end

function [acc, bo] = hb_profile(u, tg, seed) %#ok<INUSD>
  dirp = 2*pi*u(5);  nrm = [cos(dirp); sin(dirp)];
  inside = mod(seed, 3) == 0;                              % first reversal lands inside blackout 1
  t1 = 1.0 + 0.1 + 0.3*u(6);  if ~inside, t1 = 1.0 + 2*u(6); end
  gaps = 0.6 + 0.8*u(7:16);  sw = t1 + [0; cumsum(gaps(1:9))];
  sgn0 = 1 - 2*(u(17) < 0.5);
  acc = @(t) hb_acc(t, nrm, sw, sgn0);
  if mod(seed, 2) == 1
    bo = [1.0 0.5; 2.0 0.5];                               % exactly at the onset limit, back-to-back
  else
    b1 = 1 + 6*u(30);  bo = [b1 0.1 + 0.4*u(31)];
  end
end

function a = hb_acc(t, nrm, sw, sgn0)
  t = t(:)';  s = sgn0*tc_smoothstep(t/0.15);
  for k = 1:numel(sw)
    s = s - 2*sgn0*(-1)^(k+1) * tc_smoothstep((t - sw(k) + 0.075)/0.15);
  end
  a = nrm * s;
end
