function e = tc_scenario_build(name)
% TC_SCENARIO_BUILD  Build (and validate) one scenario: truth on a 1 ms grid, blackout list and
% the radar noise table (one row per radar tick, so results do not depend on the caller's
% step).  Name grammar:  BASE[+flag...]
%   BASE  A B C D1..D6 | MC<seed> | HB<seed> | <BASE>#<k>  (same trajectory, noise draw k)
%   flag  glint  heavy-tailed outliers on 5 % of ticks (out of spec, stress only)
%         range  noise grows with range  (sigma scaled by r/1500)
%         bias   constant radar bias  (+2 m, +0.02 deg)
  parts = strsplit(name, '+');  base = parts{1};  flags = parts(2:end);
  k = 0;  hash = strfind(base, '#');
  if ~isempty(hash), k = str2double(base(hash+1:end));  base = base(1:hash-1); end
  tok = regexp(upper(base), '^(MC|HB)(\d+)$', 'tokens', 'once');
  if isempty(tok), d = tc_scenario_catalog(base); else, d = tc_scenario_random(tok{1}, str2double(tok{2})); end

  e.name = name;  e.dt = 1e-3;  e.t_end = 11;
  e.tg = 0:e.dt:e.t_end;
  [p, v, a] = tc_scenario_integrate(d, e.tg);
  e.truth = [p; v; a];                                       % 6 x N
  e.bo = d.bo;
  % The assignment's envelope applies to the 10 s engagement; truth is generated to 11 s only
  % so that a call at the last step never reads past the table.  Check 0..10 s.
  w = e.tg <= 10 + 1e-9;
  rr = hypot(p(1,w), p(2,w));
  if ~(all(rr >= 800 - 1e-6) && all(rr <= 2000) && all(hypot(v(1,w), v(2,w)) <= 150) ...
       && all(hypot(a(1,w), a(2,w)) <= 20 + 1e-9))
    error('tc_scenario_build: %s leaves the admissible envelope', name);
  end

  K = tc_params();
  nt = round(e.t_end/K.dt_radar) + 1;
  u = tc_uniform_draws(tc_seed_from_name(base) + 7919*k, 4*nt);
  u1 = u(1:2*nt); u2 = u(2*nt+1:end);
  z = sqrt(-2*log(max(u1, eps))) .* cos(2*pi*u2);            % Box-Muller (standard normal)
  z = reshape(z, nt, 2);
  e.nz = z .* [K.sig_r, K.sig_th];
  e.bias = [0 0];
  for f = flags
    switch f{1}
      case 'glint'
        ug = tc_uniform_draws(tc_seed_from_name(base) + 31, 2*nt);
        hit = ug(1:nt) < 0.05;
        e.nz(hit, :) = e.nz(hit, :) .* [6 4];
      case 'range'
        e.range_scaled = true;
      case 'bias'
        e.bias = [2, 0.02*pi/180];
      otherwise
        error('tc_scenario_build: unknown flag %s', f{1});
    end
  end
  if ~isfield(e, 'range_scaled'), e.range_scaled = false; end
end
