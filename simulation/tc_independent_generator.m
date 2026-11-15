function c = tc_independent_generator(family, seed)
% TC_INDEPENDENT_GENERATOR  Independent maneuver generator for the elevation plane (written from the
% assignment text only; shares no code with the project's own scenario builder).
%
%   c = tc_independent_generator(family, seed)      family 1..5, seed any integer
%
%   1 weave      sinusoidal lateral acceleration (perpendicular to the initial heading)
%   2 step       acceleration step, rate-limited by a jerk limit J (linear ramp, then hold)
%   3 bangbang   acceleration reversing every 1 s, each reversal rate-limited by a jerk limit
%   4 arc        circular arc at (almost) maximum |a| = 19.5 m/s^2, constant speed
%   5 popup      climb pulse followed by a dive pulse (smooth quintic windows)
%
% A draw is ACCEPTED only if over 0..10 s: 800 <= r <= 2000 m, |v| <= 150 m/s, |a| <= 20 m/s^2
% and the elevation stays inside [-8, 83] deg (a target outside the turret's travel is not an
% interception problem).  Rejected draws are redrawn deterministically (seed, attempt), so a
% (family, seed) pair always yields the same case.  Random: start bearing and range, speed,
% heading, maneuver parameters, ONE blackout (onset >= 1 s, duration <= 0.5 s) and the initial
% turret angle in [-10, 85] deg.
%
% Output c: tg (1 x N, 1 ms grid 0..10 s), truth (6 x N: x y vx vy ax ay), bo [onset dur],
% nz (201 x 2 radar noise per tick: [range m, angle rad]), theta0 (rad), attempts, name,
% jerk_max, and the realised extremes (r_min r_max v_max a_max el_min el_max).
  names = {'weave', 'step', 'bangbang', 'arc', 'popup'};
  tg = 0:1e-3:10;  dt = 1e-3;
  for attempt = 0:999
    u = draw(seed*1009 + attempt*131 + family*7, 40);
    % ---- start geometry -----------------------------------------------------------------
    r0 = 900 + 1050*u(1);                         % m
    el0 = (6 + 62*u(2))*pi/180;                   % start elevation
    spd = 40 + 100*u(3);                          % m/s
    if u(4) < 0.7, hd = el0 + pi + (2*u(5) - 1)*pi/3; else, hd = 2*pi*u(5); end
    p0 = r0*[cos(el0); sin(el0)];
    v0 = spd*[cos(hd); sin(hd)];
    nrm = [-sin(hd); cos(hd)];                    % lateral direction of the initial heading
    A = 8 + 11.5*u(6);                            % maneuver amplitude, <= 19.5
    switch family
      case 1
        f = 0.15 + 0.65*u(7);  ph = 2*pi*u(8);  ton = 0.3 + 1.7*u(9);
        ramp = min(max((tg - ton)/0.6, 0), 1);  ramp = ramp.^3.*(10 - 15*ramp + 6*ramp.^2);
        a = nrm*(A*sin(2*pi*f*tg + ph).*ramp);
      case 2
        J = 40 + 160*u(7);  ts = 1 + 4*u(8);  th = 2*pi*u(9);
        a = [cos(th); sin(th)]*(A*min(max((tg - ts)*J/A, 0), 1));
      case 3
        J = 100 + 200*u(7);  t0 = 0.2 + 0.8*u(8);  sg = 1 - 2*(u(9) < 0.5);  th = 2*pi*u(10);
        cmd = A*sg*(1 - 2*mod(floor((tg - t0)/1.0), 2));  cmd(tg < t0) = A*sg;
        am = zeros(size(tg));
        for k = 2:numel(tg)                       % rate-limited tracking of the command
          am(k) = am(k-1) + min(max(cmd(k) - am(k-1), -J*dt), J*dt);
        end
        a = [cos(th); sin(th)]*am;
      case 4
        J = 60 + 90*u(7);  ton = 0.2 + 1.3*u(8);  sg = 1 - 2*(u(9) < 0.5);  Aa = 19.5;
        mag = min(max((tg - ton)*J/Aa, 0), 1)*Aa;
        a = zeros(2, numel(tg));  vv = zeros(2, numel(tg));  vv(:,1) = v0;  phi = hd;
        for k = 1:numel(tg)
          a(:,k) = mag(k)*sg*[-sin(phi); cos(phi)];
          if k < numel(tg)
            phi = phi + sg*mag(k)/spd*dt;         % constant speed: heading rate = a/v
            vv(:,k+1) = spd*[cos(phi); sin(phi)];
          end
        end
      case 5
        ramp = 0.4;  A2 = 8 + 11.5*u(7);  T1 = 1 + 1.5*u(8);  t1 = 0.5 + 2*u(9);  gap = 0.5*u(10);
        up = win(tg, t1, t1 + T1, ramp);  dn = win(tg, t1 + T1 + gap, t1 + 2*T1 + gap, ramp);
        a = [zeros(1, numel(tg)); A*up - A2*dn];
        v0 = [spd*(1 - 2*(u(11) < 0.5)); -20 + 40*u(12)];            % horizontal run plus a small vertical rate
      otherwise
        error('tc_independent_generator: family must be 1..5');
    end
    % ---- integrate (trapezoid) -------------------------------------------------------------
    if family == 4
      v = vv;
    else
      v = v0 + cumtrapz(tg, a, 2);
    end
    p = p0 + cumtrapz(tg, v, 2);
    r = hypot(p(1,:), p(2,:));  el = atan2(p(2,:), p(1,:))*180/pi;
    spdv = hypot(v(1,:), v(2,:));  am_ = hypot(a(1,:), a(2,:));
    if min(r) >= 800 && max(r) <= 2000 && max(spdv) <= 150 && max(am_) <= 20 && min(el) >= -8 && max(el) <= 83
      c.tg = tg;  c.truth = [p; v; a];
      c.attempts = attempt + 1;  c.name = names{family};  c.family = family;  c.seed = seed;
      c.jerk_max = max(max(abs(diff(a, 1, 2))))/dt;
      c.extremes = [min(r) max(r) max(spdv) max(am_) min(el) max(el)];
      % ---- blackout and turret start ------------------------------------------------------
      ub = draw(seed*31 + family*977, 8);
      c.bo = [1 + 8*ub(1), 0.05 + 0.45*ub(2)];
      c.theta0 = (-10 + 95*ub(3))*pi/180;
      % ---- radar noise, one row per tick 0..200 (Box-Muller, own stream) ------------------------
      un = draw(seed*17 + family*5003, 4*201);
      z1 = sqrt(-2*log(max(un(1:402), eps))).*cos(2*pi*un(403:804));
      z = reshape(z1, 201, 2);
      c.nz = z .* [1, 0.02*pi/180];
      return;
    end
  end
  error('tc_independent_generator: no admissible draw (family %d, seed %d)', family, seed);
end

function u = draw(seed, n)
% reproducible uniforms without disturbing the caller's random stream
  st = rand('twister');  rand('twister', mod(double(seed), 2^32));  u = rand(n, 1);  rand('twister', st);
end

function w = win(t, t_on, t_off, ramp)
% smooth pulse: quintic rise from t_on, quintic fall from t_off
  s = @(x) min(max(x, 0), 1).^3 .* (10 - 15*min(max(x, 0), 1) + 6*min(max(x, 0), 1).^2);
  w = s((t - t_on)/ramp) - s((t - t_off)/ramp);
end
