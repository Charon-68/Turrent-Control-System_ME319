function o = tc_independent_run(c, f_ctrl, conv)
% TC_INDEPENDENT_RUN  One closed-loop engagement of an tc_independent_generator case, evaluation-script style.
%   o = tc_independent_run(c, 100, 'new')      conv 'new'  : meas_valid = 1 only on a NEW radar sample,
%                                                    NaN between ticks and in blackouts
%                                     conv 'held' : meas_valid = 1 on EVERY call; the value is the
%                                                    last radar sample (stale through blackouts)
% Uses only turret_controller and turret_plant (RK4, <= 2 ms steps, zero-order-hold voltage) and its
% OWN judging: the beam is a ray along the turret angle, distance to the ray (|p| if the target is
% behind), hit <= 2 m, pulse on a 0 -> 1 edge of fire_cmd at least 1 s after the previous pulse.
% Mechanical stops -10 / 85 deg.  Pointing RMS = cross-range error r sin(theta_true - theta_turret)
% over t >= 2 s at the call instants.
  clear turret_controller
  T = 1/f_ctrl;  n = round(10*f_ctrl);  nsub = ceil(T/2e-3 - 1e-9);  h = T/nsub;
  lo = -10*pi/180;  hi = 85*pi/180;
  s = [c.theta0; 0; 0];
  last = [NaN NaN];  t_last = -Inf;  fprev = 0;
  pulses = zeros(0, 3);  err = zeros(1, n + 1);  tt = zeros(1, n + 1);
  for k = 0:n
    t = k*T;
    pos = t/1e-3;  i0 = min(floor(pos + 1e-9), numel(c.tg) - 2);  fr = pos - i0;
    tru = c.truth(:, i0 + 1)*(1 - fr) + c.truth(:, i0 + 2)*fr;
    % ---- radar -------------------------------------------------------------------------------
    r = NaN;  th = NaN;  fresh = 0;
    kt = round(t/0.05);
    if abs(t - kt*0.05) < 1e-9
      inbo = any(t >= c.bo(:,1) - 1e-9 & t < c.bo(:,1) + c.bo(:,2) - 1e-9);
      if ~inbo
        xy = c.truth(1:2, round(kt*0.05/1e-3) + 1);
        r = hypot(xy(1), xy(2)) + c.nz(kt + 1, 1);  th = atan2(xy(2), xy(1)) + c.nz(kt + 1, 2);
        fresh = 1;  last = [r th];
      end
    end
    if strcmp(conv, 'new')
      valid = fresh;
    else
      valid = 1;  r = last(1);  th = last(2);        % held flag: always "valid", value possibly stale
      if ~isfinite(r), valid = 0; end
    end
    [V, fire] = turret_controller(t, r, th, valid, s(1), s(2));
    % ---- judging ----------------------------------------------------------------------------
    x = tru(1);  y = tru(2);
    if fire > 0.5 && fprev <= 0.5 && (t - t_last) >= 1 - 1e-9
      if x*cos(s(1)) + y*sin(s(1)) >= 0, miss = abs(x*sin(s(1)) - y*cos(s(1))); else, miss = hypot(x, y); end
      pulses(end+1, :) = [t, miss, miss <= 2]; %#ok<AGROW>
      t_last = t;
    end
    fprev = fire;
    err(k + 1) = hypot(x, y)*sin(atan2(y, x) - s(1));  tt(k + 1) = t;
    % ---- plant: RK4 on [theta; omega; i] through turret_plant -----------------------------------
    if k < n
      for q = 1:nsub
        k1 = plant_rate(t, V, s);
        k2 = plant_rate(t, V, s + 0.5*h*k1);
        k3 = plant_rate(t, V, s + 0.5*h*k2);
        k4 = plant_rate(t, V, s + h*k3);
        s = s + (h/6)*(k1 + 2*k2 + 2*k3 + k4);
        if s(1) >= hi && s(2) > 0, s(1) = hi; s(2) = 0; end
        if s(1) <= lo && s(2) < 0, s(1) = lo; s(2) = 0; end
      end
      if ~all(isfinite(s)), error('tc_independent_run: plant diverged'); end
    end
  end
  o.n_pulses = size(pulses, 1);  o.n_miss = sum(pulses(:,3) == 0);
  o.success = any(pulses(:,3) > 0.5);
  o.t_first = NaN;  if o.success, o.t_first = pulses(find(pulses(:,3) > 0.5, 1), 1); end
  w = tt >= 2;  o.rms = sqrt(mean(err(w).^2));  o.pulses = pulses;
end

function ds = plant_rate(t, V, s)
  [w, a, idot] = turret_plant(t, V, s(1), s(2), s(3));
  ds = [w; a; idot];
end
