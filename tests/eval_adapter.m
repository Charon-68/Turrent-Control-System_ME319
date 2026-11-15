function o = eval_adapter(scenario, f_ctrl, theta0, opts)
% EVAL_ADAPTER  Stand-in for an evaluation script: calls the three handout functions only.
%
%   o = eval_adapter('B', 100)              scenario B, 100 Hz calls, turret starts at 0 rad
%   o = eval_adapter('C', 100, 0.3)         ... starting at theta = 0.3 rad
%   o = eval_adapter('MC5000', 20, 0.3, struct('conv', 'held'))
%
% Protocol (docs/ASSUMPTIONS.md, item 7): at its own step T = 1/f_ctrl the script
%   1. asks target_motion_generator(t, scenario) for [r_meas, theta_meas, meas_valid];
%   2. calls turret_controller(t, r_meas, theta_meas, meas_valid, theta, theta_dot);
%   3. judges a pulse on a 0 -> 1 edge of fire_cmd at least 1 s after the previous pulse:
%      the beam is a ray along the turret angle, hit if the distance from the target to that
%      ray is <= 2 m (distance to the turret if the target is behind the turret);
%   4. holds V_m constant over the step and integrates turret_plant itself (RK4, sub-steps
%      <= 1 ms), with the mechanical stops -10 / 85 deg.
% Only the 4th output of target_motion_generator (truth) is used, and only for judging.
%
% opts.conv   'new' (default) meas_valid is whatever the generator returns;
%             'held' meas_valid = 1 on every call, value = last radar sample (stale in blackouts)
% opts.t_end  default 10 s
%
% Output o: first_hit (s, NaN if none), n_pulses, n_miss, rms_point (m, cross-range error at call
% instants for t >= 2 s), max_point (m, same, t >= 2 s), pulses [t miss_m hit], t, err (m), theta.
  here = fileparts(mfilename('fullpath'));  root = fileparts(here);  addpath(root);
  if nargin < 3 || isempty(theta0), theta0 = 0; end
  if nargin < 4, opts = struct(); end
  if ~isfield(opts, 'conv'),  opts.conv = 'new'; end
  if ~isfield(opts, 't_end'), opts.t_end = 10; end
  clear turret_controller turret_plant target_motion_generator

  T = 1/f_ctrl;  n = round(opts.t_end*f_ctrl);  nsub = ceil(T/1e-3 - 1e-9);  h = T/nsub;
  lo = -10*pi/180;  hi = 85*pi/180;
  s = [theta0; 0; 0];                                   % [theta; theta_dot; i]
  last = [NaN NaN];  t_last = -Inf;  fprev = 0;
  pulses = zeros(0, 3);  err = zeros(1, n + 1);  tt = zeros(1, n + 1);  th_log = zeros(1, n + 1);
  for k = 0:n
    t = k*T;
    [r, th, valid, ~, tru] = gen(t, scenario);
    if valid > 0.5, last = [r th]; end
    if strcmp(opts.conv, 'held')
      r = last(1);  th = last(2);  valid = double(isfinite(r));
    end
    [V, fire] = turret_controller(t, r, th, valid, s(1), s(2));
    x = tru(1);  y = tru(2);
    if fire > 0.5 && fprev <= 0.5 && (t - t_last) >= 1 - 1e-9
      if x*cos(s(1)) + y*sin(s(1)) >= 0, miss = abs(x*sin(s(1)) - y*cos(s(1))); else, miss = hypot(x, y); end
      pulses(end+1, :) = [t, miss, miss <= 2]; %#ok<AGROW>
      t_last = t;
    end
    fprev = fire;
    err(k+1) = hypot(x, y)*sin(atan2(y, x) - s(1));  tt(k+1) = t;  th_log(k+1) = s(1);
    if k < n
      for q = 1:nsub
        k1 = rate(t, V, s);
        k2 = rate(t, V, s + 0.5*h*k1);
        k3 = rate(t, V, s + 0.5*h*k2);
        k4 = rate(t, V, s + h*k3);
        s = s + (h/6)*(k1 + 2*k2 + 2*k3 + k4);
        if s(1) >= hi && s(2) > 0, s(1) = hi; s(2) = 0; end
        if s(1) <= lo && s(2) < 0, s(1) = lo; s(2) = 0; end
      end
      if ~all(isfinite(s)), error('eval_adapter: plant diverged'); end
    end
  end
  o.pulses = pulses;  o.n_pulses = size(pulses, 1);  o.n_miss = sum(pulses(:,3) == 0);
  o.first_hit = NaN;
  if any(pulses(:,3) > 0.5), o.first_hit = pulses(find(pulses(:,3) > 0.5, 1), 1); end
  w = tt >= 2;  o.rms_point = sqrt(mean(err(w).^2));  o.max_point = max(abs(err(w)));
  o.t = tt;  o.err = err;  o.theta = th_log;
end

function [r, th, valid, dummy, tru] = gen(t, scenario)
  [r, th, valid, tru] = target_motion_generator(t, scenario);
  dummy = [];
end

function ds = rate(t, V, s)
  [w, a, idot] = turret_plant(t, V, s(1), s(2), s(3));
  ds = [w; a; idot];
end
