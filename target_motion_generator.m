function [r_meas, theta_meas, meas_valid, truth, info] = target_motion_generator(t, scenario)
% TARGET_MOTION_GENERATOR  Radar measurements of the scripted aerial target.
%
%   [r_meas, theta_meas, meas_valid] = target_motion_generator(t, scenario)
%
% A new (r, theta) sample exists only at the radar ticks t = 0.05 k outside blackouts;
% elsewhere r_meas = theta_meas = NaN and meas_valid = 0.  Noise is zero-mean white
% Gaussian (sigma_r = 1 m, sigma_theta = 0.02 deg) indexed by tick, so the result does not
% depend on how often you call.  scenario: 'A' 'B' 'C' (or 1 2 3), 'D1'..'D6', 'MC<seed>',
% 'HB<seed>', optional '#k' noise draw and '+glint', '+range', '+bias' stress flags
% (see core/tc_scenario_build.m).  Optional 4th/5th outputs (not part of the evaluation
% interface): truth = [x y vx vy ax ay]' at t and info (blackouts, geometry).
  persistent cache
  tc_setup();
  if isempty(cache), cache = struct('key', {{}}, 'val', {{}}); end
  if isnumeric(scenario), names = {'A', 'B', 'C'};  scenario = names{scenario}; end
  scenario = char(scenario);
  hit = find(strcmp(cache.key, scenario), 1);
  if isempty(hit)
    e = tc_scenario_build(scenario);
    cache.key{end+1} = scenario;  cache.val{end+1} = e;
    if numel(cache.key) > 8, cache.key(1) = [];  cache.val(1) = []; end   % bound memory over MC runs
  else
    e = cache.val{hit};
  end
  K = tc_params();
  r_meas = NaN;  theta_meas = NaN;  meas_valid = 0;
  if nargout > 3, truth = interp1(e.tg, e.truth', min(max(t, 0), e.t_end), 'linear')'; end
  if nargout > 4, info = struct('blackouts', e.bo, 'name', e.name); end
  k = round(t/K.dt_radar);
  if abs(t - k*K.dt_radar) > 1e-6 || k < 0 || k >= size(e.nz, 1), return; end
  for j = 1:size(e.bo, 1)
    if t >= e.bo(j,1) - 1e-9 && t < e.bo(j,1) + e.bo(j,2) - 1e-9, return; end
  end
  s = interp1(e.tg, e.truth(1:2,:)', k*K.dt_radar, 'linear');
  r0 = hypot(s(1), s(2));  th0 = atan2(s(2), s(1));
  nz = e.nz(k+1, :);
  if e.range_scaled, nz = nz*(r0/1500); end
  r_meas = r0 + nz(1) + e.bias(1);  theta_meas = th0 + nz(2) + e.bias(2);  meas_valid = 1;
end
