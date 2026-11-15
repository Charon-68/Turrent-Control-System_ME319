function d = tc_scenario_catalog(name)
% TC_SCENARIO_CATALOG  Hand-written scenarios.  The assignment gives no initial conditions or
% maneuver shapes, so these are OUR choices (stated as such in docs/DESIGN.md).
%   d.p0, d.v0  initial position / velocity [m, m/s];  d.acc(t) -> 2 x numel(t) [m/s^2]
%   d.bo        blackouts, rows [onset duration] [s]
  S = @tc_smoothstep;  W = @tc_smooth_pulse;
  bo_none = zeros(0, 2);
  switch upper(name)
    case 'A'
      d.p0 = [1700; 700];  d.v0 = [-120; 0];
      d.acc = @(t) zeros(2, numel(t));  d.bo = bo_none;
    case 'B'
      d.p0 = [1700; 700];  d.v0 = [-120; 0];
      d.acc = @(t) maneuver_b(t);  d.bo = bo_none;
    case 'C'
      d = tc_scenario_catalog('A');  d.bo = [1.0 0.5];
    case 'D1'                                        % stealth + break-away during the loss
      d.p0 = [1900; 500];  d.v0 = [-125; 0];
      d.acc = @(t) [9; 17] * W(t(:)', 3.1, 4.4, 0.3);  d.bo = [3.0 0.5];
    case 'D2'                                        % weave
      d.p0 = [1800; 800];  d.v0 = [-110; 0];
      d.acc = @(t) weave(t);  d.bo = bo_none;
    case 'D3'                                        % close pass, elevation 30 -> 83 deg
      d.p0 = [1450; 835];  d.v0 = [-135; 0];
      d.acc = @(t) [zeros(1, numel(t)); S((t(:)'-1)/0.5) .* 4.*sin(2*pi*0.4*(t(:)'-1))];
      d.bo = bo_none;
    case 'D4'                                        % repeated loss
      d = tc_scenario_catalog('B');  d.bo = [1.0 0.5; 3.2 0.4; 5.5 0.5; 7.5 0.4];
    case 'D5'                                        % weave + loss, close start
      d.p0 = [1300; 700];  d.v0 = [-80; 0];
      d.acc = @(t) weave(t);  d.bo = [4.0 0.5];
    case 'D6'                                        % +-18 m/s^2 reversals, jerk ~ 450 m/s^3
      d.p0 = [1750; 850];  d.v0 = [-125; 0];
      d.acc = @(t) [zeros(1, numel(t)); 18*square_smooth(t(:)', 1.2, 0.15)];
      d.bo = bo_none;
    otherwise
      error('tc_scenario_catalog: unknown scenario %s', name);
  end
end

function a = maneuver_b(t)
  t = t(:)';  g = tc_smoothstep((t - 1)/0.5);
  a = [8*sin(2*pi*0.15*(t - 1)); 14*sin(2*pi*0.30*(t - 1))] .* [g; g];
end

function a = weave(t)
  t = t(:)';  g = tc_smoothstep((t - 0.5)/0.5);
  a = [13*sin(2*pi*t/1.6); 14*cos(2*pi*t/1.2)] .* [g; g];
end

function s = square_smooth(t, half, ramp)
% +1 for t < half, then alternating -1/+1 every `half` seconds, each switch a quintic
% transition of length `ramp` centred on the switch time; ramps up from 0 in the first ramp.
  s = tc_smoothstep(t/ramp);
  for k = 1:ceil(max(t)/half)
    s = s - 2*(-1)^(k+1) * tc_smoothstep((t - k*half + ramp/2)/ramp);
  end
end
