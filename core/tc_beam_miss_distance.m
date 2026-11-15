function [miss, signed] = tc_beam_miss_distance(th_beam, x, y)
% TC_BEAM_MISS_DISTANCE  Distance from the target to the beam ray leaving the origin at angle
% th_beam, plus the signed cross-range offset (positive = target counter-clockwise of the beam).
% A target BEHIND the turret (negative projection on the ray) is |p| away, not the line distance.
  signed = y.*cos(th_beam) - x.*sin(th_beam);
  ahead = (x.*cos(th_beam) + y.*sin(th_beam)) >= 0;
  miss = abs(signed);
  miss(~ahead) = hypot(x(~ahead), y(~ahead));
end
