function x = tc_envelope_clip(x, K)
% TC_ENVELOPE_CLIP  Project the estimate onto the spec's physical envelope: the
% assignment guarantees |v| <= 150 m/s and |a| <= 20 m/s^2 (and smooth motion),
% so a Gaussian estimate may be scaled back to 5 % / 15 % outside those bounds.
  lim = [K.env_v, K.env_a, K.env_j];
  for q = 1:3
    ii = [1 5] + q;                        % rows of vel / acc / jerk for both axes
    m = hypot(x(ii(1)), x(ii(2)));
    if m > lim(q), x(ii) = x(ii)*(lim(q)/m); end
  end
end
