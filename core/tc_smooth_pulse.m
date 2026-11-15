function w = tc_smooth_pulse(t, t_on, t_off, ramp)
% TC_SMOOTH_PULSE  Smooth pulse: quintic rise starting at t_on, quintic fall starting at t_off.
  w = tc_smoothstep((t - t_on)/ramp) - tc_smoothstep((t - t_off)/ramp);
end
