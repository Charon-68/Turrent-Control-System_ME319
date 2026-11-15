function D = tc_stops_governor(D, K)
% TC_STOPS_GOVERNOR  Reference governor for the elevation stops.  Past the guard
% band (stops -/+ lim_margin) the whole derivative tuple is frozen, not only the
% angle, so the feed-forward never commands motion the stop would refuse.
  lo = K.th_min + K.lim_margin;  hi = K.th_max - K.lim_margin;
  if D(1) > hi
    D = [hi; 0; 0; 0];
  elseif D(1) < lo
    D = [lo; 0; 0; 0];
  end
end
