function sp = tc_step_specs(t, y)
% TC_STEP_SPECS  Rise time (10-90 %), peak time, overshoot (%) and 2 % settling time of a sampled unit-step response.
%   The final value is the last sample (the caller supplies a long enough window).  A monotonic response has
%   no peak time (NaN) and zero overshoot.  Returns NaNs if the response never reaches 90 % of its final value.
  yf = y(end);  [pk, ip] = max(y);
  i10 = find(y >= 0.1*yf, 1);  i90 = find(y >= 0.9*yf, 1);
  if isempty(i10) || isempty(i90) || i10 < 2 || i90 < 2, sp = struct('tr', NaN, 'tp', NaN, 'os', NaN, 'ts', NaN);  return; end
  t10 = interp1(y(i10-1:i10), t(i10-1:i10), 0.1*yf);  t90 = interp1(y(i90-1:i90), t(i90-1:i90), 0.9*yf);
  sp.tr = t90 - t10;
  sp.tp = t(ip);  if ip == numel(t), sp.tp = NaN; end
  sp.os = max(0, 100*(pk - yf)/yf);
  io = find(abs(y - yf) > 0.02*abs(yf), 1, 'last');  if isempty(io), io = 1; end
  sp.ts = t(io);
end
