function m = tc_loop_margins(Ln, Ld, Td)
% TC_LOOP_MARGINS  Crossover, phase margin, gain margin of L = Ln/Ld (times exp(-s Td)), from the polynomials.
%   m.wc, m.pm (deg), m.wp (phase crossover, rad/s), m.gm (dB, smallest over ALL -180 deg crossings), m.pm_all / m.wc_all (every downward
%   crossing: the worst one is the safe number when a resonance lifts |L| above 1 again), m.w, m.L, m.ph.
%   Crossings are linearly interpolated in log frequency on a fine grid.
  if nargin < 3, Td = 0; end
  w = logspace(-3, 3.5, 60000);  s = 1j*w;
  L = polyval(Ln, s)./polyval(Ld, s) .* exp(-s*Td);
  mag = abs(L);  ph = unwrap(angle(L))*180/pi;
  if ph(1) > 0, ph = ph - 360; end
  lw = log(w);  lm = log(mag);
  ic = find(mag(1:end-1) >= 1 & mag(2:end) < 1);
  wcs = zeros(size(ic));  pms = zeros(size(ic));
  for q = 1:numel(ic)
    i = ic(q);  fr = lm(i)/(lm(i) - lm(i+1));
    wcs(q) = exp(lw(i) + fr*(lw(i+1) - lw(i)));  pms(q) = 180 + ph(i) + fr*(ph(i+1) - ph(i));
  end
  m.wc_all = wcs;  m.pm_all = pms;
  if isempty(wcs), m.wc = NaN;  m.pm = NaN; else, m.wc = wcs(1);  m.pm = pms(1); end
  % gain margin: every crossing of -180 deg (mod 360), either direction; gm = the smallest -20 log|L| (negative = unstable)
  gms = [];  wps = [];
  for lev = -180 + 360*(-3:1)
    ip = find((ph(1:end-1) - lev).*(ph(2:end) - lev) < 0);
    for q = 1:numel(ip)
      i = ip(q);  fr = (lev - ph(i))/(ph(i+1) - ph(i));
      wps(end+1) = exp(lw(i) + fr*(lw(i+1) - lw(i)));  gms(end+1) = -20*log10(exp(lm(i) + fr*(lm(i+1) - lm(i)))); %#ok<AGROW>
    end
  end
  if isempty(gms), m.wp = Inf;  m.gm = Inf;  else, [m.gm, k] = min(gms);  m.wp = wps(k); end
  m.gm_all = gms;  m.wp_all = wps;
  m.w = w;  m.L = L;  m.ph = ph;
end
