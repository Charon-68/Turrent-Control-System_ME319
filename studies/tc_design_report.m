function R = tc_design_report()
% TC_DESIGN_REPORT  Analytical design checks printed as one text block (no simulation).
%   plant poles, Euler stability limit, loop margins (continuous and exact ZOH C(z) at
%   20/50/100 Hz), Taylor-section stability, reference polynomial K(s).
  here = fileparts(mfilename('fullpath'));  addpath(fileparts(here));  tc_setup();
  K = tc_params();
  p = roots(K.den);
  fprintf('Plant  theta/V = %.0f / ( s (%.4g s^2 + %.4g s + %.5g) )\n', K.gn, K.den);
  fprintf('  poles: 0, %.2f%+.2fj  (wn %.1f rad/s = %.2f Hz, zeta %.3f)\n', real(p(1)), imag(p(1)), abs(p(1)), abs(p(1))/2/pi, -real(p(1))/abs(p(1)));
  hmax = min(-2*real(p)./abs(p).^2);
  fprintf('  forward-Euler step limit %.2f ms (integrate at <= 1 ms or use RK4/exact)\n', 1e3*hmax);
  fprintf('  feed-forward V = %.4g d3y/dt3 + %.4g d2y/dt2 + %.5g dy/dt   (DEN/NUM)\n', K.ff);
  fprintf('  cruise rate at +-%g V: %.4f rad/s (%.1f deg/s)\n', K.V_lim, K.omega_cruise, K.omega_cruise*180/pi);
  % loop margins
  [wc, pm, gm] = margins(K, 0);
  fprintf('Loop (continuous C): wc %.2f rad/s, PM %.1f deg, GM %.1f dB\n', wc, pm, gm);
  R.margins = [wc pm gm];
  for f = [100 50 20]
    [wc, pm, gm] = margins(K, f);
    fprintf('Loop (C(z) + ZOH plant, %3d Hz): wc %.2f rad/s, PM %.1f deg, GM %.1f dB\n', f, wc, pm, gm);
    R.margins(end+1,:) = [wc pm gm];
  end
  % reference polynomial
  tp = K.tp;  p0 = K.p_dev;  c3 = tp^3/6;
  Ks = [c3*3*p0 - tp^2/2, 3*c3*p0^2 - tp, c3*p0^3 - 1];          % K(s) = k2 s^2 + k1 s + k0
  fprintf('Reference: (A3+K)(s) = (tp^3/6)(s+%g)^3,  K(s) = %.4g s^2 + %.4g s + %.5g\n', p0, Ks);
  for n = 1:5
    c = tp.^(0:n)./factorial(0:n);  rt = roots(fliplr(c));
    z = min(-real(rt)./abs(rt));
    fprintf('  Taylor section n=%d: %s, least damping %.2f\n', n, ifelse(all(real(rt) < 0), 'Hurwitz', 'UNSTABLE'), z);
  end
  fprintf('Error budget: R_eng = %.2f mrad @2 km, %.2f mrad @800 m; drift a t_p^2/2 = %.2f m; beam t_p ahead misses r*thd*tp ~ %.0f m\n', ...
          1e3*K.r_eng/2000, 1e3*K.r_eng/800, K.a_tgt_max*tp^2/2, 1500*0.05*tp);
end

function [wc, pm, gm] = margins(K, f)
  w = logspace(-1, log10(1500), 6000);  s = 1j*w;
  Gp = K.gn ./ (s .* (K.den(1)*s.^2 + K.den(2)*s + K.den(3)));
  C = K.kp*(1 + 1./(K.ti*s))./(K.tau_f*s + 1);
  if f > 0
    % zero-order hold of the sampled controller output: gain (1-e^{-sT})/(sT), i.e. ~ T/2 of
    % extra delay.  Conservative continuous-time equivalent of the exact C(z) + ZOH loop
    % (the two agree below ~Nyquist/5, where the crossover sits).
    T = 1/f;  C = C .* (1 - exp(-s*T))./(s*T);
  end
  Lg = Gp.*C;  mag = abs(Lg);  ph = unwrap(angle(Lg))*180/pi;
  i = find(mag(1:end-1) >= 1 & mag(2:end) < 1, 1);  wc = w(i);  pm = 180 + ph(i);
  j = find(ph(1:end-1) > -180 & ph(2:end) <= -180, 1);  gm = -20*log10(mag(j));
end

function s = ifelse(c, a, b), if c, s = a; else, s = b; end, end
