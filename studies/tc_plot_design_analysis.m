function A = tc_plot_design_analysis()
% TC_PLOT_DESIGN_ANALYSIS  Analytical design pages: G(s), Routh limit, root locus, Bode, PM/GM, steady-state
% errors (final value theorem), step response.  Writes plots/design_analysis.{pdf,png},
% plots/root_locus.{pdf,png} and outputs/analysis.txt.  No control toolbox: plain polynomials.
%
% Loop:  L(s) = C(s) G_p(s),   G_p = gn / ( s (a2 s^2 + a1 s + a0) ),
%        C(s) = Kp (Ti s + 1) / ( Ti s (tau_f s + 1) )
  here = fileparts(mfilename('fullpath'));  addpath(fileparts(here));  tc_setup();
  K = tc_params();  rd = fullfile(fileparts(here), 'outputs');  if ~exist(rd, 'dir'), mkdir(rd); end
  fid = fopen(fullfile(rd, 'analysis.txt'), 'w');
  say = @(varargin) fprintf(fid, varargin{:});

  % ---- polynomials ------------------------------------------------------------------
  a = K.den;                                       % a2 a1 a0  (L J, L b + R J, R b + N^2 Kt Ke)
  Gn = K.gn;  Gd = conv([1 0], a);                 % G_p = Gn / Gd
  Cn = K.kp*[K.ti 1];  Cd = conv([K.ti 0], [K.tau_f 1]);
  Ln = Gn*Cn;  Ld = conv(Gd, Cd);                  % L = Ln / Ld
  cl = polyadd(Ld, Ln);                            % closed-loop characteristic polynomial

  say('Plant: G_p(s) = %.0f / ( s (%.4g s^2 + %.4g s + %.6g) )\n', Gn, a);
  say('  open-loop plant poles: %s\n', mat2str(roots(Gd), 5));
  say('Compensator: C(s) = Kp (Ti s + 1) / (Ti s (tau_f s + 1)), Kp = %g V/rad, Ti = %g s, tau_f = %g s\n', K.kp, K.ti, K.tau_f);
  say('Closed-loop poles of the feedback loop: %s\n', mat2str(sort(roots(cl)), 5));
  say('  all in the open left half plane: %d\n', all(real(roots(cl)) < 0));

  % ---- Routh gain limit ---------------------------------------------------------------
  % Proportional-only loop: s (a2 s^2 + a1 s + a0) + k gn = 0  ->  a2 s^3 + a1 s^2 + a0 s + k gn
  kmax_p = a(2)*a(3)/(a(1)*Gn);
  say('Routh, proportional only: a2 s^3 + a1 s^2 + a0 s + k gn.  First-column condition a1 a0 > a2 k gn gives\n');
  say('  k < a1 a0 /(a2 gn) = %.0f V/rad   (design Kp = %g is %.1f %% of it)\n', kmax_p, K.kp, 100*K.kp/kmax_p);
  % Full compensator: scale k on the loop gain; bisection on the max real part of the roots
  lo = 1;  hi = 1e3;
  for it = 1:80
    mid = sqrt(lo*hi);  if max(real(roots(polyadd(Ld, mid*Ln)))) < 0, lo = mid; else, hi = mid; end
  end
  say('Full PI + roll-off loop: stable for gain scale 0 < k < %.2f (i.e. Kp < %.0f V/rad); the design sits at k = 1 (gain margin %.1f dB)\n', lo, lo*K.kp, 20*log10(lo));

  % ---- frequency response, margins ------------------------------------------------------
  w = logspace(-2, 3, 5000);  s = 1j*w;
  Lw = polyval(Ln, s)./polyval(Ld, s);  mag = abs(Lw);  ph = unwrap(angle(Lw))*180/pi;
  ic = find(mag(1:end-1) >= 1 & mag(2:end) < 1, 1);  wc = w(ic);  pm = 180 + ph(ic);
  ip = find(ph(1:end-1) > -180 & ph(2:end) <= -180, 1);  wp = w(ip);  gm = -20*log10(mag(ip));
  Sw = 1./(1 + Lw);  Tw = Lw.*Sw;  Ms = max(abs(Sw));
  ib = find(abs(Tw) < 1/sqrt(2) & w > wc/3, 1);  wb = w(ib);
  say('Frequency domain: crossover %.2f rad/s, PM %.1f deg, phase-crossover %.1f rad/s, GM %.1f dB, Ms = %.2f (%.1f dB), closed-loop bandwidth %.2f rad/s\n', wc, pm, wp, gm, Ms, 20*log10(Ms), wb);
  say('  crossover is %.1fx below the plant resonance (%.1f rad/s): gain-stabilised, no notch\n', abs(roots([a(1) a(2) a(3)])(1))/wc, abs(roots([a(1) a(2) a(3)])(1)));

  % ---- steady-state errors (final value theorem) -----------------------------------------
  % L has two poles at the origin (plant + PI): system type 2.
  Ka = K.kp*Gn/(K.ti*a(3));                        % lim s^2 L(s)
  say('Steady state (FVT, E = R/(1+L)): type 2 loop; Kp_pos = inf, Kv = inf, Ka = lim s^2 L = %.3f 1/s^2\n', Ka);
  say('  step e_ss = 0, ramp e_ss = 0, parabola (accel a_th rad/s^2) e_ss = a_th/Ka = %.4f x a_th rad\n', 1/Ka);
  say('  worst LOS acceleration 0.060 rad/s^2 -> feedback-only error %.4f rad = %.1f m at 1.5 km (the feed-forward removes it for an exact model)\n', 0.06/Ka, 0.06/Ka*1500);
  % numerical check of the FVT: simulate E = S R for a ramp and a parabola
  Sn = Ld;  Sd = cl;                                     % S = Ld / cl
  [tt, e_ramp] = lin_sim(Sn, Sd, 'ramp', 0.1, 0:0.01:60);
  [~,  e_par ] = lin_sim(Sn, Sd, 'par',  0.06, 0:0.01:60);
  say('  simulated: ramp 0.1 rad/s -> e(60 s) = %.2e rad;  parabola 0.06 rad/s^2 -> e(60 s) = %.4f rad (FVT predicts %.4f)\n', e_ramp(end), e_par(end), 0.06/Ka);
  [~, ystep] = lin_sim(Ln, cl, 'step', 1, 0:0.002:6);
  ov = 100*(max(ystep) - 1);  i10 = find(ystep >= 0.1, 1);  i90 = find(ystep >= 0.9, 1);
  i2 = find(abs(ystep - 1) > 0.02, 1, 'last');
  say('Step of the feedback loop alone T = L/(1+L): overshoot %.1f %%, 10-90 %% rise %.2f s, 2 %% settling %.2f s\n', ov, (i90 - i10)*0.002, i2*0.002);
  say('  (the full 2-DOF controller tracks y_d through the exact inverse, so the reference response is set by the generator poles -%g rad/s, not by this loop.)\n', K.p_dev);
  fclose(fid);  type(fullfile(rd, 'analysis.txt'));

  % ---- figure 1: 2x2 design page ----------------------------------------------------------
  h = figure('visible', 'off', 'position', [50 50 1100 800], 'color', 'w');
  subplot(2,2,1);  pz = roots(Ld);  cp = roots(cl);  zz = roots(Ln);
  plot(real(pz), imag(pz), 'bx', 'markersize', 9, 'linewidth', 1.5);  hold on;
  plot(real(zz), imag(zz), 'bo', 'markersize', 8, 'linewidth', 1.5);
  plot(real(cp), imag(cp), 'rs', 'markersize', 8, 'linewidth', 1.5);  plot([0 0], [-70 70], 'k:');  hold off;
  grid on;  xlabel('Re [rad/s]');  ylabel('Im [rad/s]');  title('Pole-zero map of L(s) (x poles, o zero) and closed-loop poles (squares)');
  legend({'open-loop poles', 'open-loop zero', 'closed-loop poles'}, 'location', 'west');  axis([-40 5 -70 70]);
  subplot(2,2,2);
  semilogx(w, 20*log10(mag), 'b', 'linewidth', 1.6);  hold on;  semilogx(w, zeros(size(w)), 'k:');
  semilogx(wc*[1 1], [-60 60], 'r--');  semilogx(wp*[1 1], [-60 60], 'm--');  hold off;
  grid on;  ylabel('|L| [dB]');  title(sprintf('Bode of L: wc = %.2f rad/s, PM = %.1f deg, GM = %.1f dB', wc, pm, gm));  axis([1e-2 1e3 -60 80]);
  subplot(2,2,4);
  semilogx(w, ph, 'b', 'linewidth', 1.6);  hold on;  semilogx(w, -180*ones(size(w)), 'k:');
  semilogx(wc*[1 1], [-270 -90], 'r--');  semilogx(wp*[1 1], [-270 -90], 'm--');  hold off;
  grid on;  xlabel('\omega [rad/s]');  ylabel('phase [deg]');  axis([1e-2 1e3 -270 -90]);
  subplot(2,2,3);
  plot(0:0.002:6, ystep, 'b', 'linewidth', 1.6);  hold on;  plot([0 6], [1 1], 'k:');  hold off;
  grid on;  xlabel('t [s]');  ylabel('\theta / \theta_{ref}');  title('Step response of the feedback loop T = L/(1+L)');
  tc_save_figure(h, 'design_analysis');

  % ---- figure 2: root locus ------------------------------------------------------------------
  ks = [logspace(-3, 0, 120), logspace(0, log10(lo*1.6), 120)];  R = [];
  for k = ks, R = [R; sort(roots(polyadd(Ld, k*Ln)))']; end %#ok<AGROW>
  h = figure('visible', 'off', 'position', [50 50 900 620], 'color', 'w');
  plot(real(R), imag(R), 'b.', 'markersize', 4);  hold on;
  plot(real(pz), imag(pz), 'kx', 'markersize', 10, 'linewidth', 1.6);  plot(real(zz), imag(zz), 'ko', 'markersize', 9, 'linewidth', 1.6);
  plot(real(cp), imag(cp), 'rs', 'markersize', 9, 'linewidth', 1.6);
  rc = roots(polyadd(Ld, lo*Ln));  plot(real(rc), imag(rc), 'm^', 'markersize', 9, 'linewidth', 1.6);  hold off;
  grid on;  xlabel('Re [rad/s]');  ylabel('Im [rad/s]');  axis([-40 25 -90 90]);
  title(sprintf('Root locus in the loop gain k (k = 1 is the design; instability at k = %.2f)', lo));
  legend({'locus', 'open-loop poles', 'zero', 'design poles (k=1)', 'imaginary-axis crossing'}, 'location', 'northwest');
  tc_save_figure(h, 'root_locus');
  A = struct('wc', wc, 'pm', pm, 'gm', gm, 'Ms', Ms, 'kmax', lo, 'kmax_p', kmax_p, 'Ka', Ka);
end

function p = polyadd(a, b)
  n = max(numel(a), numel(b));  p = [zeros(1, n - numel(a)), a] + [zeros(1, n - numel(b)), b];
end

function [t, y] = lin_sim(num, den, kind, amp, t)
% Exact time response of num/den to amp * {step, ramp, parabola/2} by partial fractions of
% num*amp / (den * s^(m+1)) (a companion-form state space is hopelessly ill-conditioned here:
% coefficients span ten decades).  Repeated poles are handled through residue's multiplicities.
  m = find(strcmp(kind, {'step', 'ramp', 'par'})) - 1;
  dtot = conv(den, [1 zeros(1, m + 1)]);
  [r, p, kd, ee] = residue(amp*num, dtot);
  y = zeros(size(t));
  for q = 1:numel(r), y = y + real(r(q) * t.^(ee(q) - 1) / factorial(ee(q) - 1) .* exp(p(q)*t)); end
  if ~isempty(kd), y = y + kd(end)*(t == 0); end
end
