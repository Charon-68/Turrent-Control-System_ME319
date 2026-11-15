function R = tc_study_bode_asymptotes()
% TC_STUDY_BODE_ASYMPTOTES  Hand-style asymptotic Bode plot of G(s) overlaid on the exact response.
%   G(s) = Kv / ( s (1 + 2 zeta s/wn + s^2/wn^2) ),  Kv = gn/a0,  wn = sqrt(a0/a2),  zeta = a1 / (2 sqrt(a0 a2)).
%   Magnitude asymptotes: -20 dB/dec (integrator) up to wn, -60 dB/dec beyond, resonant peak 1/(2 zeta) above the
%   corner.  Phase asymptote: -90 deg, then -270 deg, passing -180 deg at wn.
%   Writes outputs/tc_study_bode_asymptotes.txt, plots/tc_study_bode_asymptotes.{pdf,png}.
  here = fileparts(mfilename('fullpath'));  addpath(fileparts(here));  tc_setup();
  M = tc_course_model();  a = M.a;
  [say, done] = tc_course_report('tc_study_bode_asymptotes');
  Kv = M.gn/a(3);  wn = sqrt(a(3)/a(1));  zt = a(2)/(2*sqrt(a(1)*a(3)));
  say('G(s) = Kv / ( s (1 + 2 zeta s/wn + s^2/wn^2) ),  Kv = gn/a0 = %.4e rad/s per V, wn = %.3f rad/s, zeta = %.4f\n\n', Kv, wn, zt);
  say('Asymptotic construction\n');
  say('  %-22s %-14s %-26s %s\n', 'break / corner', 'frequency', 'slope before -> after', 'phase asymptote');
  say('  %-22s %-14s %-26s %s\n', 'integrator (start)', 'all w', '-20 dB/dec', '-90 deg');
  say('  %-22s %-14s %-26s %s\n', 'line crosses 0 dB at', sprintf('w = Kv = %.2e', Kv), '(-20 dB/dec line)', 'not a real crossover: |G| is above 0 dB below Kv');
  say('  %-22s %-14s %-26s %s\n', 'resonant pair', sprintf('%.2f rad/s', wn), '-20 -> -60 dB/dec (-40 added)', '-90 -> -270 deg (-180 added, sharp: zeta = 0.094)');
  say('  corner frequency %.3f rad/s = %.3f Hz;  period %.4f s\n', wn, wn/2/pi, 2*pi/wn);
  say('  resonant peak above the asymptote at wn: 1/(2 zeta) = %.3f = %.2f dB\n', 1/(2*zt), 20*log10(1/(2*zt)));
  say('  asymptote value at the corner: |G| = Kv/wn = %.3e = %.2f dB\n', Kv/wn, 20*log10(Kv/wn));
  w = logspace(-3, 3, 20000);  s = 1j*w;  G = polyval(M.Gn, s)./polyval(M.Gd, s);
  mag = abs(G);  ph = unwrap(angle(G))*180/pi;
  asy = Kv./w;  asy(w > wn) = Kv*wn^2./w(w > wn).^3;                      % |G| asymptote
  jr = find(w > wn/4 & w < 4*wn);  [pk, ipr] = max(mag(jr));  ip = jr(ipr);  [~, ic] = min(abs(w - wn));
  say('  exact |G| at wn: %.3e = %.2f dB (asymptote + 1/(2 zeta) = %.2f dB);  exact resonance maximum (searched in wn/4..4wn) %.3e = %.2f dB at %.3f rad/s\n', mag(ic), 20*log10(mag(ic)), 20*log10(Kv/wn/(2*zt)), pk, 20*log10(pk), w(ip));
  err = abs(20*log10(mag) - 20*log10(asy));
  lo = w <= wn/5;  hi = w >= 5*wn;
  say('  asymptote error away from the corner: max %.3f dB for w <= wn/5, max %.3f dB for w >= 5 wn; within a factor of 2 of wn (peak region) up to %.2f dB\n', max(err(lo)), max(err(hi)), max(err(w > wn/2 & w < 2*wn)));
  phase_asy = -90*ones(size(w));  phase_asy(w > wn) = -270;
  ph_at = ph(ic);
  say('  exact phase at wn: %.2f deg (asymptote jump passes -180 deg); at wn/5 %.2f deg, at 5 wn %.2f deg\n', ph_at, interp1(w, ph, wn/5), interp1(w, ph, 5*wn));
  jj = find(w > wn/4 & w < 4*wn);
  say('  phase transition: the exact phase passes -135 deg at %.1f rad/s and -225 deg at %.1f rad/s (a %.1f%% wide band around wn)\n', ...
      interp1(ph(jj), w(jj), -135), interp1(ph(jj), w(jj), -225), 100*(interp1(ph(jj), w(jj), -225) - interp1(ph(jj), w(jj), -135))/wn);
  say('\nConsequence for the loop: with the PI + roll-off compensator the loop crossover is at 4.10 rad/s, on the -20 dB/dec part, %.1fx below the corner, so the\n', wn/4.10);
  say('resonance sits on the -60 dB/dec skirt and is gain-stabilised (see tc_study_sensitivity / outputs/analysis.txt).\n');
  done();

  hf = figure('visible', 'off', 'position', [50 50 900 700], 'color', 'w');
  subplot(2,1,1);  semilogx(w, 20*log10(mag), 'b', 'linewidth', 1.8);  hold on;  semilogx(w, 20*log10(asy), 'r--', 'linewidth', 1.6);
  semilogx(wn*[1 1], [-200 40], 'k:');  plot(w(ip), 20*log10(pk), 'ko', 'markersize', 7);  hold off;  grid on;
  ylabel('|G| [dB]');  axis([1e-3 1e3 -160 40]);
  title(sprintf('G(s): asymptotes -20 dB/dec then -60 dB/dec, corner %.1f rad/s, peak 1/(2\\zeta) = %.1f dB above the line', wn, 20*log10(1/(2*zt))));
  legend({'exact', 'asymptotes', 'corner w_n', 'exact peak'}, 'location', 'southwest');
  text(0.02, -20, '-20 dB/dec'); text(150, -120, '-60 dB/dec');
  subplot(2,1,2);  semilogx(w, ph, 'b', 'linewidth', 1.8);  hold on;  semilogx(w, phase_asy, 'r--', 'linewidth', 1.6);  semilogx(wn*[1 1], [-300 -60], 'k:');  hold off;
  grid on;  xlabel('\omega [rad/s]');  ylabel('phase [deg]');  axis([1e-3 1e3 -300 -60]);  legend({'exact', 'asymptote (-90, -270)'}, 'location', 'northeast');
  tc_save_figure(hf, 'tc_study_bode_asymptotes');
  R.wn = wn;  R.zeta = zt;  R.peak_db = 20*log10(1/(2*zt));
end
