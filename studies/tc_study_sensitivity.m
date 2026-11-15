function R = tc_study_sensitivity()
% TC_STUDY_SENSITIVITY  S, T and the controller sensitivity K*S of the final loop L = C G_p.
%   S = 1/(1+L), T = L/(1+L), KS = C/(1+L) (volts per radian of reference or sensor error).
%   Bandwidth, peaks, and where radar noise enters.  The loop polynomials are the design script's
%   (tc_course_model; nominal margins are cross-checked against outputs/analysis.txt).
%   Writes outputs/tc_study_sensitivity.{txt,mat}, plots/tc_study_sensitivity.{pdf,png}.
  here = fileparts(mfilename('fullpath'));  addpath(fileparts(here));  tc_setup();
  M = tc_course_model();  K = M.K;  rd = fullfile(fileparts(here), 'outputs');
  [say, done] = tc_course_report('tc_study_sensitivity');
  mg = tc_loop_margins(M.Ln, M.Ld);
  w = logspace(-2, 3, 20000);  s = 1j*w;
  L = polyval(M.Ln, s)./polyval(M.Ld, s);  C = polyval(M.Cn, s)./polyval(M.Cd, s);
  S = 1./(1 + L);  T = L.*S;  KS = C.*S;
  [Ms, i1] = max(abs(S));  [Mt, i2] = max(abs(T));  [Mks, i3] = max(abs(KS));
  ib = find(abs(T) < 1/sqrt(2) & w > mg.wc/3, 1);  wb = interp1(abs(T(ib-1:ib)), w(ib-1:ib), 1/sqrt(2));
  isb = find(abs(S) >= 1/sqrt(2), 1);  ws = interp1(abs(S(isb-1:isb)), w(isb-1:isb), 1/sqrt(2));
  wN = pi/K.dt_radar;                                        % radar Nyquist, rad/s
  wres = sqrt(M.a(3)/M.a(1));
  at = @(x, f) interp1(w, abs(x), f);
  say('Loop L = C G_p, final design (Kp = %g V/rad, Ti = %g s, tau_f = %g s)\n', K.kp, K.ti, K.tau_f);
  say('Cross-check against outputs/analysis.txt (nominal): crossover %.2f rad/s, PM %.1f deg, GM %.1f dB (file: 4.10, 68.5, 16.1)\n', mg.wc, mg.pm, mg.gm);
  say('\nSensitivity functions\n');
  say('  peak |S| = Ms = %.3f (%.2f dB) at %.2f rad/s   (analysis.txt: 1.21)\n', Ms, 20*log10(Ms), w(i1));
  say('  peak |T| = Mt = %.3f (%.2f dB) at %.2f rad/s\n', Mt, 20*log10(Mt), w(i2));
  say('  closed-loop bandwidth (|T| = -3 dB): %.2f rad/s = %.2f Hz;  |S| rises through -3 dB at %.2f rad/s (disturbance rejection band)\n', wb, wb/2/pi, ws);
  say('  |S| at 0.1 / 1 rad/s: %.4f / %.3f;  |S| < 0.1 below %.3f rad/s\n', at(S, 0.1), at(S, 1), w(find(abs(S) > 0.1, 1)));
  say('  |T| at 1 / 10 / wN = %.1f rad/s / resonance %.1f rad/s: %.3f / %.3f / %.4f (%.1f dB) / %.4f (%.1f dB)\n', wN, wres, at(T,1), at(T,10), at(T,wN), 20*log10(at(T,wN)), at(T,wres), 20*log10(at(T,wres)));
  say('  S + T = 1 check: max |S + T - 1| = %.1e\n', max(abs(S + T - 1)));
  say('  controller sensitivity KS = C S: peak %.0f V/rad at %.2f rad/s; at wN %.0f V/rad; at the resonance %.0f V/rad; high-frequency limit |C| -> Kp/(tau_f w)\n', Mks, w(i3), at(KS, wN), at(KS, wres));
  say('  (a 1 mrad sensor/estimator error at the resonance frequency would ask for %.2f V)\n', 1e-3*at(KS, wres));

  % ---- where noise enters ----------------------------------------------------------------------------
  wl = linspace(1e-3, wN, 40000);  sl = 1j*wl;
  Ll = polyval(M.Ln, sl)./polyval(M.Ld, sl);  Tl = Ll./(1 + Ll);  Fl = (K.p_dev./(sl + K.p_dev)).^3;
  encT = trapz(wl, abs(Tl).^2)/wN;  encF = trapz(wl, abs(Fl).^2)/wN;  encTF = trapz(wl, abs(Tl.*Fl).^2)/wN;
  sd = K.sig_th;  r2 = 2000;
  say('\nWhere radar noise enters\n');
  say('  White radar bearing noise (sigma_theta = %.3e rad per sample, flat to the %.1f rad/s Nyquist of the 20 Hz radar) reaching the loop as a reference error e_ref:\n', sd, wN);
  say('  - feedback-only path: theta = T e_ref, so noise inside the bandwidth (|T| ~ 1 up to %.1f rad/s) passes untouched: the loop can only remove the part above it.\n', wb);
  say('    fraction of noise power passed (integral of |T|^2 over 0..wN divided by wN): %.3f (amplitude %.3f);  cross-range at %d m if the raw bearing drove the loop: %.3f m (raw %.3f m)\n', encT, sqrt(encT), r2, r2*sd*sqrt(encT), r2*sd);
  say('  - with the exact feed-forward added (our 2-DOF design) the nominal map from the reference to theta is 1 (G * G^-1 = 1): the loop does NOT attenuate reference noise at all;\n');
  say('    the only low-pass left is the reference generator (triple pole p = %g rad/s): power fraction %.3f, |F| at wN = %.4f (%.1f dB)\n', K.p_dev, encF, abs(Fl(end)), 20*log10(abs(Fl(end))));
  say('  - T and F both have bandwidth far above the LOS maneuver band (<= ~0.3 rad/s, see tc_study_sampling), so neither shapes the in-band noise: it must be removed by the estimator before the loop.\n');
  tk = regexp(fileread(fullfile(rd, 'error_budget.txt')), 'raw ([\d.]+) m, filtered ([\d.]+) m, after 0.5 s blackout ([\d.]+) m', 'tokens', 'once');
  if ~isempty(tk)
    say('    That is the Kalman filter: its steady-state cross-range error at 2000 m is %s m against %s m raw, and %s m after a 0.5 s blackout (outputs/error_budget.txt).\n', tk{2}, tk{1}, tk{3});
  end
  say('  Disturbance (torque) rejection: |S| < 0.1 below %.3f rad/s; at the plant resonance |S| = %.2f, i.e. a disturbance there is amplified by %.0f %% (the peak Ms = %.2f is the worst case, at %.1f rad/s).\n', w(find(abs(S) > 0.1, 1)), at(S, wres), 100*(at(S, wres) - 1), Ms, w(i1));
  say('  Voltage cost of noise: the feedback path adds |KS| (peak %.0f V/rad); the feed-forward path costs |G^-1 F| at wN = %.0f V/rad (G^-1 = s (a2 s^2 + a1 s + a0)/gn).\n', Mks, abs(polyval([M.a 0], 1j*wN)/M.gn*Fl(end)));
  done();
  R.Ms = Ms;  R.Mt = Mt;  R.wb = wb;  R.ws = ws;  R.wc = mg.wc;  R.pm = mg.pm;  R.gm = mg.gm;  R.encT = encT;  R.encF = encF;
  R.KSpk = Mks;  R.wN = wN;  R.T_at_N = at(T, wN);  R.wMs = w(i1);
  save('-mat', fullfile(rd, 'tc_study_sensitivity.mat'), 'R');

  hf = figure('visible', 'off', 'position', [50 50 1100 430], 'color', 'w');
  subplot(1,2,1);  semilogx(w, 20*log10(abs(S)), 'b', 'linewidth', 1.8);  hold on;  semilogx(w, 20*log10(abs(T)), 'r', 'linewidth', 1.8);
  semilogx(w, zeros(size(w)), 'k:');  semilogx(wb*[1 1], [-60 12], 'r:');  semilogx(wN*[1 1], [-60 12], 'k--');  hold off;
  grid on;  axis([1e-2 1e3 -60 12]);  xlabel('\omega [rad/s]');  ylabel('magnitude [dB]');
  title(sprintf('|S| (peak %.2f) and |T|; bandwidth %.1f rad/s', Ms, wb));
  legend({'|S|', '|T|', '0 dB', sprintf('bandwidth %.1f', wb), sprintf('radar Nyquist %.1f', wN)}, 'location', 'southwest');
  subplot(1,2,2);  semilogx(w, 20*log10(abs(T)), 'r', 'linewidth', 1.6);  hold on;
  semilogx(w, 20*log10(abs((K.p_dev./(s + K.p_dev)).^3)), 'm-.', 'linewidth', 1.6);
  semilogx(w, 20*log10(abs(T).*abs((K.p_dev./(s + K.p_dev)).^3)), 'k', 'linewidth', 1.2);
  semilogx(wN*[1 1], [-100 12], 'k--');  semilogx([0.01 0.3], [8 8], 'g', 'linewidth', 6);  hold off;
  grid on;  axis([1e-2 1e3 -100 12]);  xlabel('\omega [rad/s]');  ylabel('magnitude [dB]');
  title(sprintf('Radar noise (flat to Nyquist): passes |T| up to %.1f rad/s; power passed %.2f', wb, encT));
  legend({'|T| (feedback-only path)', '|F| generator (p = 20)', '|T F|', 'radar Nyquist', 'LOS maneuver band'}, 'location', 'southwest');
  tc_save_figure(hf, 'tc_study_sensitivity');

  % single-column version for the paper (3.5 in wide, two stacked panels, 7 pt text)
  hc = figure('visible', 'off', 'position', [50 50 350 370], 'color', 'w');
  ax1 = axes('position', [0.14 0.60 0.83 0.29]);  semilogx(w, 20*log10(abs(S)), 'b', 'linewidth', 1.3);  hold on;  semilogx(w, 20*log10(abs(T)), 'r', 'linewidth', 1.3);
  semilogx(wb*[1 1], [-60 12], 'k:');  semilogx(wN*[1 1], [-60 12], 'k--');  hold off;  grid on;  axis([1e-2 1e3 -60 12]);  ylabel('[dB]');  set(ax1, 'xticklabel', []);
  text(0.013, -50, '|S|', 'color', 'b');  text(0.013, 6, '|T|', 'color', 'r');  text(wb*1.1, -52, sprintf('w_b = %.1f', wb));  text(wN*0.14, 7, 'Nyquist');
  title(sprintf('(a) |S| (M_s = %.2f) and |T|', Ms));
  ax2 = axes('position', [0.14 0.12 0.83 0.29]);  semilogx(w, 20*log10(abs(T)), 'r', 'linewidth', 1.3);  hold on;
  semilogx(w, 20*log10(abs((K.p_dev./(s + K.p_dev)).^3)), 'm-.', 'linewidth', 1.3);  semilogx(w, 20*log10(abs(T).*abs((K.p_dev./(s + K.p_dev)).^3)), 'k', 'linewidth', 1.0);
  semilogx(wN*[1 1], [-100 12], 'k--');  semilogx([0.01 0.3], [6 6], 'g', 'linewidth', 4);  hold off;  grid on;  axis([1e-2 1e3 -100 12]);  xlabel('\omega [rad/s]');  ylabel('[dB]');
  text(1.5, -45, '|T|', 'color', 'r');  text(25, -22, '|F|', 'color', 'm');  text(3, -60, '|T F|');  text(0.013, -2, 'LOS band', 'color', [0 0.5 0]);
  title('(b) radar noise passes |T| and |F| (p = 20)');
  set(findall(hc, '-property', 'fontsize'), 'fontsize', 7);
  tc_save_figure(hc, 'tc_study_sensitivity_col');
end
