function R = tc_study_sampling()
% TC_STUDY_SAMPLING  Sampling facts for the design: radar rate against Nyquist, LOS-rate content of the worst-case
%   target, zero-order-hold delay, and the controller call rate against the plant resonance.  Table of conclusions.
%   Read-only inputs: tc_params (radar period, targets, plant), outputs/design_report.txt (continuous-equivalent
%   ZOH margins computed by the design script) and the rate_20Hz row of outputs/ablations.txt.
%   Writes outputs/tc_study_sampling.{txt,mat}, plots/tc_study_sampling.{pdf,png}.
  here = fileparts(mfilename('fullpath'));  addpath(fileparts(here));  tc_setup();
  M = tc_course_model();  K = M.K;  rd = fullfile(fileparts(here), 'outputs');
  [say, done] = tc_course_report('tc_study_sampling');
  T = K.dt_radar;  fs = 1/T;  wNyq = pi/T;  wres = sqrt(M.a(3)/M.a(1));  m0 = tc_loop_margins(M.Ln, M.Ld);
  say('1. Radar rate against Nyquist\n');
  say('   radar period %.0f ms -> %.0f Hz, Nyquist %.1f Hz = %.2f rad/s.  Plant resonance %.2f rad/s = %.2f Hz is %.0f %% of that Nyquist; loop crossover %.2f rad/s is %.1f %%.\n', 1e3*T, fs, fs/2, wNyq, wres, wres/2/pi, 100*wres/wNyq, m0.wc, 100*m0.wc/wNyq);
  th1 = K.v_tgt_max/K.r_min;  th2 = K.a_tgt_max/K.r_min + K.v_tgt_max^2/K.r_min^2;
  weq = th2/th1;
  say('\n2. LOS-rate content of the worst-case target (v = %g m/s, a = %g m/s^2, r = %g m)\n', K.v_tgt_max, K.a_tgt_max, K.r_min);
  say('   LOS rate    |theta_dot|  <= v/r = %.4f rad/s (%.2f deg/s)\n', th1, th1*180/pi);
  say('   LOS accel   |theta_ddot| <= a/r + v^2/r^2 = %.4f rad/s^2  (tangential acceleration plus 2 v_r v_t / r^2 at 45 deg to the line of sight)\n', th2);
  say('   equivalent frequency: a sinusoidal LOS rate of amplitude %.4f rad/s with acceleration amplitude %.4f rad/s^2 oscillates at w = theta_ddot/theta_dot = %.3f rad/s = %.3f Hz\n', th1, th2, weq, weq/2/pi);
  say('   -> %.4f of the radar Nyquist; %.0f radar samples per cycle (Nyquist asks for 2).\n', weq/wNyq, fs/(weq/2/pi));
  say('   motion between two radar samples: rate x T = %.2f mrad = %.1f m of cross-range at 800 m (this is what the estimator extrapolates; it is large against the 2 m budget, small against the sampling theorem).\n', 1e3*th1*T, th1*T*K.r_min);
  say('   fastest maneuver family simulated (simulation/tc_independent_generator.m, bang-bang acceleration reversing every 1 s): content up to ~2 pi/(1 s) = %.1f rad/s = %.0f %% of the Nyquist - still below it, but not by a large margin; that family is a stress case, not part of the handout envelope.\n', 2*pi, 100*2*pi/wNyq);
  say('   Radar noise is white and sampled at the radar instants: there is no signal above Nyquist to alias (the target signal sits ~%.0fx below it), so no anti-alias filter is needed; the noise itself is what the Kalman filter smooths.\n', wNyq/weq);

  say('\n3. Zero-order-hold delay of a sampled controller output\n');
  rates = [20 50 100 1000];  Tz = 1./rates/2;
  say('   %-8s %10s %14s %16s %16s %18s\n', 'f_call', 'T/2 [ms]', 'phase at wc', 'phase at LOS w', 'phase at wres', '|ZOH| at wres');
  rows = zeros(numel(rates), 9);
  for q = 1:numel(rates)
    Th = 1/rates(q);  zoh = @(w) (1 - exp(-1j*w*Th))./(1j*w*Th);
    ph_c = angle(zoh(m0.wc))*180/pi;  ph_l = angle(zoh(weq))*180/pi;  ph_r = angle(zoh(wres))*180/pi;  mg_r = abs(zoh(wres));
    say('   %-8d %10.1f %12.2f deg %14.3f deg %14.1f deg %18.3f\n', rates(q), 1e3*Tz(q), ph_c, ph_l, ph_r, mg_r);
    rows(q,1:5) = [rates(q), Tz(q), ph_c, ph_l, mg_r];
  end
  say('   (ZOH(jw) = (1 - e^{-jwT})/(jwT) = sinc e^{-jwT/2}: it equals a delay of T/2 only for wT << 1.)\n');
  say('   The 20 Hz row is the radar-synchronous worst case: %.1f deg lost at crossover, %.1f deg at the LOS-content frequency.\n', -rows(1,3), -rows(1,4));

  say('\n4. Controller call rate against the plant resonance (%.1f rad/s)\n', wres);
  say('   continuous-equivalent loop with the exact ZOH factor (1 - e^{-sT})/(sT) on C(s); margins recomputed here and compared with outputs/design_report.txt\n');
  tk = regexp(fileread(fullfile(rd, 'design_report.txt')), 'ZOH plant,\s+(\d+) Hz\): wc ([\d.]+) rad/s, PM ([\d.]+) deg, GM ([\d.]+) dB', 'tokens');
  say('   %-8s %12s %10s %10s %10s %10s | %24s\n', 'f_call', 'w_s/w_res', 'Nyq/w_res', 'PM [deg]', 'GM [dB]', 'wc/Nyq', 'design_report.txt PM/GM');
  w = logspace(-3, 3.5, 60000);  s = 1j*w;
  for q = [3 2 1 4]
    Th = 1/rates(q);  Lz = polyval(M.Ln, s)./polyval(M.Ld, s) .* (1 - exp(-s*Th))./(s*Th);
    mm = margins_from_L(w, Lz);
    ref = '';
    for k = 1:numel(tk), if str2double(tk{k}{1}) == rates(q), ref = sprintf('%s / %s', tk{k}{3}, tk{k}{4}); end, end
    say('   %-8d %12.1f %10.2f %10.1f %10.1f %10.3f | %24s\n', rates(q), 2*pi*rates(q)/wres, pi*rates(q)/wres, mm(1), mm(2), m0.wc/(pi*rates(q)), ref);
    rows(q,6:9) = [mm(1), mm(2), 2*pi*rates(q)/wres, pi*rates(q)/wres];
  end
  say('   At 20 Hz the resonance (%.1f rad/s) is %.0f %% of the controller Nyquist (%.1f rad/s): a 20 Hz controller cannot shape it, only gain-stabilise it; the continuous-equivalent model is\n', wres, 100*wres/(pi*20), pi*20);
  say('   reliable only where the crossover is well below Nyquist (crossover/Nyquist = %.3f at 20 Hz; the comment in tc_design_report.m says the two models agree below about Nyquist/5), which holds for the crossover but NOT near the resonance.\n', m0.wc/(pi*20));
  say('   The exact sampled-data C(z) loop was not re-derived here; the design report quotes the same continuous-equivalent method.\n');
  tk2 = regexp(fileread(fullfile(rd, 'ablations.txt')), 'rate_20Hz\s+(\d+)/(\d+)\s+(\d+) \|\s+([+-][\d.]+) \[\s*([+-][\d.]+),\s*([+-][\d.]+)\]\s+[\d.]+ \|\s+([+-][\d.]+)', 'tokens', 'once');
  if ~isempty(tk2)
    say('   Simulation evidence (outputs/ablations.txt, seeds 201-260): 20 Hz call rate intercepts %s/%s with %s pulse miss(es), first hit later by %s s [%s, %s], RMS pointing %s m worse.\n', tk2{1}, tk2{2}, tk2{3}, tk2{4}, tk2{5}, tk2{6}, tk2{7});
  end

  say('\n5. Table of conclusions\n');
  say('   %-46s %-34s %s\n', 'question', 'number', 'conclusion');
  say('   %-46s %-34s %s\n', 'radar sampling vs target content', sprintf('%.3f rad/s vs Nyquist %.1f rad/s', weq, wNyq), 'no aliasing of the target; 20 Hz is ample for the LOS signal');
  say('   %-46s %-34s %s\n', 'motion per radar sample at worst case', sprintf('%.1f m at 800 m', th1*T*K.r_min), 'estimator must extrapolate; hold would exceed the 2 m budget');
  say('   %-46s %-34s %s\n', 'ZOH delay at the 20 Hz call rate', sprintf('%.1f ms, %.1f deg at wc', 1e3*Tz(1), -rows(1,3)), 'small against PM 68.5 deg');
  say('   %-46s %-34s %s\n', 'plant resonance vs 20 Hz Nyquist', sprintf('%.1f of %.1f rad/s (%.0f %%)', wres, pi*20, 100*wres/(pi*20)), 'gain-stabilised only; not shaped');
  say('   %-46s %-34s %s\n', 'plant resonance vs 100 Hz call rate', sprintf('Nyq/w_res = %.1f', pi*100/wres), 'resonance well inside the band; continuous model valid');
  say('   %-46s %-34s %s\n', 'crossover vs 20 Hz Nyquist', sprintf('%.3f', m0.wc/(pi*20)), 'continuous-equivalent margins trustworthy at crossover');
  done();
  R.rows = rows;  R.weq = weq;  R.th1 = th1;  R.th2 = th2;  R.wNyq = wNyq;  R.wres = wres;
  save('-mat', fullfile(rd, 'tc_study_sampling.mat'), 'R');

  hf = figure('visible', 'off', 'position', [50 50 1150 430], 'color', 'w');
  subplot(1,2,1);  ax = [1e-2 1e3];  plot(ax, [0 0], 'k');  hold on;  set(gca, 'xscale', 'log');
  semilogx([0.01 weq], [3 3], 'g', 'linewidth', 8);  semilogx([0.01 m0.wc], [2 2], 'b', 'linewidth', 8);
  semilogx(wres*[1 1], [0 4], 'r', 'linewidth', 2);  semilogx(wNyq*[1 1], [0 4], 'k--', 'linewidth', 2);  semilogx(pi*100*[1 1], [0 4], 'k:', 'linewidth', 2);
  semilogx(2*pi*[1 1], [0 4], 'm-.', 'linewidth', 1.5);
  hold off;  axis([1e-2 1e3 0 4.5]);  grid on;  set(gca, 'ytick', []);  xlabel('\omega [rad/s]');
  title('Frequency map: LOS content, loop, resonance, Nyquist limits');
  legend({'', 'LOS content (worst case)', 'loop crossover band', sprintf('resonance %.1f', wres), sprintf('radar Nyquist %.1f', wNyq), 'Nyquist at 100 Hz call rate', 'bang-bang 1 s (2\pi/1 s)'}, 'location', 'northwest');
  subplot(1,2,2);  hold on;  wz = logspace(-1, 3, 3000);  cc = {'r', 'm', 'b', 'k'};
  for q = 1:4, Th = 1/rates(q);  zz = (1 - exp(-1j*wz*Th))./(1j*wz*Th);  semilogx(wz, unwrap(angle(zz))*180/pi, cc{q}, 'linewidth', 1.5); end
  set(gca, 'xscale', 'log');  semilogx(m0.wc*[1 1], [-400 0], 'g:');  semilogx(wres*[1 1], [-400 0], 'r:');  hold off;
  grid on;  axis([0.1 1000 -400 0]);  xlabel('\omega [rad/s]');  ylabel('ZOH phase [deg]');  title('Zero-order-hold phase at call rates 20 / 50 / 100 / 1000 Hz');
  legend({'20 Hz', '50 Hz', '100 Hz', '1000 Hz', 'crossover', 'resonance'}, 'location', 'southwest');
  tc_save_figure(hf, 'tc_study_sampling');
end

function v = margins_from_L(w, L)
% [PM deg (lowest), GM dB (smallest over -180 crossings)] of a frequency response on a log grid
  mag = abs(L);  ph = unwrap(angle(L))*180/pi;  if ph(1) > 0, ph = ph - 360; end
  ic = find(mag(1:end-1) >= 1 & mag(2:end) < 1);  pm = zeros(size(ic));
  for q = 1:numel(ic), i = ic(q);  fr = log(mag(i))/(log(mag(i)) - log(mag(i+1)));  pm(q) = 180 + ph(i) + fr*(ph(i+1) - ph(i)); end
  gm = [];
  for lev = -180 + 360*(-3:1)
    ip = find((ph(1:end-1) - lev).*(ph(2:end) - lev) < 0);
    for q = 1:numel(ip), i = ip(q);  fr = (lev - ph(i))/(ph(i+1) - ph(i));  gm(end+1) = -20*log10(exp(log(mag(i)) + fr*(log(mag(i+1)) - log(mag(i))))); end %#ok<AGROW>
  end
  v = [min(pm), min([gm Inf])];
end
