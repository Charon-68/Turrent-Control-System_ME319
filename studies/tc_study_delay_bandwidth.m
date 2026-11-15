function R = tc_study_delay_bandwidth()
% TC_STUDY_DELAY_BANDWIDTH  Phase margin and crossover versus added loop delay, and the bandwidth that stays safe.
%   A pure delay Td multiplies L by exp(-s Td): |L| is unchanged, the phase falls by wc Td, so
%   PM(Td) = PM(0) - wc Td (deg) and the delay margin is PM/wc (checked numerically below).
%   Delays considered (all added in the loop):
%     - zero-order hold of the controller output: T_call/2 (5 ms at 100 Hz, 25 ms at the 20 Hz radar rate);
%     - estimator latency: ASSUMED 50 ms (one radar period) and 100 ms (two) as bounds, because the nowcast is
%       propagated to the call time, so the true lag is smaller; this is an assumption, not a measurement;
%     - a 0.5 s radar blackout "viewed as a held reference": it is NOT a delay inside the loop (the turret angle
%       is fed back from the encoder), so it is treated in two ways: as a hypothetical loop delay (worst case for
%       stability) and as a reference hold (what it really is), with the resulting pointing error.
%   Safe bandwidth: scale the whole compensator gain by k, take the largest crossover with PM >= 45 deg and
%   GM >= 6 dB for each delay (45 deg / 6 dB are conventional targets, assumed).
%   Writes outputs/tc_study_delay_bandwidth.{txt,mat}, plots/tc_study_delay_bandwidth.{pdf,png}.
  here = fileparts(mfilename('fullpath'));  addpath(fileparts(here));  tc_setup();
  M = tc_course_model();  K = M.K;  rd = fullfile(fileparts(here), 'outputs');
  [say, done] = tc_course_report('tc_study_delay_bandwidth');
  m0 = tc_loop_margins(M.Ln, M.Ld);
  say('Final loop: crossover %.2f rad/s, PM %.1f deg, GM %.1f dB.  PM(Td) = PM - wc Td; delay margin = PM/wc = %.3f s (analytic)\n', m0.wc, m0.pm, m0.gm, m0.pm*pi/180/m0.wc);
  lo = 0;  hi = 2;                                   % numerical delay margin: smallest Td with a crossing at PM <= 0
  for it = 1:60, mid = (lo + hi)/2;  mm = tc_loop_margins(M.Ln, M.Ld, mid);  if min(mm.pm_all) > 0, lo = mid; else, hi = mid; end, end
  Tdm = lo;
  say('  delay margin found by bisection on the delayed loop: %.4f s (analytic %.4f s)\n\n', Tdm, m0.pm*pi/180/m0.wc);
  dl = [0, 0.5/100, 0.5/20, K.dt_radar, 0.5/20 + K.dt_radar, 2*K.dt_radar, 0.5];
  dn = {'none', 'ZOH at 100 Hz (T/2)', 'ZOH at 20 Hz (T/2)', 'estimator latency, 1 radar period (assumed)', 'ZOH 20 Hz + 1 period', 'estimator latency, 2 periods (assumed)', '0.5 s blackout as a loop delay (hypothetical)'};
  say('Phase margin and gain margin of the design loop with added delay\n');
  say('  %-48s %8s %8s %8s %8s %8s\n', 'delay', 'Td [s]', 'wc', 'PM [deg]', 'GM [dB]', 'stable');
  rows = zeros(numel(dl), 5);
  for q = 1:numel(dl)
    mm = tc_loop_margins(M.Ln, M.Ld, dl(q));  pmq = min(mm.pm_all);
    [Np, Dp] = tc_pade_delay(dl(q));  stq = all(real(roots(tc_poly_add(conv(M.Ld, Dp), conv(M.Ln, Np)))) < 0);   % stability from the Pade-8 characteristic polynomial
    rows(q,:) = [dl(q), mm.wc, pmq, mm.gm, stq];
    say('  %-48s %8.4f %8.2f %8.1f %8.1f %8d\n', dn{q}, rows(q,:));
  end
  say('  GM = smallest gain margin over all -180 deg crossings; stable = all roots of the characteristic polynomial with a Pade-8 delay in the open left half plane.\n');
  say('  Phase lost to a delay at the crossover: %.1f deg per 10 ms (wc = %.2f rad/s).  PM reaches zero at Td = %.3f s; 45 deg of PM is kept up to Td = %.3f s.\n', m0.wc*0.01*180/pi, m0.wc, Tdm, (m0.pm - 45)*pi/180/m0.wc);

  th_dot = 150/800;  th_ddot = 20/800 + 150^2/800^2;  tb = 0.5;
  say('\nBlackout (0.5 s) as a held reference: it delays the reference, not the loop\n');
  say('  worst LOS rate %.4f rad/s, worst LOS acceleration %.4f rad/s^2 (a/r + v^2/r^2 at 800 m)\n', th_dot, th_ddot);
  say('  hold the last angle for 0.5 s: pointing error up to rate x 0.5 = %.4f rad = %.0f m at 800 m (budget 2 m)\n', th_dot*tb, th_dot*tb*800);
  say('  hold the last angle AND rate (coast): error from the acceleration 0.5 a t^2 = %.4f rad = %.1f m at 800 m (before the filter''s own uncertainty, see outputs/error_budget.txt)\n', 0.5*th_ddot*tb^2, 0.5*th_ddot*tb^2*800);
  say('  -> a held angle misses the 2 m budget by %.0fx; extrapolating the estimated motion is what keeps the shot possible.  As a loop delay of 0.5 s the PM would be %.0f deg (unstable): the loop does not contain it.\n', th_dot*tb*800/2, rows(end,3));

  ks = logspace(-1.5, 0.6, 140);  pm_req = 45;  gm_req = 6;
  tdl = [0, 0.005, 0.025, 0.05, 0.075, 0.10, 0.25];
  say('\nSafe bandwidth: scale the compensator gain by k (Kp = k x %g V/rad); largest crossover with PM >= %g deg and GM >= %g dB\n', K.kp, pm_req, gm_req);
  say('  %8s %8s %10s %10s %10s %10s %14s\n', 'Td [s]', 'k_max', 'wc_max', 'wb_max', 'PM [deg]', 'GM [dB]', 'wc_max x Td');
  safe = zeros(numel(tdl), 6);
  for q = 1:numel(tdl)
    best = [0 0 0 0 0 0];
    for i = 1:numel(ks)
      Ln = ks(i)*M.Ln;  mm = tc_loop_margins(Ln, M.Ld, tdl(q));
      [Np, Dp] = tc_pade_delay(tdl(q));
      if ~isnan(mm.wc) && min(mm.pm_all) >= pm_req && mm.gm >= gm_req && mm.wc > best(3) && all(real(roots(tc_poly_add(conv(M.Ld, Dp), conv(ks(i)*M.Ln, Np)))) < 0)
        w = logspace(-2, 3, 4000);  L = polyval(Ln, 1j*w)./polyval(M.Ld, 1j*w);  Tw = abs(L./(1 + L));
        ib = find(Tw < 1/sqrt(2) & w > mm.wc/3, 1);  wb = NaN;  if ~isempty(ib), wb = w(ib); end
        best = [ks(i), ks(i)*K.kp, mm.wc, wb, min(mm.pm_all), mm.gm];
      end
    end
    safe(q,:) = best;
    if best(3) > 0, say('  %8.3f %8.3f %10.2f %10.2f %10.1f %10.1f %14.3f\n', tdl(q), best(1), best(3), best(4), best(5), best(6), best(3)*tdl(q));
    else, say('  %8.3f %8s %10s %10s %10s %10s %14s   (no gain scale in %.3g..%.3g meets PM >= %g, GM >= %g)\n', tdl(q), '--', 'none', '--', '--', '--', '--', ks(1), ks(end), pm_req, gm_req); end
  end
  say('  (the closed-loop bandwidth wb here is evaluated without the delay term, as a guide)\n');
  say('  Reading: with the delays that actually exist in the loop (ZOH 5-25 ms; assumed latency 50-100 ms) the safe crossover falls from %.1f rad/s (5 ms) to %.1f rad/s (100 ms); the design crossover is %.2f rad/s,\n', safe(2,3), safe(6,3), m0.wc);
  say('  so the design keeps PM >= 45 deg up to Td = %.3f s (first table) and is at the safe limit at 100 ms.  Safe BANDWIDTH for these delays: closed-loop wb up to ~%.1f rad/s (5 ms) ... %.1f rad/s (100 ms); the design wb = 5.72 rad/s.\n', (m0.pm - 45)*pi/180/m0.wc, safe(2,4), safe(6,4));
  say('  The k-search is not monotone at the smallest delays (the 5 ms row allows a higher gain than Td = 0): a small delay moves the -180 deg crossing down in frequency, away from the resonance peak, which raises the gain margin\n');
  say('  in the first table too (16.1 -> 17.3 dB at 5 ms); from about 25 ms on the phase loss dominates.\n');
  say('  With no delay the limit is the resonance (%.1f rad/s, gain margin), not the delay: the safe crossover saturates near %.1f rad/s at Td = 0.\n', sqrt(M.a(3)/M.a(1)), safe(1,3));
  say('  Rule of thumb that follows: wc x Td should stay under about %.2f (rad) to keep 45 deg of margin from a starting PM of %.0f deg.\n', (m0.pm - 45)*pi/180, m0.pm);
  done();
  R.hold_m = th_dot*tb*800;  R.coast_m = 0.5*th_ddot*tb^2*800;  R.rows = rows;  R.safe = safe;  R.tdl = tdl;  R.Tdm = Tdm;  R.names = dn;
  save('-mat', fullfile(rd, 'tc_study_delay_bandwidth.mat'), 'R');

  hf = figure('visible', 'off', 'position', [50 50 1100 430], 'color', 'w');
  td = linspace(0, 0.6, 300);  pmc = arrayfun(@(x) min(tc_loop_margins(M.Ln, M.Ld, x).pm_all), td);
  subplot(1,2,1);  plot(td, pmc, 'b', 'linewidth', 1.8);  hold on;  plot(td, 0*td, 'k:');  plot(td, 45 + 0*td, 'g--');
  for x = [0.025 0.05 0.5], plot(x*[1 1], [-120 80], 'r:'); end
  hold off;  grid on;  xlabel('added loop delay T_d [s]');  ylabel('phase margin [deg]');
  title(sprintf('PM(T_d) of the design loop: zero at T_d = %.3f s', Tdm));  legend({'PM', '0', '45 deg target', 'ZOH 20 Hz, 1 radar period, 0.5 s blackout'}, 'location', 'northeast');
  ok = safe(:,3) > 0;
  subplot(1,2,2);  semilogy(tdl(ok), safe(ok,3), 'ko-', 'linewidth', 1.6);  hold on;  semilogy([0 0.25], m0.wc*[1 1], 'b--');  hold off;  grid on;
  xlabel('loop delay T_d [s]');  ylabel('largest crossover, PM >= 45 deg, GM >= 6 dB [rad/s]');  title('Safe crossover versus delay');  legend({'safe crossover', 'design crossover'}, 'location', 'northeast');
  tc_save_figure(hf, 'tc_study_delay_bandwidth');
end
