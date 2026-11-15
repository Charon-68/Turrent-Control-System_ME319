function R = tc_study_compensator_compare()
% TC_STUDY_COMPENSATOR_COMPARE  Design-rationale study on the LINEAR plant only (nothing is wired into the pipeline).
%   Compared: P (Kp as in the final design), P at a higher gain, a lead compensator from the lecture procedure,
%   and the PI + roll-off compensator we use.  Reported: PM, GM, crossover, closed-loop bandwidth, velocity
%   constant Kv (and Ka), ramp error, step response, controller gain at the radar Nyquist frequency.
%
%   Lead procedure (lecture), C = K (1 + T s)/(1 + alpha T s):
%     1. K from the velocity-constant requirement Kv_req (assumed 8 1/s: twice the P design's 4.0);
%     2. PM0 of K G(s) (lowest crossover margin, because the resonance can lift |L| above 1 again);
%     3. phase needed phi_m = PM_des - PM0 + eps  (PM_des = 60 deg, eps = 10 deg: assumed targets);
%     4. alpha = (1 - sin phi_m)/(1 + sin phi_m);
%     5. corner placement: new crossover wm where |K G(j wm)| = sqrt(alpha); T = 1/(wm sqrt(alpha)).
%   If step 3 gives phi_m <= 0 the procedure asks for no lead.  That is what happens here (see output), so the
%   lead that is evaluated is a FORCED lead (phi_m = 30 deg, same steps 4-5) to show what a lead would do.
%   A sweep of Kv_req shows the procedure never asks for a feasible lead on this plant.
%   Writes outputs/tc_study_compensator_compare.{txt,mat}, plots/tc_study_compensator_compare.{pdf,png}.
  here = fileparts(mfilename('fullpath'));  addpath(fileparts(here));  tc_setup();
  M = tc_course_model();  K = M.K;  rd = fullfile(fileparts(here), 'outputs');
  [say, done] = tc_course_report('tc_study_compensator_compare');
  Kvp = M.gn/M.a(3);  wN = pi/K.dt_radar;
  say('Linear plant G = %.0f/(s a(s)); plant velocity constant gn/a0 = %.4e rad/s per V; type 1 (one free integrator).\n\n', M.gn, Kvp);

  % ---- lecture procedure ------------------------------------------------------------------------------
  Kv_req = 8;  PM_des = 60;  eps_d = 10;
  Klead = Kv_req/Kvp;  m0 = tc_loop_margins(Klead*M.Gn, M.Gd);  PM0 = min(m0.pm_all);
  phi_needed = PM_des - PM0 + eps_d;
  say('Lead design by the lecture procedure\n');
  say('  step 1  Kv_req = %g 1/s  ->  K = Kv_req/(gn/a0) = %.0f V/rad\n', Kv_req, Klead);
  say('  step 2  uncompensated K G: crossover %.2f rad/s, PM0 = %.1f deg, GM0 = %.1f dB\n', m0.wc, PM0, m0.gm);
  say('  step 3  phi_m = PM_des (%g) - PM0 + eps (%g) = %.1f deg  ->  %s\n', PM_des, eps_d, phi_needed, ifelse(phi_needed <= 0, 'NO PHASE LEAD NEEDED (alpha = 1)', 'lead needed'));
  % sweep of Kv_req
  say('  Sweep of Kv_req (same PM_des, eps): what the procedure would ask for on this plant\n');
  say('    %8s %10s %10s %10s %10s  %s\n', 'Kv_req', 'K [V/rad]', 'PM0 [deg]', 'GM0 [dB]', 'phi_m', 'verdict');
  for kv = [2 4 8 10 10.4 10.6 12 15 20 40 75]
    mm = tc_loop_margins(kv/Kvp*M.Gn, M.Gd);  p0 = min(mm.pm_all);  ph = PM_des - p0 + eps_d;
    if ph <= 0, v = 'no lead needed'; elseif ph >= 90, v = 'single lead cannot supply it'; else, v = 'lead needed'; end
    say('    %8.1f %10.0f %10.1f %10.1f %10.1f  %s%s\n', kv, kv/Kvp, p0, mm.gm, ph, v, ifelse(mm.gm < 0, ' (and the loop is unstable: GM < 0)', ''));
  end
  say('  Reading: below Kv = %.2f (the Routh limit of the P loop, Kp < %.0f V/rad) PM0 is ~90 deg, no lead is ever needed; above it the resonance at %.1f rad/s lifts |L| over 1 again,\n', M.a(2)*M.a(3)/(M.a(1)*M.gn)*Kvp, M.a(2)*M.a(3)/(M.a(1)*M.gn), sqrt(M.a(3)/M.a(1)));
  say('  the margin there is hugely negative and the demanded phase exceeds 90 deg, which one lead cannot give.  The limit is the resonance (gain), not phase.\n');
  phi_m = 30;
  [alpha, Tl, wm] = lead_design(M, Kvp, Kv_req, phi_m);
  say('  FORCED lead (phi_m = %g deg): alpha = %.4f, new crossover wm where |K G| = sqrt(alpha) = %.3f: wm = %.2f rad/s, T = 1/(wm sqrt(alpha)) = %.4f s\n', phi_m, alpha, sqrt(alpha), wm, Tl);
  say('    zero at 1/T = %.2f rad/s, pole at 1/(alpha T) = %.2f rad/s, K = %.0f V/rad, high-frequency gain K/alpha = %.0f V/rad\n', 1/Tl, 1/(alpha*Tl), Klead, Klead/alpha);
  Kv2 = 2;  Klead2 = Kv2/Kvp;  [alpha2, Tl2, wm2] = lead_design(M, Kvp, Kv2, phi_m);
  say('  FORCED lead, low-gain variant (Kv_req = %g, so the loop can survive): alpha = %.4f, wm = %.2f rad/s, T = %.4f s, K = %.0f V/rad, high-frequency gain %.0f V/rad\n\n', Kv2, alpha2, wm2, Tl2, Klead2, Klead2/alpha2);

  % ---- the five loops -------------------------------------------------------------------------------------
  name = {'P, Kp = 4000', 'P, K = 8012', 'lead 30 deg, Kv 8', 'lead 30 deg, Kv 2', 'PI + roll-off (final)'};
  Cn = {K.kp, Klead, Klead*[Tl 1], Klead2*[Tl2 1], M.Cn};  Cd = {1, 1, [alpha*Tl 1], [alpha2*Tl2 1], M.Cd};
  t = 0:0.0005:15;  rows = zeros(5, 12);  Ysteps = cell(5,1);  Ss = cell(4,1);
  w = logspace(-2, 3, 20000);  s = 1j*w;
  for q = 1:5
    Ln = conv(M.Gn, Cn{q});  Ld = conv(M.Gd, Cd{q});  cl = tc_poly_add(Ld, Ln);
    mg = tc_loop_margins(Ln, Ld);  stab = all(real(roots(cl)) < 0);
    L = polyval(Ln, s)./polyval(Ld, s);  Tw = abs(L./(1 + L));  ib = find(Tw < 1/sqrt(2) & w > mg.wc/3, 1);
    wb = NaN;  if ~isempty(ib), wb = w(ib); end
    Cw = polyval(Cn{q}, 1j*wN)/polyval(Cd{q}, 1j*wN);  Sw = 1/(1 + polyval(Ln, 1j*wN)/polyval(Ld, 1j*wN));
    % velocity and acceleration constants from the low-frequency expansion of L
    ndeg0 = find(abs(Ld) > 0, 1, 'last');  ztype = sum(abs(roots(Ld)) < 1e-9);       % number of integrators in L
    if ztype == 1, Kv = polyval(Ln, 0)/polyval(Ld(1:end-1), 0);  Ka = 0;
    else, Kv = Inf;  Ka = polyval(Ln, 0)/polyval(Ld(1:end-2), 0);  end
    if stab
      y = tc_lin_response(Ln, cl, 'step', 1, t);  sp = tc_step_specs(t, y);
      er = tc_lin_response(Ld, cl, 'ramp', 1, t);                              % ramp error for a 1 rad/s ramp
      e_ramp = er(end);
    else
      y = NaN(size(t));  sp = struct('tr', NaN, 'tp', NaN, 'os', NaN, 'ts', NaN);  e_ramp = NaN;
    end
    Ysteps{q} = y;
    rows(q,:) = [mg.wc, min(mg.pm_all), mg.gm, wb, Kv, Ka, e_ramp, sp.os, sp.tr, sp.ts, abs(Cw*Sw), stab];
  end
  say('Comparison (linear plant, nominal)\n');
  say('%-22s %7s %7s %7s %7s %9s %8s %12s %8s %7s %7s %12s %7s\n', 'compensator', 'wc', 'PM', 'GM', 'wb', 'Kv', 'Ka', 'ramp err/rate', 'overshoot', 'tr', 'ts', '|CS|(wN)', 'stable');
  say('%-22s %7s %7s %7s %7s %9s %8s %12s %8s %7s %7s %12s %7s\n', '', '[rad/s]', '[deg]', '[dB]', '[rad/s]', '[1/s]', '[1/s^2]', '[s]', '[%]', '[s]', '[s]', '[V/rad]', '');
  for q = 1:5
    r = rows(q,:);
    say('%-22s %7.2f %7.1f %7.1f %7.2f %9.3g %8.3g %12.4f %8.1f %7.2f %7.2f %12.0f %7d\n', name{q}, r(1), r(2), r(3), r(4), r(5), r(6), r(7), r(8), r(9), r(10), r(11), r(12));
  end
  say('  ramp err/rate: steady-state error divided by the ramp rate (= 1/Kv for a type-1 loop, 0 for type 2).  Kv = Inf means type 2 (Ka is the acceleration constant).\n');
  wl = pi/K.dt_radar;
  say('\nWhat that means for this problem\n');
  say('  Tracking a ramp-like LOS rate: the worst LOS rate is v/r = 150/800 = %.4f rad/s.  Feedback-only error with the type-1 loops: P (Kv %.2f) %.4f rad = %.0f m at 800 m;\n', 150/800, rows(1,5), 150/800/rows(1,5), 800*150/800/rows(1,5));
  say('  P with K = 8012 (Kv %.2f) %.4f rad = %.0f m at 800 m; the PI loop is type 2: zero ramp error.  (Budget: 2 m, i.e. %.4f rad at 800 m.)\n', rows(2,5), 150/800/rows(2,5), 800*150/800/rows(2,5), 2/800);
  say('  Noise: the controller gain seen by radar-rate noise (|C S| at the Nyquist %.1f rad/s) is %.0f V/rad for the PI + roll-off, %.0f for P, %.0f for the Kv-8 lead (%.1fx the PI; the Kv-2 lead: %.0f).\n', wl, rows(5,11), rows(1,11), rows(3,11), rows(3,11)/rows(5,11), rows(4,11));
  say('  Margins: the P loop with the higher gain has GM %.1f dB (Routh limit Kp < %.0f); a lead raises the high-frequency gain by 1/alpha = %.1f and moves the GM the wrong way: %.1f dB at Kv 8 (unstable if negative), %.1f dB even at Kv 2 (the P loop at Kv 2 has %.1f dB).\n', rows(2,3), M.a(2)*M.a(3)/(M.a(1)*M.gn), 1/alpha, rows(3,3), rows(4,3), tc_loop_margins(Klead2*M.Gn, M.Gd).gm);
  say('  Why the chosen one wins: (1) the plant is type 1, so a ramp-like LOS rate leaves a finite feedback error 1/Kv with P or lead, and Kv is capped by the resonance gain limit; the PI adds the second integrator (type 2, zero ramp error);\n');
  say('  (2) the plant already has ~90 deg of phase at crossover, so there is no phase deficit for a lead to repair; (3) the roll-off pole keeps the radar-rate noise gain (and the resonance gain) low, which a lead does the opposite of;\n');
  say('  (4) the price of the PI is the 15 %% overshoot of the loop step and a slow ~2.6 s tail (zero at -1/Ti), which the 2-DOF structure hides for the reference (exact inverse), see tc_study_feedforward_vs_feedback.\n');
  done();
  R.rows = rows;  R.names = name;  R.alpha = alpha;  R.wm = wm;  R.T = Tl;  R.K = Klead;  R.PM0 = PM0;  R.phi_needed = phi_needed;  R.kv_limit = M.a(2)*M.a(3)/(M.a(1)*M.gn)*Kvp;
  save('-mat', fullfile(rd, 'tc_study_compensator_compare.mat'), 'R');

  hf = figure('visible', 'off', 'position', [50 50 1200 450], 'color', 'w');  col = {'k', 'c', 'm', 'g', 'b'};
  subplot(1,3,1);  hold on;
  for q = 1:5
    Ln = conv(M.Gn, Cn{q});  Ld = conv(M.Gd, Cd{q});  L = polyval(Ln, s)./polyval(Ld, s);
    semilogx(w, 20*log10(abs(L)), col{q}, 'linewidth', 1.5);
  end
  set(gca, 'xscale', 'log');  semilogx(w, zeros(size(w)), 'k:');  hold off;  grid on;  axis([1e-2 1e3 -60 80]);  xlabel('\omega [rad/s]');  ylabel('|L| [dB]');  title('Loop gain');
  legend(name, 'location', 'northeast');
  subplot(1,3,2);  hold on;
  for q = 1:5
    Ln = conv(M.Gn, Cn{q});  Ld = conv(M.Gd, Cd{q});  L = polyval(Ln, s)./polyval(Ld, s);  ph = unwrap(angle(L))*180/pi;  if ph(1) > 0, ph = ph - 360; end
    semilogx(w, ph, col{q}, 'linewidth', 1.5);
  end
  set(gca, 'xscale', 'log');  semilogx(w, -180*ones(size(w)), 'k:');  hold off;  grid on;  axis([1e-2 1e3 -360 -60]);  xlabel('\omega [rad/s]');  ylabel('phase [deg]');  title('Loop phase');
  subplot(1,3,3);  hold on;  tm = t <= 6;
  for q = 1:5, if rows(q,12), plot(t(tm), Ysteps{q}(tm), col{q}, 'linewidth', 1.5); end, end
  plot([0 6], [1 1], 'k:');  hold off;  grid on;  axis([0 6 0 1.6]);  xlabel('t [s]');  ylabel('step response');  title('Closed-loop unit step (unstable loops omitted)');
  tc_save_figure(hf, 'tc_study_compensator_compare');
end

function [alpha, T, wm] = lead_design(M, Kvp, Kv_req, phi_m)
% steps 4-5 of the lecture procedure for C = K (1 + T s)/(1 + alpha T s), K = Kv_req/Kvp
  K = Kv_req/Kvp;  alpha = (1 - sind(phi_m))/(1 + sind(phi_m));
  wg = logspace(-1, 3, 20000);  gg = abs(polyval(K*M.Gn, 1j*wg)./polyval(M.Gd, 1j*wg));
  i = find(gg(1:end-1) >= sqrt(alpha) & gg(2:end) < sqrt(alpha), 1);  wm = wg(i);  T = 1/(wm*sqrt(alpha));
end

function s = ifelse(c, a, b), if c, s = a; else, s = b; end, end
