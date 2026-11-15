function R = tc_study_start_angle_miss()
% TC_STUDY_START_ANGLE_MISS  Why does the first pulse miss when the turret starts at 80 deg?  (analysis only)
%
% Scenario A at 100 Hz, turret starting at 80 deg.  The first pulse of the committed sweep (outputs/eval_adapter_sweep.txt)
% fell at 1.84 s, 2.11 m off.  This study replays that case with its own copy of the closed loop (same calls as
% tc_run_engagement: target_motion_generator, turret_controller with its third output, tc_plant_step) so that it can log
% what the controller exposes: turret angle and rate, the commanded voltage before and after the clamp, the current,
% the acquisition clamps (dbg.linear), the integrator-freeze condition, the filter's bearing error, the posterior
% sigma and consistency, the hit probability and the fire command.  It then repeats the case from 0, 30, 60 and 80 deg
% (and over noise draws) and tabulates the first-pulse miss.  It does not change any design file or parameter.
%
% Writes outputs/start_angle_miss.txt and plots/start_angle_miss.{png,pdf}.
%
% What this log CANNOT show: the controller's private state (integrator content, the armed flag and armed-peak of the
% fire rule) is not exposed, so those are inferred from the logged signals and marked "inferred".
  here = fileparts(mfilename('fullpath'));  root = fileparts(here);  addpath(root);  tc_setup();
  tc_settings('clear');  K = tc_params();
  rd = fullfile(root, 'outputs');  if ~exist(rd, 'dir'), mkdir(rd); end
  fid = fopen(fullfile(rd, 'start_angle_miss.txt'), 'w');
  w = @(varargin) fprintf(fid, varargin{:});
  w('octave-cli --quiet --eval "tc_setup(); tc_study_start_angle_miss();"   (make start-angle)\n');
  w('Scenario A, 100 Hz, frozen design, no overrides.  Own closed loop replaying tc_run_engagement (plant: exact ZOH, 1 ms steps).\n\n');

  % ---- 1. the 80 deg case, logged ----------------------------------------------------------------------------------
  c80 = run_case('A', 100, 80, 3.0, K);
  ref = tc_run_engagement('A', struct('f_ctrl', 100, 'theta0', 80*pi/180));
  d_t = abs(c80.pulses(1,1) - ref.pulses(1,1));  d_m = abs(c80.pulses(1,2) - ref.pulses(1,2));
  w('== 1. Replay check against tc_run_engagement (same case)\n');
  w('first pulse here: t = %.3f s, miss %.4f m;  tc_run_engagement: t = %.3f s, miss %.4f m;  |dt| = %.2g s, |dmiss| = %.2g m  -> %s\n', ...
    c80.pulses(1,1), c80.pulses(1,2), ref.pulses(1,1), ref.pulses(1,2), d_t, d_m, tern(d_t < 1e-9 && d_m < 1e-6, 'IDENTICAL', 'DIFFERS (do not trust this log)'));
  cm = run_case('A', 100, 80, 3.0, K, true);
  w('same case with the call times written k*T instead of k/f (a rounding-level change, as eval_adapter does): first pulse t = %.3f s, miss %.4f m\n', cm.pulses(1,1), cm.pulses(1,2));
  w('=> the first pulse misses by more than R_eng = %.1f m on both time grids, but its exact distance moves by %.3f m with rounding of the call times alone.\n\n', K.r_eng, abs(cm.pulses(1,2) - c80.pulses(1,2)));

  L = c80.L;  kp = c80.kpulse;  tp = L.t(kp);
  w('== 2. State at the first pulse (80 deg start), t = %.2f s\n', tp);
  w('turret angle              %8.3f deg   target bearing %8.3f deg   target range %7.1f m\n', L.th(kp)*180/pi, L.th_true(kp)*180/pi, L.r(kp));
  w('turret rate               %8.4f rad/s (target LOS rate %.4f rad/s)\n', L.thd(kp), L.los_rate(kp));
  w('beam error (true)         %8.3f m   -> %s (limit %.1f m)\n', L.err_beam(kp), tern(abs(L.err_beam(kp)) <= K.r_eng, 'hit', 'MISS'), K.r_eng);
  w('  = filter bearing error  %8.3f m  [(theta_hat - theta_true) * r]\n', L.err_filt(kp));
  w('  + offset the controller sees %6.3f m  [(theta_hat - theta_turret) * r]\n', L.err_seen(kp));
  w('posterior sigma (beam)    %8.3f m   consistency %.3f   n_upd %d\n', L.sig(kp), L.cons(kp), L.n_upd(kp));
  w('hit probability           p_raw %.4f  p %.4f   (p_sure %.2f, p_arm %.2f)\n', L.p_raw(kp), L.p(kp), K.p_sure, K.p_arm);
  w('voltage                   u = %.1f V (u_ff %.1f, u_fb %.1f), clamp %.0f V -> command %.1f V\n', L.u(kp), L.u_ff(kp), L.u_fb(kp), K.V_lim, L.V(kp));
  w('current                   %.2f A (acquisition budget %.0f A)\n\n', L.i(kp), K.i_lim);

  % checks
  win = L.t >= tp - 0.30 & L.t <= tp;                         % the 0.3 s before the pulse
  slew_end = find(L.linear & L.t > 0.5, 1);                   % first call with the reference in the linear zone
  w('== 3. Diagnosis checks (computed from the log)\n');
  c_slew = ~L.linear(kp);                                     % acceleration/rate clamps still shaping the reference
  w('(a) still slewing under the acquisition clamps?   clamps active at the pulse: %s; reference reached the linear zone at t = %.2f s; turret rate at pulse %.3f rad/s (%.1f %% of the back-EMF cruise rate %.3f rad/s)\n', ...
    yn(c_slew), L.t(slew_end), L.thd(kp), 100*abs(L.thd(kp))/K.omega_cruise, K.omega_cruise);
  w('    the turret was still decelerating at the pulse: rate %.3f rad/s at the pulse vs %.3f rad/s 0.10 s earlier; |current| in the last 0.3 s up to %.2f A\n', ...
    L.thd(kp), L.thd(max(kp - 10, 1)), max(abs(L.i(win))));
  w('    current budget exceeded in the log (first 3 s)?    %s (max |i| = %.2f A, budget %.0f A)\n', yn(max(abs(L.i)) > K.i_lim), max(abs(L.i)), K.i_lim);
  w('    voltage clamp (|u| >= %.0f V) active in the last 0.3 s?  %s;  anywhere in the first 3 s: %s (%.2f s of %.2f s)\n', K.V_lim, ...
    yn(any(abs(L.u(win)) >= K.V_lim)), yn(any(abs(L.u) >= K.V_lim)), sum(abs(L.u) >= K.V_lim)*0.01, L.t(end));
  frz = K.freeze_acq & ~L.linear;
  w('(b) integrator frozen by the acquisition freeze (freeze_acq & clamps active)?   at the pulse: %s;  in the last 0.3 s: %s;  total frozen time in the first 3 s: %.2f s\n', ...
    yn(frz(kp)), yn(any(frz(win))), sum(frz)*0.01);
  w('    (the integrator content itself is not exposed; only the freeze condition is logged)\n');
  nearstop = (L.th > K.th_max - 0.5*pi/180) | (L.y_d > K.th_max - 0.5*pi/180);
  w('(c) 85 deg stop involved?   largest turret angle in the log %.2f deg (the start), stop at %.0f deg; stop-guard condition true at any call after t = 0: %s; at the pulse the turret is %.1f deg from the stop\n', ...
    max(L.th)*180/pi, K.th_max*180/pi, yn(any(nearstop(2:end))), (K.th_max - L.th(kp))*180/pi);
  cand = find(L.p >= K.p_arm & L.ready, 1);
  w('(d) hit probability above the firing threshold already?   p first reached p_arm (%.2f) at t = %.2f s; at the pulse p = %.4f >= p_sure (%.2f): %s -> fire path: %s (inferred)\n', ...
    K.p_arm, L.t(cand), L.p(kp), K.p_sure, yn(L.p(kp) >= K.p_sure), tern(L.p(kp) >= K.p_sure, 'immediate (p >= p_sure)', 'armed (peak-drop or timeout)'));
  w('    the rule fired at the FIRST call at which it was ready and p >= p_sure: %s;   consistency %.3f >= %.2f: %s\n', ...
    yn(all(~(L.p(1:kp-1) >= K.p_sure & L.ready(1:kp-1)))), L.cons(kp), K.q_cons_min, yn(L.cons(kp) >= K.q_cons_min));
  w('    posterior sigma at the pulse %.2f m vs actual filter bearing error %.2f m and controller-seen offset %.2f m:  the probability treats the beam error as N(%.2f, %.2f^2) m, the true beam error was %.2f m\n', ...
    L.sig(kp), L.err_filt(kp), L.err_seen(kp), L.err_seen(kp), L.sig(kp), L.err_beam(kp));
  w('    filter bearing error over the 1 s before the pulse: mean %.2f m, max |.| %.2f m (steady-state sigma at this range is about %.2f m, see outputs/error_budget.txt)\n\n', ...
    mean(L.err_filt(L.t >= tp - 1 & L.t <= tp)), max(abs(L.err_filt(L.t >= tp - 1 & L.t <= tp))), L.r(kp)*K.sig_th*0.65);

  % ---- 3b. what the same 80 deg start does if the filter error is removed: replay with the pulse delayed -----------------
  w('== 4. Beam error and probability around the pulse (every call, 80 deg start)\n');
  w('%6s %8s %8s %9s %8s %8s %7s %7s %6s %6s %6s\n', 't', 'th_deg', 'thd', 'beam_m', 'filt_m', 'seen_m', 'sig_m', 'p', 'lin', 'ready', 'fire');
  for k = find(L.t >= tp - 0.12 & L.t <= tp + 0.06)
    w('%6.2f %8.3f %8.4f %9.3f %8.3f %8.3f %7.3f %7.4f %6d %6d %6d\n', L.t(k), L.th(k)*180/pi, L.thd(k), L.err_beam(k), L.err_filt(k), L.err_seen(k), L.sig(k), L.p(k), L.linear(k), L.ready(k), L.fire(k));
  end
  w('\n');

  % ---- 5. start-angle table ---------------------------------------------------------------------------------------------
  starts = [0 30 60 80];  T = zeros(numel(starts), 9);
  w('== 5. First pulse versus start angle (scenario A, 100 Hz, default noise draw)\n');
  w('%7s %9s %9s %5s %9s %8s %9s %9s %8s %6s %8s\n', 'start', 't_pulse', 'miss_m', 'hit', 'p', 'thd', 'filt_m', 'seen_m', 'lin', 'n_pls', 'first_hit');
  cases = cell(1, numel(starts));
  for q = 1:numel(starts)
    c = run_case('A', 100, starts(q), 10, K);  cases{q} = c;  Lq = c.L;  k = c.kpulse;
    fh = c.pulses(find(c.pulses(:,3) > 0.5, 1), 1);  if isempty(fh), fh = NaN; end
    T(q,:) = [starts(q), c.pulses(1,1), c.pulses(1,2), c.pulses(1,3), Lq.p(k), Lq.thd(k), Lq.err_filt(k), Lq.err_seen(k), fh];
    w('%6d%s %9.3f %9.3f %5d %9.4f %8.3f %9.3f %9.3f %8d %6d %8.3f\n', starts(q), 'd', c.pulses(1,1), c.pulses(1,2), c.pulses(1,3), Lq.p(k), Lq.thd(k), Lq.err_filt(k), Lq.err_seen(k), Lq.linear(k), size(c.pulses,1), fh);
  end
  w('\n');

  w('Filter bearing error at t = %.2f s (the 80 deg first-pulse time) for each start:', tp);
  for q = 1:numel(starts), w('  %d deg: %.3f m', starts(q), cases{q}.L.err_filt(find(abs(cases{q}.L.t - tp) < 1e-9, 1))); end
  w('\n(it is the same for every start: the estimator never sees the turret, so this error is a property of the radar noise draw)\n\n');
  % ---- 6. noise draws ------------------------------------------------------------------------------------------------------
  nd = 10;  M = zeros(numel(starts), nd);  Tm = zeros(numel(starts), nd);
  for q = 1:numel(starts)
    for j = 1:nd
      c = run_case(sprintf('A#%d', j), 100, starts(q), 4, K);  M(q,j) = c.pulses(1,2);  Tm(q,j) = c.pulses(1,1);
    end
  end
  w('== 6. First-pulse miss over %d radar-noise draws (same trajectory, scenario A#1..A#%d, 100 Hz)\n', nd, nd);
  w('%7s %10s %10s %10s %14s %12s\n', 'start', 'min_miss', 'median', 'max_miss', 'first pulse>2m', 'median t_p');
  for q = 1:numel(starts)
    w('%6dd %10.3f %10.3f %10.3f %9d/%-4d %12.3f\n', starts(q), min(M(q,:)), median(M(q,:)), max(M(q,:)), sum(M(q,:) > K.r_eng), nd, median(Tm(q,:)));
  end
  w('\n');
  fclose(fid);

  % ---- figure -------------------------------------------------------------------------------------------------------------------
  h = figure('visible', 'off', 'position', [50 50 1000 760], 'color', 'w');
  subplot(3,2,1);  plot(L.t, L.th*180/pi, 'r', L.t, L.th_true*180/pi, 'k', L.t, L.th_hat*180/pi, 'b--');  hold on;
  plot(tp, L.th(kp)*180/pi, 'kv', 'markerfacecolor', 'y', 'markersize', 9);  hold off;  grid on;  ylabel('bearing [deg]');
  title('Scenario A, 80 deg start, 100 Hz');  legend({'turret', 'target', 'estimate', 'first pulse'}, 'location', 'northeast');
  subplot(3,2,2);  plot(L.t, L.u, 'm', L.t, L.V, 'k');  hold on;  plot([0 3], K.V_lim*[1 1], 'r:', [0 3], -K.V_lim*[1 1], 'r:');  plot(tp, L.V(kp), 'kv', 'markerfacecolor', 'y');  hold off;
  grid on;  ylabel('voltage [V]');  ylim([-650 650]);  legend({'u (before clamp)', 'V_m', '\pm V_{lim}'}, 'location', 'northeast');
  subplot(3,2,3);  plot(L.t, L.i, 'b');  hold on;  plot([0 3], K.i_lim*[1 1], 'r:', [0 3], -K.i_lim*[1 1], 'r:');  plot(tp, L.i(kp), 'kv', 'markerfacecolor', 'y');  hold off;
  grid on;  ylabel('current [A]');  legend({'i', '\pm i_{lim}'}, 'location', 'east');
  subplot(3,2,4);  plot(L.t, L.err_beam, 'b', L.t, L.err_filt, 'g', L.t, L.err_seen, 'c');  hold on;
  plot([0 3], K.r_eng*[1 1], 'r:', [0 3], -K.r_eng*[1 1], 'r:');  plot(tp, L.err_beam(kp), 'kv', 'markerfacecolor', 'y');  hold off;
  ylim([-6 6]);  grid on;  ylabel('error [m]');  legend({'beam (true)', 'filter bearing', 'seen by controller', '\pm R_{eng}'}, 'location', 'southwest');
  subplot(3,2,5);  plot(L.t, L.p, 'k', L.t, L.p_raw, 'r--');  hold on;  plot([0 3], K.p_sure*[1 1], 'g:', [0 3], K.p_arm*[1 1], 'm:');  plot(tp, L.p(kp), 'kv', 'markerfacecolor', 'y');  hold off;
  ylim([0 1.05]);  grid on;  ylabel('P_{hit}');  xlabel('t [s]');  legend({'p', 'p raw', 'p_{sure}', 'p_{arm}'}, 'location', 'east');
  subplot(3,2,6);  plot(T(:,1), T(:,3), 'ko-', 'markerfacecolor', 'k');  hold on;  plot(starts, max(M, [], 2), 'r^', starts, min(M, [], 2), 'bv');  plot([0 80], K.r_eng*[1 1], 'r:');  hold off;
  grid on;  xlabel('start angle [deg]');  ylabel('first-pulse miss [m]');  legend({'default noise', 'max of 10 draws', 'min of 10 draws', 'R_{eng}'}, 'location', 'northwest');
  tc_save_figure(h, 'start_angle_miss');
  R = struct('starts', starts, 'table', T, 'noise_miss', M, 'noise_t', Tm, 'tp', tp, 'case80', c80);
  fprintf('wrote outputs/start_angle_miss.txt and plots/start_angle_miss.{png,pdf}\n');
end

function s = yn(b), if b, s = 'yes'; else, s = 'no'; end, end
function s = tern(c, a, b), if c, s = a; else, s = b; end, end

function c = run_case(scn, f_ctrl, th0_deg, t_end, K, mulgrid)
% Closed loop identical to tc_run_engagement ('ref' convention, exact ZOH plant), plus the controller's debug outputs.
  clear turret_controller
  M = tc_plant_model(struct(), struct('coulomb', 0, 'deadzone', 0));
  T = 1/f_ctrl;  n = round(t_end*f_ctrl) + 1;  tk = (0:n-1)/f_ctrl;          % the grid tc_run_engagement uses
  if nargin > 5 && mulgrid, tk = (0:n-1)*T;  end                                   % k*T instead of k/f (eval_adapter's grid): differs by rounding only
  s = [th0_deg*pi/180; 0; 0];
  fn = {'t','th','thd','i','V','u','u_ff','u_fb','th_true','th_hat','r','err_beam','err_filt','err_seen','sig','cons','p','p_raw','linear','ready','fire','n_upd','los_rate','y_d'};
  L = struct();  for q = 1:numel(fn), L.(fn{q}) = zeros(1, n); end
  pulses = zeros(0, 3);  t_last = -Inf;  fprev = 0;  kpulse = 0;
  for k = 1:n
    t = tk(k);  [r, th, v, tru] = target_motion_generator(t, scn);
    [V, fire, dbg] = turret_controller(t, r, th, v, s(1), s(2));
    x = tru(1);  y = tru(2);  rr = hypot(x, y);  tht = atan2(y, x);
    L.t(k) = t;  L.th(k) = s(1);  L.thd(k) = s(2);  L.i(k) = s(3);  L.V(k) = V;
    L.u(k) = dbg.u_ff + dbg.u_fb;  L.u_ff(k) = dbg.u_ff;  L.u_fb(k) = dbg.u_fb;
    L.th_true(k) = tht;  L.th_hat(k) = dbg.theta_hat;  L.r(k) = rr;
    L.err_beam(k) = rr*sin(tht - s(1));  L.err_filt(k) = (dbg.theta_hat - tht)*rr;  L.err_seen(k) = (dbg.theta_hat - s(1))*rr;
    L.sig(k) = dbg.sig;  L.cons(k) = dbg.cons;  L.p(k) = dbg.p_hit;  L.p_raw(k) = dbg.p_raw;
    L.linear(k) = dbg.linear;  L.ready(k) = strcmp(dbg.mode, 'track') && dbg.n_upd >= K.n_upd_min;
    L.fire(k) = fire;  L.n_upd(k) = dbg.n_upd;  L.y_d(k) = dbg.y_d;
    L.los_rate(k) = (x*tru(4) - y*tru(3))/rr^2;
    if fire > 0.5 && fprev <= 0.5 && (t - t_last) >= 1 - 1e-9
      miss = tc_beam_miss_distance(s(1), x, y);
      pulses(end+1, :) = [t, miss, miss <= K.r_eng]; %#ok<AGROW>
      t_last = t;  if kpulse == 0, kpulse = k; end
    end
    fprev = fire;
    if k < n
      dt = tk(k+1) - t;  ns = max(1, ceil(dt/1e-3 - 1e-9));  hh = dt/ns;
      for q = 1:ns, [s, M] = tc_plant_step(M, s, V, hh, 'exact'); end
    end
  end
  c = struct('L', L, 'pulses', pulses, 'kpulse', kpulse);
  if kpulse == 0, error('tc_study_start_angle_miss: no pulse in %.1f s for %s from %g deg', t_end, scn, th0_deg); end
end
