function ok = tc_test_all(quick)
% TC_TEST_ALL  Self-contained test-suite (plain Octave, no xUnit dependency).
%   tc_test_all        full suite (a few minutes)
%   tc_test_all(true)  fast subset used by CI on every push
% Exits with an error (non-zero status under `octave --eval`) if any check fails.
  here = fileparts(mfilename('fullpath'));  addpath(fileparts(here));  tc_setup();
  if nargin < 1, quick = false; end
  tc_settings('clear');
  T = {@t_plant_poles, @t_plant_saturation, @t_ff_derived, @t_quadrature, @t_los_derivatives, ...
       @t_transition_vs_vanloan, @t_polar_conversion, @t_taylor_hurwitz, @t_reference_triple_pole, ...
       @t_beam_geometry, @t_envelope_all, @t_ticks_and_blackouts, @t_noise_isolated, @t_session_reset, ...
       @t_env_clip, @t_hit_probability, @t_integrators_agree, @t_loop_margins, @t_submission_portable};
  if ~quick
    T = [T, {@t_standard_intercept, @t_rates, @t_interface_conventions, @t_lost_track_recovery, ...
             @t_determinism, @t_plant_mismatch}];
  end
  nfail = 0;
  for q = 1:numel(T)
    nm = func2str(T{q});
    try
      T{q}();  fprintf('PASS  %s\n', nm);
    catch err
      nfail = nfail + 1;  fprintf('FAIL  %s : %s\n', nm, err.message);
    end
    tc_settings('clear');
  end
  ok = (nfail == 0);
  fprintf('%d/%d checks passed\n', numel(T) - nfail, numel(T));
  if ~ok, error('tc_test_all: %d check(s) failed', nfail); end
end

function check(c, msg), if ~c, error(msg); end, end

function t_submission_portable()
  % Static guard (NOT a MATLAB run): the submission path must not use Octave-only syntax.
  root = fileparts(fileparts(mfilename('fullpath')));
  files = [{fullfile(root, 'turret_controller.m'), fullfile(root, 'turret_plant.m'), ...
            fullfile(root, 'target_motion_generator.m'), fullfile(root, 'tc_setup.m')}, ...
           cellfun(@(f) fullfile(root, 'core', f), {dir(fullfile(root, 'core', '*.m')).name}, 'UniformOutput', false)];
  bad = {};
  pats = {'\)\(', '\}\{', '\+=', '-=', '\*=', '(^|[^fs])printf\(', 'end(if|for|while|function|_try_catch)\>', ...
          'unwind_protect', '#', '(^|[^~=<>])!(?!=)'};
  for q = 1:numel(files)
    L = strsplit(fileread(files{q}), "\n");
    for k = 1:numel(L)
      ln = regexprep(L{k}, '''[^'']*''', '''''');            % drop string literals
      ln = regexprep(ln, '%.*$', '');                        % drop comments
      for p = 1:numel(pats)
        if ~isempty(regexp(ln, pats{p}, 'once')), bad{end+1} = sprintf('%s:%d (%s)', files{q}, k, pats{p}); end %#ok<AGROW>
      end
    end
  end
  check(isempty(bad), ['Octave-only syntax: ' strjoin(bad, '; ')]);
end

function t_plant_poles()
  K = tc_params();
  p = roots(K.den);
  check(abs(p(1)*0 + 1) > 0, 'sanity');
  A = [0 1 0; 0 -K.b/K.J_eq K.gn/K.J_eq; 0 -K.N*K.Ke/K.L -K.R/K.L];
  e = sort(real(eig(A)));  r = sort(real(roots(K.den)));
  check(abs(e(1) - r(1)) < 1e-6 && abs(e(2) - r(2)) < 1e-6 && abs(e(3)) < 1e-9, 'plant poles mismatch');
  check(abs(imag(roots(K.den))(1)) > 55 && abs(imag(roots(K.den))(1)) < 56, 'resonance not 55.7 rad/s');
  % numerical linearisation of turret_plant gives the same A
  clear turret_plant
  s0 = [0; 0; 0];  Jm = zeros(3);  h = 1e-3;  Vq = 0;
  for c = 1:3
    ds = zeros(3,1);  ds(c) = h;  sp = s0 + ds;  sm = s0 - ds;
    [w1, a1, i1] = turret_plant(0, Vq, sp(1), sp(2), sp(3));  [w0, a0, i0] = turret_plant(0, Vq, sm(1), sm(2), sm(3));
    Jm(:,c) = ([w1; a1; i1] - [w0; a0; i0])/(2*h);
  end
  check(norm(Jm - A, 'fro') < 1e-6*norm(A, 'fro'), 'turret_plant does not match the derived model');
end

function t_plant_saturation()
  clear turret_plant
  [~, ~, idot] = turret_plant(0, 1e4, 0, 0, 0);
  check(abs(idot - 600/0.1) < 1e-9, 'voltage clamp');
  [~, ~, idot] = turret_plant(0, -1e4, 0, 0, 0);
  check(abs(idot + 600/0.1) < 1e-9, 'negative voltage clamp');
end

function t_ff_derived()
  K = tc_params();
  check(norm(K.ff - [0.32 3.35 1001.5], 'inf') < 1e-9, 'feed-forward gains are not DEN/NUM of the plant');
  tc_settings('set', struct('V_lim', 400));
  K2 = tc_params();
  check(abs(K2.omega_cruise - 400/1001.5) < 1e-12, 'cruise rate must follow V_lim');
end

function t_quadrature()
  W = tc_quadrature_rule('legendre', 8);  check(abs(sum(W.w) - 2) < 1e-12 && abs(W.w'*W.x.^6 - 2/7) < 1e-12, 'legendre');
  H = tc_quadrature_rule('hermite', 5);   check(abs(sum(H.w) - 1) < 1e-12 && abs(H.w'*H.x.^4 - 3) < 1e-12, 'hermite');
end

function t_los_derivatives()
  % z(t) = smooth polynomial trajectory; compare against central differences
  z = @(t) complex(1500 - 100*t + 4*t.^2 - 0.3*t.^3, 600 + 20*t - 6*t.^2 + 0.5*t.^3);
  zd = @(t) complex(-100 + 8*t - 0.9*t.^2, 20 - 12*t + 1.5*t.^2);
  zdd = @(t) complex(8 - 1.8*t, -12 + 3*t);
  zddd = @(t) complex(-1.8 + 0*t, 3 + 0*t);
  t0 = 1.7;  h = 1e-3;
  x = [real(z(t0)); real(zd(t0)); real(zdd(t0)); real(zddd(t0)); imag(z(t0)); imag(zd(t0)); imag(zdd(t0)); imag(zddd(t0))];
  d = tc_los_derivatives(x);
  th = @(t) unwrap(arg(z(t)));
  f1 = (th(t0+h) - th(t0-h))/(2*h);
  f2 = (th(t0+h) - 2*th(t0) + th(t0-h))/h^2;
  f3 = (th(t0+2*h) - 2*th(t0+h) + 2*th(t0-h) - th(t0-2*h))/(2*h^3);
  check(abs(d(2) - f1) < 1e-8, 'theta''');
  check(abs(d(3) - f2) < 1e-6, 'theta''''');
  check(abs(d(4) - f3) < 5e-5*max(1, abs(f3)) + 1e-6, 'theta''''''');
end

function t_transition_vs_vanloan()
  alpha = 2.5;  q = 2000;  tau = 0.37;
  W = tc_quadrature_rule('legendre', 8);
  [F, Q] = tc_jerk_model_transition(tau, alpha, q, W);
  A = [0 1 0 0; 0 0 1 0; 0 0 0 1; 0 0 0 -alpha];  Qc = zeros(4);  Qc(4,4) = q;
  M = expm([-A, Qc; zeros(4), A']*tau);
  Fv = M(5:8,5:8)';  Qv = Fv*M(1:4,5:8);
  check(norm(F - Fv, 'fro') < 1e-10*norm(Fv, 'fro'), 'F mismatch');
  check(norm(Q - Qv, 'fro') < 1e-9*norm(Qv, 'fro'), 'Q mismatch');
  F0 = tc_jerk_model_transition(1e-9, alpha, q, W);  check(all(isfinite(F0(:))), 'tiny tau');
end

function t_polar_conversion()
  % Monte-Carlo check: converted measurement is unbiased and its covariance matches Rz
  K = tc_params();  r = 1600;  th = 0.5;  n = 40000;
  u = tc_uniform_draws(7, 4*n);  z1 = sqrt(-2*log(u(1:n))).*cos(2*pi*u(n+1:2*n));  z2 = sqrt(-2*log(u(2*n+1:3*n))).*cos(2*pi*u(3*n+1:4*n));
  rm = r + K.sig_r*z1;  tm = th + K.sig_th*z2;
  Z = zeros(n, 2);  for q = 1:2000, Z(q,:) = tc_convert_measurement(rm(q), tm(q), K.sig_r, K.sig_th)'; end
  Z = Z(1:2000,:);  [~, Rz] = tc_convert_measurement(r, th, K.sig_r, K.sig_th);
  truth = r*[cos(th) sin(th)];
  check(norm(mean(Z) - truth) < 4*sqrt(max(diag(Rz))/2000), 'converted measurement biased');
  check(abs(trace(cov(Z))/trace(Rz) - 1) < 0.15, 'converted covariance off');
end

function t_taylor_hurwitz()
  tp = 0.5;
  for n = 1:5
    c = tp.^(0:n)./factorial(0:n);  rt = roots(fliplr(c));
    stable = all(real(rt) < 0);
    check(stable == (n <= 4), sprintf('Taylor section %d Hurwitz status wrong', n));
  end
end

function t_reference_triple_pole()
  % The deviation obeys (D+p)^3 d = 0 exactly inside the linear zone.
  K = tc_params();  S.cfg = K;  S.dev = [0.01; -0.02; 0.03];  S.Phi_dt = NaN;  S.Phi = eye(3);
  D = [0.3; 0.02; 0.001; 0];
  dt = 0.013;  S2 = tc_reference_advance(S, D, dt);
  p = K.p_dev;  A = [0 1 0; 0 0 1; -p^3 -3*p^2 -3*p];
  check(norm(S2.dev - expm(A*dt)*S.dev) < 1e-12, 'linear advance is not the triple-pole transition');
  % Forward-Euler sub-stepping of the same law must CONVERGE to it at first order (error / 10
  % when the step is / 10): that is what the acquisition-zone sub-stepping relies on.
  e = zeros(1, 2);  ns = [200 2000];
  for j = 1:2
    d = S.dev;  h = dt/ns(j);
    for q = 1:ns(j), d3 = -3*p*d(3) - 3*p^2*d(2) - p^3*d(1); d(3) += d3*h; d(2) += d(3)*h; d(1) += d(2)*h; end
    e(j) = norm(d - S2.dev);
  end
  check(e(2) < 2e-4 && abs(e(1)/e(2) - 10) < 1.5, sprintf('Euler cascade does not converge at O(h): %.2e %.2e', e(1), e(2)));
end

function t_beam_geometry()
  [m, s] = tc_beam_miss_distance(0, 100, 3);   check(abs(m - 3) < 1e-12 && abs(s - 3) < 1e-12, 'ahead');
  [m, s] = tc_beam_miss_distance(0, -100, 3);  check(abs(m - hypot(100, 3)) < 1e-12, 'behind');
  [m, ~] = tc_beam_miss_distance(pi/4, 100, 100);  check(m < 1e-12, 'on beam');
end

function t_envelope_all()
  for nm = {'A','B','C','D1','D2','D3','D4','D5','D6','MC1','MC2','MC3','MC5000','MC5001','HB1','HB2','HB3','HB5000'}
    e = tc_scenario_build(nm{1});  w = e.tg <= 10 + 1e-9;
    p = e.truth(1:2,w);  v = e.truth(3:4,w);  a = e.truth(5:6,w);
    check(min(hypot(p(1,:), p(2,:))) >= 800 && max(hypot(p(1,:), p(2,:))) <= 2000, [nm{1} ' range']);
    check(max(hypot(v(1,:), v(2,:))) <= 150 && max(hypot(a(1,:), a(2,:))) <= 20 + 1e-9, [nm{1} ' v/a']);
  end
  e = tc_scenario_build('D6');  a = e.truth(5:6,:);  jerk = max(abs(diff(a(2,:))))/1e-3;
  check(jerk > 400 && jerk < 500, sprintf('D6 jerk %.0f not ~450 m/s^3', jerk));
end

function t_ticks_and_blackouts()
  [r, th, v] = target_motion_generator(0.05, 'C');   check(v == 1 && isfinite(r) && isfinite(th), 'tick');
  [~, ~, v] = target_motion_generator(0.0537, 'C');  check(v == 0, 'between ticks');
  [~, ~, v] = target_motion_generator(1.0, 'C');     check(v == 0, 'blackout onset');
  [~, ~, v] = target_motion_generator(1.45, 'C');    check(v == 0, 'inside blackout');
  [~, ~, v] = target_motion_generator(1.5, 'C');     check(v == 1, 'blackout end');
  % noise level of the radar
  K = tc_params();  er = zeros(1, 180);  et = er;
  for k = 1:180
    t = 0.05*k; if t >= 1 && t < 1.5, continue; end
    [r, th, v, tru] = target_motion_generator(t, 'A');
    if v, er(k) = r - hypot(tru(1), tru(2));  et(k) = th - atan2(tru(2), tru(1)); end
  end
  er = er(er ~= 0);  et = et(et ~= 0);
  check(abs(std(er)/K.sig_r - 1) < 0.2 && abs(std(et)/K.sig_th - 1) < 0.2, 'radar noise level');
end

function t_noise_isolated()
  rand('twister', 123);  a = rand(1, 3);  rand('twister', 123);
  target_motion_generator(0.05, 'MC77');
  b = rand(1, 3);
  check(isequal(a, b), 'generator consumed the caller''s random stream');
end

function t_session_reset()
  clear turret_controller
  [V1, f1] = turret_controller(0, NaN, NaN, 0, 0, 0);
  for k = 1:50, turret_controller(0.01*k, NaN, NaN, 0, 0, 0); end
  [V2, f2] = turret_controller(1e-13, NaN, NaN, 0, 0, 0);       % ~0 after earlier calls: new run
  check(V1 == V2 && f1 == f2, 'tolerance-based t = 0 reset failed');
  [V3, ~] = turret_controller(0.3, NaN, NaN, 0, 0, 0);          % decreasing t: reset
  [V4, ~] = turret_controller(0.2, NaN, NaN, 0, 0, 0);
  check(isfinite(V3) && isfinite(V4), 'decreasing t');
  clear turret_controller
end

function t_env_clip()
  K = tc_params();  x = zeros(8,1);  x([2 6]) = [300 0];  x([3 7]) = [0 90];  x([4 8]) = [1000 0];
  y = tc_envelope_clip(x, K);
  check(abs(hypot(y(2), y(6)) - K.env_v) < 1e-9 && abs(hypot(y(3), y(7)) - K.env_a) < 1e-9 && abs(hypot(y(4), y(8)) - K.env_j) < 1e-9, 'clip');
end

function t_hit_probability()
  K = tc_params();  S.cfg = K;
  Wg = tc_quadrature_rule('hermite', 5);  [Z1, Z2] = meshgrid(Wg.x, Wg.x);
  S.Zgh = [Z1(:)'; Z2(:)'];  S.Wgh = (Wg.w*Wg.w')(:)';
  x = zeros(8,1);  x(1) = 1500;  x(5) = 500;  x(2) = -100;  P = diag([1 100 4 100 1 100 4 100]);
  th = atan2(500, 1500);
  [p0, ~, c0] = tc_hit_probability(S, x, P, th);        % beam on target
  [p1, ~] = tc_hit_probability(S, x, P, th + 3/1581);   % beam ~3 m off
  check(p0 > 0.9 && p1 < p0 && p1 >= 0 && c0 > 0.5, 'hit probability monotone in offset');
  [p2, ~] = tc_hit_probability(S, x, P, th + pi);        % pointing away
  check(p2 == 0, 'target behind beam must have P = 0');
end

function t_integrators_agree()
  % The exact ZOH discretisation is the reference; RK4 is 4th order, so halving h must cut the
  % difference by ~16 and 1 ms must already be within 1e-5 (relative) over 0.2 s of a 3 Hz drive.
  df = zeros(1, 3);  hs = [1e-3 5e-4 2.5e-4];
  for j = 1:3
    h = hs(j);  n = round(0.2/h);  M = tc_plant_model();  Mr = M;  se = [0.1; 0; 0];  sr = se;
    for q = 1:n
      V = 300*sin(2*pi*3.18*floor((q-1)*h/1e-3)*1e-3);        % same held voltage in both
      [se, M]  = tc_plant_step(M,  se, V, h, 'exact');
      [sr, Mr] = tc_plant_step(Mr, sr, V, h, 'rk4');
    end
    df(j) = norm(se - sr);
  end
  check(df(1) < 1e-5*norm(se), sprintf('rk4 vs exact at 1 ms: %.2e', df(1)));
  check(abs(log2(df(1)/df(2)) - 4) < 0.6 && abs(log2(df(2)/df(3)) - 4) < 0.6, 'rk4 is not converging at 4th order');
  % and the exact path must really propagate (guards the NaN-cache bug that froze the state)
  check(norm(se - [0.1; 0; 0]) > 1e-3, 'exact integrator did not move the state');
end

function t_loop_margins()
  % Frequency-domain check of the designed loop (continuous): PM > 55 deg, GM > 10 dB, wc ~ 4 rad/s.
  K = tc_params();  w = logspace(-1, 3, 4000);  s = 1j*w;
  Gp = K.gn ./ (s .* (K.den(1)*s.^2 + K.den(2)*s + K.den(3)));
  C = K.kp*(1 + 1./(K.ti*s))./(K.tau_f*s + 1);
  Lg = Gp.*C;  mag = abs(Lg);
  i = find(mag(1:end-1) >= 1 & mag(2:end) < 1, 1);
  pm = 180 + angle(Lg(i))*180/pi;
  ph = unwrap(angle(Lg)) * 180/pi;  j = find(ph(1:end-1) > -180 & ph(2:end) <= -180, 1);
  gm = -20*log10(mag(j));
  check(w(i) > 3 && w(i) < 6 && pm > 55 && gm > 10, sprintf('margins wc=%.2f pm=%.1f gm=%.1f', w(i), pm, gm));
end

function t_standard_intercept()
  for nm = {'A','B','C'}
    for f = [20 100 1000]
      o = tc_run_engagement(nm{1}, struct('f_ctrl', f));  m = o.metrics;
      check(m.success && m.t_first_hit < 2.5 && m.V_peak <= 600 + 1e-9, sprintf('%s @%d Hz: hit=%d t=%.2f', nm{1}, f, m.success, m.t_first_hit));
    end
  end
end

function t_rates()
  for f = [50 400 2000]   % integer multiples of the 20 Hz radar, as the spec's ticks require
    o = tc_run_engagement('B', struct('f_ctrl', f, 't_end', 5));
    check(o.metrics.success, sprintf('rate %d', f));
  end
end

function t_interface_conventions()
  for cv = {'held', 'drift', 'jitter'}
    o = tc_run_engagement('B', struct('convention', cv{1}, 'f_ctrl', 100));
    check(o.metrics.success && o.metrics.point_rms_m < 5, ['convention ' cv{1}]);
  end
  o = tc_run_engagement('A', struct('theta0', 60*pi/180));
  check(o.metrics.success, 'turret starting at 60 deg');
end

function t_lost_track_recovery()
  % Data stops entirely for 2 s (far outside the spec): the controller must not diverge and
  % must re-acquire when data returns.
  clear turret_controller;  K = tc_params();  s = [0; 0; 0];  M = tc_plant_model();  worst = 0;
  for k = 0:1000
    t = k*0.01;  [r, th, v] = target_motion_generator(t, 'A');
    if t > 2 && t < 4.2, v = 0; r = NaN; th = NaN; end
    V = turret_controller(t, r, th, v, s(1), s(2));  worst = max(worst, abs(V));
    [s, M] = tc_plant_step(M, s, V, 0.01, 'exact');
  end
  [~, ~, ~, tru] = target_motion_generator(10, 'A');
  check(worst <= 600 + 1e-9 && abs(s(1) - atan2(tru(2), tru(1))) < 0.01, 'no recovery after long data loss');
end

function t_determinism()
  a = tc_run_engagement('D2');  b = tc_run_engagement('D2');
  check(isequal(a.log.V, b.log.V) && isequal(a.pulses, b.pulses), 'back-to-back runs differ');
end

function t_plant_mismatch()
  o = tc_run_engagement('B', struct('plant_scale', struct('Kt', 1.2, 'Ke', 0.85, 'J', 1.2, 'b', 0.8, 'R', 1.2, 'L', 0.8)));
  check(o.metrics.success, 'plant mismatch intercept');
end
