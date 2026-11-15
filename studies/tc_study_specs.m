function R = tc_study_specs()
% TC_STUDY_SPECS  Closed-loop step specifications of the final loop from the polynomial model, against
%   (a) dominant-pole estimates and (b) requirements.
%   Model: T(s) = Ln/(Ln + Ld), the feedback loop alone (as in outputs/analysis.txt).
%   Requirements: the handout asks for "satisfactory" stability and transient response and gives no numbers,
%   so the targets below are OUR ASSUMED, conventional design targets, fixed before running (overshoot <= 20 %,
%   10-90 % rise <= 0.5 s, 2 % settling <= 3 s; no peak-time target).
%   Estimates:  E1 two-pole model of the two slowest closed-loop poles (its own exact step response);
%               E2 zeta = PM/100 and wn from the bandwidth with the standard second-order formulas;
%               E3 the two slowest poles plus the closed-loop zero at -1/Ti (what E1 ignores).
%   Writes outputs/tc_study_specs.{txt,mat}, plots/tc_study_specs.{pdf,png}.
  here = fileparts(mfilename('fullpath'));  addpath(fileparts(here));  tc_setup();
  M = tc_course_model();  K = M.K;  rd = fullfile(fileparts(here), 'outputs');
  [say, done] = tc_course_report('tc_study_specs');
  t = 0:0.0005:15;  y = tc_lin_response(M.Ln, M.cl, 'step', 1, t);
  sp = tc_step_specs(t, y);
  pl = roots(M.cl);  [~, ord] = sort(abs(real(pl)));  pl = pl(ord);
  say('Closed-loop poles (slowest first): %s\n', mat2str(pl.', 5));
  say('Closed-loop zeros of T = Ln/cl: %s  (the PI zero -1/Ti = %.1f rad/s stays in T)\n', mat2str(roots(M.Ln).', 4), -1/K.ti);
  % ---- E1 -----------------------------------------------------------------------------------------------
  p1 = -real(pl(1));  p2 = -real(pl(2));  wn1 = sqrt(p1*p2);  z1 = (p1 + p2)/(2*wn1);
  y1 = tc_lin_response(p1*p2, conv([1 p1], [1 p2]), 'step', 1, t);  e1 = tc_step_specs(t, y1);
  % ---- E2 -----------------------------------------------------------------------------------------------
  mg = tc_loop_margins(M.Ln, M.Ld);
  w = logspace(-2, 3, 20000);  s = 1j*w;  Lw = polyval(M.Ln, s)./polyval(M.Ld, s);  Tw = abs(Lw./(1 + Lw));
  ib = find(Tw < 1/sqrt(2) & w > mg.wc/3, 1);  wb = w(ib);
  z2 = mg.pm/100;  fb = sqrt(1 - 2*z2^2 + sqrt(4*z2^4 - 4*z2^2 + 2));  wn2 = wb/fb;
  e2 = struct('tr', 1.8/wn2, 'tp', pi/(wn2*sqrt(1 - z2^2)), 'os', 100*exp(-pi*z2/sqrt(1 - z2^2)), 'ts', 4/(z2*wn2));
  % ---- E3 -----------------------------------------------------------------------------------------------
  y3 = tc_lin_response(K.ti*p1*p2*[1 1/K.ti], conv([1 p1], [1 p2]), 'step', 1, t);  e3 = tc_step_specs(t, y3);
  say('\nPolynomial-model step response (%.0f s, 0.5 ms grid): final value %.4f (type 2, so 1)\n', t(end), y(end));
  say('  Estimates\n');
  say('   E1 two slowest poles: p = %.4f, %.4f -> wn = %.3f rad/s, zeta = %.3f  (%s: standard 2nd-order formulas do not apply; the exact step of that two-pole model is used)\n', p1, p2, wn1, z1, ifelse(z1 >= 1, 'overdamped', 'underdamped'));
  say('   E2 PM/bandwidth rule: zeta = PM/100 = %.3f, wb = %.2f rad/s -> wn = %.3f rad/s; tr = 1.8/wn, tp = pi/(wn sqrt(1-z^2)), Mp = exp(-pi z/sqrt(1-z^2)), ts = 4/(z wn)\n', z2, wb, wn2);
  say('   E3 E1 plus the closed-loop zero at %.1f rad/s (unit DC gain)\n\n', -1/K.ti);
  req = struct('os', 20, 'tr', 0.5, 'ts', 3.0);
  say('Step specifications (feedback loop alone)\n');
  say('  %-22s %12s | %12s %8s | %12s %8s | %12s %8s | %s\n', 'metric', 'polynomial', 'E1 2 poles', 'dev %', 'E2 PM rule', 'dev %', 'E3 +zero', 'dev %', 'requirement (assumed)');
  M_ = {'rise 10-90 % [s]', 'tr', req.tr, '<='; 'peak time [s]', 'tp', NaN, ''; 'overshoot [%]', 'os', req.os, '<='; 'settling 2 % [s]', 'ts', req.ts, '<='};
  tab = zeros(4, 7);  verdict = cell(4, 1);
  for q = 1:4
    f = M_{q,2};  ex = sp.(f);  v1 = e1.(f);  v2 = e2.(f);  v3 = e3.(f);
    d = @(v) 100*(v - ex)/max(abs(ex), eps);   % NaN when the estimate has no peak
    if isnan(M_{q,3}), rq = '       none'; vd = 'n/a'; else, rq = sprintf('%s %g', M_{q,4}, M_{q,3}); vd = ifelse(ex <= M_{q,3}, 'met', 'NOT met'); end
    say('  %-22s %12.4g | %12.4g %8.1f | %12.4g %8.1f | %12.4g %8.1f | %s -> %s\n', M_{q,1}, ex, v1, d(v1), v2, d(v2), v3, d(v3), rq, vd);
    tab(q,:) = [ex v1 d(v1) v2 d(v2) v3 d(v3)];  verdict{q} = vd;
  end
  say('\nHow good is the dominant-pole approximation here?\n');
  poor1 = ~(abs(tab(:,3)) <= 25);  poor2 = ~(abs(tab(:,5)) <= 25);  poor3 = ~(abs(tab(:,7)) <= 25);   % NaN (no peak) counts as poor
  say('  deviation > 25 %% counts as poor.  E1 poor on: %s;  E2 poor on: %s;  E3 poor on: %s\n', list_names(M_(:,1), poor1), list_names(M_(:,1), poor2), list_names(M_(:,1), poor3));
  say('  Reasons: (1) the two slowest poles (%.2f, %.2f rad/s) are real and of the same order as each other and as the PI zero (%.1f rad/s), so there is no dominant pair;\n', p1, p2, 1/K.ti);
  say('  the zero sits next to the slow poles and creates the %.1f %% overshoot that the overdamped two-pole model (zeta = %.2f) cannot show; (2) the lightly damped pair at %.1f +- %.1fj is %.0fx faster than wn(E1) and is ignored by every estimate;\n', sp.os, z1, -real(pl(imag(pl) > 0)), imag(pl(imag(pl) > 0)), abs(pl(imag(pl) > 0))/wn1);
  say('  (3) the PM-based second-order rule is a rough guide for a type-2 loop (it assumes no zero and a single crossover mechanism).\n');
  say('  Note: the full 2-DOF controller does not follow this response; with the exact inverse the reference response is set by the generator (triple pole %g rad/s).\n', K.p_dev);
  say('  Cross-check with outputs/analysis.txt (feedback loop step): overshoot 15.0 %%, 10-90 rise 0.31 s, 2 %% settling 2.63 s; here %.1f %%, %.2f s, %.2f s\n', sp.os, sp.tr, sp.ts);
  done();
  R.sp = sp;  R.e1 = e1;  R.e2 = e2;  R.e3 = e3;  R.tab = tab;  R.req = req;  R.verdict = verdict;  R.z1 = z1;  R.wn1 = wn1;  R.z2 = z2;  R.wn2 = wn2;
  save('-mat', fullfile(rd, 'tc_study_specs.mat'), 'R');

  hf = figure('visible', 'off', 'position', [50 50 800 500], 'color', 'w');
  tm = t <= 6;  plot(t(tm), y(tm), 'b', 'linewidth', 2);  hold on;  plot(t(tm), y1(tm), 'r--', 'linewidth', 1.5);  plot(t(tm), y3(tm), 'g-.', 'linewidth', 1.5);
  tt = linspace(0, 6, 600);  z = z2;  wd2 = wn2*sqrt(1 - z^2);
  y2 = 1 - exp(-z*wn2*tt).*(cos(wd2*tt) + z*wn2/wd2*sin(wd2*tt));  plot(tt, y2, 'm:', 'linewidth', 1.5);
  plot([0 6], [1 1], 'k:');  plot([0 6], 1 + req.os/100*[1 1], 'k--');  plot([0 6], 1.02*[1 1], 'Color', [.6 .6 .6]);  hold off;  grid on;
  xlabel('t [s]');  ylabel('\theta / \theta_{ref}');  title('Closed-loop step: polynomial model against dominant-pole estimates');
  legend({'polynomial T = L/(1+L)', 'E1: two slowest poles', 'E3: two poles + PI zero', 'E2: PM/bandwidth rule', '', 'overshoot target 20 %', '2 % band'}, 'location', 'southeast');
  tc_save_figure(hf, 'tc_study_specs');
end

function s = list_names(names, mask)
  if ~any(mask), s = 'none'; else, s = strjoin(names(mask)', ', '); end
end

function s = ifelse(c, a, b), if c, s = a; else, s = b; end, end
