function R = tc_study_uncertainty_margins()
% TC_STUDY_UNCERTAINTY_MARGINS  Gain margin, phase margin, crossover and Routh gain limits for Kt, Ke, J (J_eq)
%   and b each varied by +-20 %, one at a time, and over all 16 corners (worst corner reported).
%   Same linear loop model as the design script (tc_course_model): only the plant polynomial changes, the
%   compensator is the fixed final design.  Kt enters both the torque gain gn = N Kt and the coupling N^2 Kt Ke.
%   Routh limits: P = proportional-only, k < a1 a0/(a2 gn) (closed form, cross-checked by the Routh array);
%   full = largest scale on the whole PI + roll-off loop gain found from the Routh first column (and from roots).
%   Writes outputs/tc_study_uncertainty_margins.{txt,mat}; the paper table is written from the .mat by tc_export_numbers.
  here = fileparts(mfilename('fullpath'));  root = fileparts(here);  addpath(root);  tc_setup();
  K = tc_params();  rd = fullfile(root, 'outputs');
  [say, done] = tc_course_report('tc_study_uncertainty_margins');
  names = {'Kt', 'Ke', 'J', 'b'};  d = 0.2;
  cases = {struct(), 'nominal'};
  for q = 1:4
    for sgn = [-1 1]
      sc = struct();  sc.(names{q}) = 1 + sgn*d;
      cases(end+1,:) = {sc, sprintf('%s %+d %%', names{q}, sgn*100*d)}; %#ok<AGROW>
    end
  end
  % all 16 corners
  corners = cell(16, 2);
  for c = 0:15
    sc = struct();  lab = '';
    for q = 1:4
      sg = 2*bitget(c, q) - 1;  sc.(names{q}) = 1 + sg*d;  lab = [lab sprintf('%s%s ', names{q}, ifelse(sg > 0, '+', '-'))]; %#ok<AGROW>
    end
    corners(c+1,:) = {sc, strtrim(lab)};
  end
  say('Final compensator: Kp = %g V/rad, Ti = %g s, tau_f = %g s.  Plant G = gn/(s a(s)), a = [L J, L b + R J, R b + N^2 Kt Ke]; J means J_eq = %.0f kg m^2.\n', K.kp, K.ti, K.tau_f, K.J_eq);
  say('Nominal cross-check (outputs/analysis.txt): wc 4.10, PM 68.5, GM 16.1 dB, P-only Routh limit 10484 V/rad, full-loop limit 6.35x.\n\n');
  say('%-18s %8s %8s %8s %12s %12s %10s %8s\n', 'case', 'wc', 'PM', 'GM', 'P-only Kp', 'full-loop', 'full Kp', 'stable');
  say('%-18s %8s %8s %8s %12s %12s %10s %8s\n', '', '[rad/s]', '[deg]', '[dB]', 'limit[V/rad]', 'scale k', 'lim[V/rad]', '(1=yes)');
  [rows, ~] = run_cases(cases, K);
  print_rows(say, cases(:,2), rows);
  R.single = rows;  R.single_names = cases(:,2);
  say('\nWorst single-parameter cases: lowest PM %.1f deg (%s), lowest GM %.1f dB (%s), lowest full-loop Routh scale %.2f (%s)\n', ...
      min(rows(2:end,2)), cases{1 + find(rows(2:end,2) == min(rows(2:end,2)), 1), 2}, min(rows(2:end,3)), cases{1 + find(rows(2:end,3) == min(rows(2:end,3)), 1), 2}, ...
      min(rows(2:end,5)), cases{1 + find(rows(2:end,5) == min(rows(2:end,5)), 1), 2});
  say('\nAll 16 corners (+-20 %% on Kt, Ke, J, b together)\n');
  say('%-18s %8s %8s %8s %12s %12s %10s %8s\n', 'corner', 'wc', 'PM', 'GM', 'P-only Kp', 'full-loop', 'full Kp', 'stable');
  [rc, ~] = run_cases(corners, K);
  print_rows(say, corners(:,2), rc);
  [pmw, ip] = min(rc(:,2));  [gmw, ig] = min(rc(:,3));  [kw, ik] = min(rc(:,5));
  say('\nWorst corner by phase margin: %s  PM %.1f deg (wc %.2f rad/s, GM %.1f dB)\n', corners{ip,2}, pmw, rc(ip,1), rc(ip,3));
  say('Worst corner by gain margin : %s  GM %.1f dB, PM %.1f deg, full-loop Routh scale %.2f (Kp < %.0f V/rad), P-only limit %.0f V/rad\n', corners{ig,2}, gmw, rc(ig,2), rc(ig,5), rc(ig,6), rc(ig,4));
  say('Lowest full-loop Routh limit: %s  k < %.2f\n', corners{ik,2}, kw);
  say('All %d corners and %d single cases closed-loop stable at the design gain: %d\n', size(corners,1), size(cases,1), all(rc(:,7) == 1) && all(rows(:,7) == 1));
  say('Routh-array limit vs root-based limit, max relative difference over all cases: %.1e\n', max([abs(rows(:,5) - rows(:,8))./rows(:,5); abs(rc(:,5) - rc(:,8))./rc(:,5)]));
  say('Closed-form P-only limit vs Routh-array bisection, max relative difference: %.1e\n', max([abs(rows(:,4) - rows(:,9))./rows(:,4); abs(rc(:,4) - rc(:,9))./rc(:,4)]));
  done();
  R.corners = rc;  R.corner_names = corners(:,2);  R.worst_pm = pmw;  R.worst_pm_name = corners{ip,2};  R.worst_gm = gmw;  R.worst_gm_name = corners{ig,2};
  R.worst_k = kw;  R.worst_k_name = corners{ik,2};  R.cols = {'wc', 'pm', 'gm', 'kmaxP', 'kfull', 'kpfull', 'stable', 'kfull_roots', 'kmaxP_routh'};
  save('-mat', fullfile(rd, 'tc_study_uncertainty_margins.mat'), 'R');

end

function [rows, M] = run_cases(cases, K) %#ok<INUSD>
  rows = zeros(size(cases, 1), 9);  M = [];
  for q = 1:size(cases, 1)
    m = tc_course_model(cases{q,1});  mg = tc_loop_margins(m.Ln, m.Ld);
    [kf, kroot] = tc_routh_limit(m.Ln, m.Ld);
    kmaxP = m.a(2)*m.a(3)/(m.a(1)*m.gn);
    [~, kP] = tc_routh_limit(m.gn*K.kp, m.Gd);          % proportional-only loop gain Kp*gn/(s a(s)), scaled
    kmaxP_routh = kP*K.kp;
    pm = min(mg.pm_all);
    rows(q,:) = [mg.wc, pm, mg.gm, kmaxP, kf, kf*K.kp, all(real(roots(m.cl)) < 0), kroot, kmaxP_routh];
  end
end

function print_rows(say, names, rows)
  for q = 1:numel(names)
    say('%-18s %8.2f %8.1f %8.1f %12.0f %12.2f %10.0f %8d\n', names{q}, rows(q,1), rows(q,2), rows(q,3), rows(q,4), rows(q,5), rows(q,6), rows(q,7));
  end
end

function s = ifelse(c, a, b), if c, s = a; else, s = b; end, end
