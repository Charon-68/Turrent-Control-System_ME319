function R = tc_study_feedforward_vs_feedback()
% TC_STUDY_FEEDFORWARD_VS_FEEDBACK  2-DOF structure on the LINEAR plant: feedback only, feed-forward only, both.
%   Controller:  V = Ginv_nom(s) y_d + C(s) (y_d - theta)           (both)
%                V = C(s) (y_d - theta)                              (feedback only)
%                V = Ginv_nom(s) y_d                                 (feed-forward only)
%   Ginv_nom = s a_nom(s)/gn_nom (exact inverse of the NOMINAL plant); the true plant G_t has Kt, Ke, J or b off by +-20 %.
%   Reference y_d: a ramp of rate 150/800 = 0.1875 rad/s (the worst LOS rate) passed through the generator's
%   triple pole (p = 20 rad/s), which also makes the inverse realisable.  Error e = y_d - theta, window 0-10 s
%   (the engagement length), in rad and as cross-range metres at 800 m.
%     feedback only  E = S_t y_d;   feed-forward only  E = (1 - R) y_d;   both  E = S_t (1 - R) y_d,
%     R = G_t Ginv_nom = gn_t a_nom/(gn_nom a_t),  S_t = 1/(1 + C G_t).
%   Exact responses by partial fractions.  Writes outputs/tc_study_feedforward_vs_feedback.{txt,mat}, plots/....
  here = fileparts(mfilename('fullpath'));  addpath(fileparts(here));  tc_setup();
  Mn = tc_course_model();  K = Mn.K;  rd = fullfile(fileparts(here), 'outputs');
  [say, done] = tc_course_report('tc_study_feedforward_vs_feedback');
  om = 150/800;  p = K.p_dev;  Fn = p^3;  Fd = conv([1 p], conv([1 p], [1 p]));  T_end = 10;
  t = 0:0.002:T_end;  names = {'Kt', 'Ke', 'J', 'b'};  d = 0.2;
  say('Reference: ramp %.4f rad/s (150 m/s at 800 m) through the generator (s+%g)^-3; error e = y_d - theta over 0..%g s; metres = rad x 800 m.\n', om, p, T_end);
  say('Ramp lag of the generator itself (3 om/p) = %.4f rad = %.1f m: common to all rows, it is the reference, not the tracking error.\n\n', 3*om/p, 3*om/p*800);
  cases = {struct(), 'nominal'};
  for q = 1:4, for sg = [-1 1], sc = struct();  sc.(names{q}) = 1 + sg*d;  cases(end+1,:) = {sc, sprintf('%s %+d %%', names{q}, sg*100*d)}; end, end %#ok<AGROW>
  for c = 0:15
    sc = struct();  lab = '';
    for q = 1:4, sg = 2*bitget(c, q) - 1;  sc.(names{q}) = 1 + sg*d;  lab = [lab sprintf('%s%s ', names{q}, ifelse(sg > 0, '+', '-'))]; end %#ok<AGROW>
    cases(end+1,:) = {sc, ['corner ' strtrim(lab)]}; %#ok<AGROW>
  end
  nC = size(cases, 1);  res = zeros(nC, 9);  E = cell(nC, 3);
  for q = 1:nC
    Mt = tc_course_model(cases{q,1});
    Nd = tc_poly_add(Mn.gn*Mt.a, -Mt.gn*Mn.a);  Dd = Mn.gn*Mt.a;                     % 1 - R = Nd/Dd
    e1 = tc_lin_response(conv(Mt.Ld, Fn), conv(Mt.cl, Fd), 'ramp', om, t);               % feedback only
    e2 = tc_lin_response(conv(Nd, Fn), conv(Dd, Fd), 'ramp', om, t);                     % feed-forward only
    e3 = tc_lin_response(conv(conv(Nd, Mt.Ld), Fn), conv(conv(Dd, Mt.cl), Fd), 'ramp', om, t);   % both
    E(q,:) = {e1, e2, e3};
    res(q,:) = [e1(end) max(abs(e1)) e2(end) max(abs(e2)) e3(end) max(abs(e3)) 0 0 0];
  end
  say('%-22s | %12s %12s | %12s %12s | %12s %12s   (rad; end = value at %g s, peak = max |e|)\n', 'plant', 'FB end', 'FB peak', 'FF end', 'FF peak', 'both end', 'both peak', T_end);
  for q = 1:nC
    if q == 10, say('\nAll 16 corners (all four parameters at +-20 %%)\n'); end
    say('%-22s | %12.3e %12.3e | %12.3e %12.3e | %12.3e %12.3e\n', cases{q,2}, res(q,1:6));
  end
  one = 1:9;  cor = 10:nC;
  [~, iw] = max(res(cor,6));  iw = cor(iw);
  say('\nMetres at 800 m (rad x 800)\n');
  say('%-22s | %10s %10s | %10s %10s | %10s %10s\n', 'plant', 'FB end', 'FB peak', 'FF end', 'FF peak', 'both end', 'both peak');
  for q = [1 2 3 4 5 6 7 8 9 iw]
    say('%-22s | %10.3f %10.3f | %10.3f %10.3f | %10.3f %10.3f\n', cases{q,2}, 800*res(q,1:6));
  end
  say('\nNominal plant: feedback only has peak %.3f m and end %.1e m; feed-forward only and both are exact: %.1e and %.1e m (numerical zero).\n', 800*res(1,2), 800*abs(res(1,1)), 800*res(1,4), 800*res(1,6));
  say('Worst single-parameter case for each structure (peak, m): FB %.3f (%s), FF %.3f (%s), both %.3f (%s).\n', ...
      800*max(res(one,2)), cases{find(res(one,2) == max(res(one,2)), 1), 2}, 800*max(res(one,4)), cases{find(res(one,4) == max(res(one,4)), 1), 2}, 800*max(res(one,6)), cases{find(res(one,6) == max(res(one,6)), 1), 2});
  say('Worst of the 16 corners for each structure (peak, m): FB %.3f, FF %.3f (%s), both %.3f (%s).\n', 800*max(res(cor,2)), 800*max(res(cor,4)), cases{cor(find(res(cor,4) == max(res(cor,4)), 1)), 2}, 800*max(res(cor,6)), cases{iw, 2});
  say('Interpretation: feed-forward alone is exact only for the exact model, and the error then GROWS with time (a wrong rate gain: e = (1 - Kv_t/Kv_nom) om t); feedback alone has a start-up transient\n');
  say('but forgets the model (type 2: error returns to zero); both: the feed-forward carries the ramp so the feedback only has to correct the small mismatch.  Peak error of both is below FB-only in %d of %d cases and below FF-only in %d of %d.\n', ...
      sum(res(:,6) < res(:,2)), nC, sum(res(:,6) < res(:,4)), nC);
  fn = fieldnames(K);  ffn = fn(~cellfun(@isempty, regexp(fn, 'ff')));
  say('Time-domain check from the stored ablations: tc_params has these feed-forward-related fields: %s; none of them disables the feed-forward (adapt_ff only scales its velocity term), so no switch exists and no code was changed to add one: no time-domain ablation is reported here.\n', strjoin(ffn', ', '));
  done();
  R.cases = cases(:,2);  R.res = res;  R.om = om;  R.worst_corner = cases{iw,2};
  save('-mat', fullfile(rd, 'tc_study_feedforward_vs_feedback.mat'), 'R');

  hf = figure('visible', 'off', 'position', [50 50 1100 430], 'color', 'w');  tl = {'FB only', 'FF only', 'both'};  col = {'b', 'r', 'k'};
  for k = 1:2
    subplot(1,2,k);  hold on;  qq = ifelse(k == 1, 1, iw);
    for m = 1:3, plot(t, 800*E{qq,m}, col{m}, 'linewidth', 1.6); end
    hold off;  grid on;  xlabel('t [s]');  ylabel('error y_d - \theta at 800 m [m]');
    title(sprintf('%s plant, ramp %.3f rad/s', ifelse(k == 1, 'nominal', ['worst corner ' cases{iw,2}(8:end)]), om));  legend(tl, 'location', 'northeast');
  end
  tc_save_figure(hf, 'tc_study_feedforward_vs_feedback');
end

function s = ifelse(c, a, b), if c, s = a; else, s = b; end, end
