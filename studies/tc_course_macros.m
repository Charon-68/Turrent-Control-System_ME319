function tc_course_macros(mac, pd, rd)
% TC_COURSE_MACROS  Course-study numbers for the paper: LaTeX macros and two small tables, taken ONLY from the
%   result files of the tc_study_* scripts in outputs/ (.mat).  Called by tc_export_numbers; a study whose result
%   file is missing is skipped (its macros are then simply not defined, and the paper would fail to compile loudly
%   rather than show a stale number).  mac(name, value) writes \newcommand{\name}{value}.
  ld = @(n) fullfile(rd, [n '.mat']);
  tex_sci = @(x) sprintf('%.1f\\times10^{%d}', x/10^floor(log10(x)), floor(log10(x)));

  if exist(ld('tc_study_los_linearization'), 'file')
    load(ld('tc_study_los_linearization'));  T = R.table;                 % [r sig_th sd_lin sd_exact range_only r*sig_th allowed sig_conv]
    mac('LosRelOneM', tex_sci(R.rel_err_1m));  mac('LosSlope', sprintf('%.0f', mean(R.slope)));
    mac('LosRawLo', sprintf('%.2f', T(1,6)));  mac('LosRawHi', sprintf('%.2f', T(end,6)));
    mac('LosUseLo', sprintf('%.0f', 100*T(1,6)/2));  mac('LosUseHi', sprintf('%.0f', 100*T(end,6)/2));
    if ~isempty(R.eb)
      mac('LosBlkLo', sprintf('%.2f', R.eb(1,5)*pi/180*R.eb(1,1)));  mac('LosBlkHi', sprintf('%.2f', R.eb(end,5)*pi/180*R.eb(end,1)));
      [~, i2] = min(abs(R.eb(:,5)*pi/180.*R.eb(:,1) - 2));  mac('LosBlkCross', sprintf('%.0f', R.eb(i2,1)));
    end
  end
  if exist(ld('tc_study_sensitivity'), 'file')
    load(ld('tc_study_sensitivity'));
    mac('SensMt', sprintf('%.2f', R.Mt));  mac('SensWb', sprintf('%.2f', R.wb));  mac('SensEncT', sprintf('%.2f', R.encT));
    mac('SensTNyq', sprintf('%.1f', 20*log10(R.T_at_N)));  mac('SensWMs', sprintf('%.0f', R.wMs));  mac('SensWs', sprintf('%.1f', R.ws));
  end
  if exist(ld('tc_study_specs'), 'file')
    load(ld('tc_study_specs'));  tab = R.tab;  tab_f = fopen(fullfile(pd, 'course_spec_table.tex'), 'w');
    nm = {'rise time 10--90\,\% [s]', 'peak time [s]', 'overshoot [\%]', 'settling 2\,\% [s]'};
    rq = {sprintf('$\\le %g$', R.req.tr), '--', sprintf('$\\le %g$', R.req.os), sprintf('$\\le %g$', R.req.ts)};
    fm = {'%.2f', '%.2f', '%.1f', '%.2f'};
    for q = 1:4
      if isnan(tab(q,2)), e1 = 'none'; d1 = '--'; else, e1 = sprintf(fm{q}, tab(q,2)); d1 = sprintf('%+.0f', tab(q,3)); end
      ok = R.verdict{q};  if strcmp(ok, 'n/a'), ok = '--'; end
      fprintf(tab_f, '%s & %s & %s & %s & %s & %s & %s \\\\\n', nm{q}, sprintf(fm{q}, tab(q,1)), e1, d1, sprintf(fm{q}, tab(q,6)), rq{q}, ok);
    end
    fclose(tab_f);
    mac('SpecZetaOne', sprintf('%.2f', R.z1));  mac('SpecWnOne', sprintf('%.2f', R.wn1));  mac('SpecReqOs', sprintf('%g', R.req.os));
    mac('SpecEThreeMax', sprintf('%.0f', max(abs(tab(:,7)))));  mac('SpecReqTr', sprintf('%g', R.req.tr));  mac('SpecReqTs', sprintf('%g', R.req.ts));
  end
  if exist(ld('tc_study_uncertainty_margins'), 'file')
    load(ld('tc_study_uncertainty_margins'));  rows = R.single;  rc = R.corners;  [~, ig] = min(rc(:,3));
    tab_f = fopen(fullfile(pd, 'course_margin_table.tex'), 'w');
    fprintf(tab_f, 'nominal & %.2f & %.1f & %.1f & %.2f \\\\\n', rows(1,1), rows(1,2), rows(1,3), rows(1,5));
    lab = {'$K_t$', '$K_e$', '$J$', '$b$'};
    for q = 1:4
      ii = [2 3] + 2*(q - 1);
      fprintf(tab_f, '%s $\\pm20\\,\\%%$ & %.2f--%.2f & %.1f & %.1f & %.2f \\\\\n', lab{q}, min(rows(ii,1)), max(rows(ii,1)), min(rows(ii,2)), min(rows(ii,3)), min(rows(ii,5)));
    end
    fprintf(tab_f, '16 corners & %.2f--%.2f & %.1f & %.1f & %.2f \\\\\n', min(rc(:,1)), max(rc(:,1)), min(rc(:,2)), min(rc(:,3)), min(rc(:,5)));
    fclose(tab_f);
    mac('UncPMmin', sprintf('%.1f', min(rc(:,2))));  mac('UncGMmin', sprintf('%.1f', min(rc(:,3))));  mac('UncKmin', sprintf('%.2f', min(rc(:,5))));
  end
  if exist(ld('tc_study_compensator_compare'), 'file')
    load(ld('tc_study_compensator_compare'));  r = R.rows;       % rows: P4000, P8012, lead Kv8, lead Kv2, PI ; cols wc pm gm wb Kv Ka ramp os tr ts CSnyq stable
    mac('CompKvP', sprintf('%.2f', r(1,5)));  mac('CompPmP', sprintf('%.0f', r(1,2)));  mac('CompGmP', sprintf('%.1f', r(1,3)));
    mac('CompGmPhi', sprintf('%.1f', r(2,3)));  mac('CompPhiNeeded', sprintf('%.1f', R.phi_needed));
    mac('CompGmLeadLo', sprintf('%.1f', r(4,3)));  mac('CompGmLeadHi', sprintf('%.1f', r(3,3)));
    mac('CompRampM', sprintf('%.0f', 800*(150/800)/r(1,5)));  mac('CompNoiseRatio', sprintf('%.1f', r(4,11)/r(5,11)));
    mac('CompKvLim', sprintf('%.1f', R.kv_limit));
  end
  if exist(ld('tc_study_delay_bandwidth'), 'file')
    load(ld('tc_study_delay_bandwidth'));  rw = R.rows;  sf = R.safe;
    mac('DelayMargin', sprintf('%.2f', R.Tdm));  mac('DelayPMZoh', sprintf('%.1f', rw(3,3)));  mac('DelayPMLat', sprintf('%.1f', rw(4,3)));
    mac('DelayHold', sprintf('%.0f', R.hold_m));  mac('DelayCoast', sprintf('%.1f', R.coast_m));
    mac('DelaySafeFive', sprintf('%.1f', sf(2,3)));  mac('DelaySafeHund', sprintf('%.1f', sf(6,3)));  mac('DelayWbHund', sprintf('%.1f', sf(6,4)));
    mac('DelayDegTen', sprintf('%.1f', rw(1,2)*0.01*180/pi));  mac('DelayTdFortyFive', sprintf('%.2f', (rw(1,3) - 45)*pi/180/rw(1,2)));
  end
  if exist(ld('tc_study_sampling'), 'file')
    load(ld('tc_study_sampling'));  rw = R.rows;
    mac('SampWeq', sprintf('%.2f', R.weq));  mac('SampNyq', sprintf('%.1f', R.wNyq));  mac('SampResPct', sprintf('%.0f', 100*R.wres/R.wNyq));
    mac('SampZohDeg', sprintf('%.1f', -rw(1,3)));  mac('SampPerSample', sprintf('%.1f', R.th1*0.05*800));
    mac('SampPMTwenty', sprintf('%.1f', rw(1,6)));  mac('SampGMTwenty', sprintf('%.1f', rw(1,7)));
  end
end
