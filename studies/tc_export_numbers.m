function tc_export_numbers(heldout_file)
% TC_EXPORT_NUMBERS  The ONLY bridge from results to the paper: reads result files, writes
% report/generated/numbers.tex (macros) and report/generated/*_table.tex.  Never edit those by hand; a number that is
% not produced here must not appear in the paper.
  here = fileparts(mfilename('fullpath'));  root = fileparts(here);  addpath(root);  tc_setup();
  rd = fullfile(root, 'outputs');  pd = fullfile(root, 'report', 'generated');  if ~exist(pd, 'dir'), mkdir(pd); end
  if nargin < 1, heldout_file = 'heldout_final'; end
  fid = fopen(fullfile(pd, 'numbers.tex'), 'w');
  mac = @(n, v) fprintf(fid, '\\newcommand{\\%s}{%s}\n', n, v);

  % ---- held-out campaign ---------------------------------------------------------------
  load(fullfile(rd, [heldout_file '.mat']));  s = R.sum;
  mac('HeldN', sprintf('%d', s.n));  mac('HeldInt', sprintf('%d', s.intercepted));
  mac('HeldFail', sprintf('%.2f', 100*s.failure_upper));
  mac('HeldPulses', sprintf('%d', s.pulses));  mac('HeldPulseRate', sprintf('%.2f', 100*s.pulse_hit_rate));
  mac('HeldFirstMean', sprintf('%.2f', s.t_first_mean));  mac('HeldFirstPNN', sprintf('%.2f', s.t_first_p95));
  mac('HeldFirstMax', sprintf('%.2f', s.t_first_max));  mac('HeldRmsMed', sprintf('%.2f', s.rms_median));
  mac('HeldRmsPNN', sprintf('%.2f', s.rms_p95));  mac('HeldInReng', sprintf('%.1f', 100*s.in_reng_mean));
  mac('HeldVpeak', sprintf('%.0f', s.V_peak));  mac('HeldMissMax', sprintf('%.2f', max(R.miss_max)));
  mac('HeldSeedLo', sprintf('%d', R.seeds(1)));  mac('HeldSeedHi', sprintf('%d', R.seeds(end)));

  % ---- analytical design (recomputed, not remembered) ----------------------------------------
  A = tc_plot_design_analysis();
  mac('DesWc', sprintf('%.2f', A.wc));  mac('DesPM', sprintf('%.1f', A.pm));  mac('DesGM', sprintf('%.1f', A.gm));
  mac('DesMs', sprintf('%.2f', A.Ms));  mac('DesKmaxP', sprintf('%.0f', A.kmax_p));
  mac('DesKmax', sprintf('%.2f', A.kmax));  mac('DesKa', sprintf('%.2f', A.Ka));
  K = tc_params();  mac('DesKp', sprintf('%g', K.kp));  mac('DesIlim', sprintf('%g', K.i_lim));
  Teb = tc_plot_error_budget();  mac('EbRaw', sprintf('%.2f', 2000*K.sig_th));
  mac('EbFilt', sprintf('%.2f', Teb(end,4)*pi/180*2000));  mac('EbBo', sprintf('%.2f', Teb(end,5)*pi/180*2000));

  % ---- scenarios -------------------------------------------------------------------------
  names = {'A','B','C','D1','D2','D3','D4','D5','D6'};  tab = fopen(fullfile(pd, 'suite_table.tex'), 'w');
  for q = 1:numel(names)
    o = tc_run_engagement(names{q});  m = o.metrics;
    fprintf(tab, '%s & %.2f & %d/%d & %.2f & %.1f & %.0f & %.1f \\\\\n', names{q}, m.t_first_hit, m.n_hits, m.n_pulses, m.point_rms_m, 100*m.frac_in_Reng, m.V_peak, m.I_peak);
  end
  fclose(tab);

  % ---- calibration (parsed from the generated table header) ---------------------------------
  txt = fileread(fullfile(rd, 'calibration_study.txt'));
  tk = regexp(txt, 'Brier raw\s*:\s*([\d.]+).*?Brier calibrated\s*:\s*([\d.]+)', 'tokens', 'once');
  if ~isempty(tk), mac('BrierRaw', tk{1});  mac('BrierCal', tk{2}); end

  % ---- ablations -----------------------------------------------------------------------------
  f = fullfile(rd, 'ablations.mat');
  if exist(f, 'file')
    load(f);  tab = fopen(fullfile(pd, 'ablation_table.tex'), 'w');
    fprintf(tab, 'frozen & %d/%d & %d & -- & -- & -- & -- \\\\\n', base.sum.intercepted, base.sum.n, sum(base.n_pulses - base.n_hits));
    for q = 1:numel(T)
      fprintf(tab, '%s & %d/%d & %d & %+.3f & [%+.2f,%+.2f] & %+.3f & %.3f \\\\\n', strrep(T(q).name, '_', '\_'), T(q).R.sum.intercepted, T(q).R.sum.n, ...
              sum(T(q).R.n_pulses - T(q).R.n_hits), T(q).dt.mean, T(q).dt.ci(1), T(q).dt.ci(2), T(q).dr.mean, T(q).pt);
    end
    fclose(tab);  mac('AblN', sprintf('%d', numel(seeds)));
    mac('AblSeedLo', sprintf('%d', seeds(1)));  mac('AblSeedHi', sprintf('%d', seeds(end)));
    q = find(strcmp({T.name}, 'aim_tp'));
    if ~isempty(q)
      mac('AimInt', sprintf('%d', T(q).R.sum.intercepted));  mac('AimFirst', sprintf('%.2f', T(q).R.sum.t_first_mean));
      mac('AimRms', sprintf('%.2f', T(q).R.sum.rms_median));  mac('AimPulseRate', sprintf('%.1f', 100*T(q).R.sum.pulse_hit_rate));
      mac('AimMiss', sprintf('%d', sum(T(q).R.n_pulses - T(q).R.n_hits)));  mac('AimPulses', sprintf('%d', T(q).R.sum.pulses));
    end
  end

  % ---- robustness ----------------------------------------------------------------------------
  f = fullfile(rd, 'robustness.mat');
  if exist(f, 'file')
    load(f);  tab = fopen(fullfile(pd, 'robust_table.tex'), 'w');
    for q = 1:numel(T)
      r = T(q).R;  n = numel(r.success);
      fprintf(tab, '%s & %d/%d & %d & %.2f & %.2f \\\\\n', strrep(strrep(T(q).name, '%', '\%'), '_', '\_'), sum(r.success), n, sum(r.n_pulses - r.n_hits), mean(r.t_first(r.success)), median(r.rms));
    end
    fclose(tab);  mac('RobN', sprintf('%d', numel(T(1).R.success)));
  end
  f = fullfile(rd, 'time.txt');
  if exist(f, 'file')
    L = strsplit(strtrim(fileread(f)), "\n");  v = sscanf(L{2}, '%f');
    mac('TimeMean', sprintf('%.2f', 1e3*v(1)));  mac('TimeMed', sprintf('%.2f', 1e3*v(2)));  mac('TimePNN', sprintf('%.2f', 1e3*v(3)));
  end
  % ---- independent check and evaluation-adapter sweep (macros only; from outputs/indep_*.mat and outputs/eval_adapter_*.txt) --------
  fams = {'Weave', 'Step', 'Bang', 'Arc', 'Popup'};  ok = true;  Ri = cell(1, 5);
  for q = 1:5
    f = fullfile(rd, sprintf('indep_%d.mat', q));
    if ~exist(f, 'file'), ok = false; break; end
    A = load(f);  Ri{q} = A.R;
  end
  if ok
    nc = size(Ri{1}.success, 1);  ncase = 0;  succ = zeros(nc, 1);  miss = zeros(nc, 1);  anym = false(1, 0);
    for q = 1:5
      r = Ri{q};  ncase = ncase + size(r.success, 2);  succ = succ + sum(r.success, 2);  miss = miss + sum(r.n_miss, 2);
      cm = any(r.n_miss > 0, 1);  anym = [anym, cm]; %#ok<AGROW>
      mac(['IndMiss' fams{q}], sprintf('%d', sum(cm)));
    end
    ci = tc_wilson_interval(min(succ), ncase);  lo = ci(1);
    mac('IndConfigs', sprintf('%d', nc));  mac('IndCases', sprintf('%d', ncase));  mac('IndRuns', sprintf('%d', nc*ncase));
    mac('IndIntMin', sprintf('%d', min(succ)));  mac('IndFail', sprintf('%.2f', 100*(1 - lo)));
    mac('IndMissCases', sprintf('%d', sum(anym)));  mac('IndMissLo', sprintf('%d', min(miss)));  mac('IndMissHi', sprintf('%d', max(miss)));
  end
  f = fullfile(rd, 'eval_adapter_sweep.txt');  g = fullfile(rd, 'eval_adapter_misses.txt');
  if exist(f, 'file') && exist(g, 'file')
    tk = regexp(fileread(f), 'runs (\d+), runs without a hit (\d+), pulses that missed (\d+)', 'tokens', 'once');
    mac('SwRuns', tk{1});  mac('SwHits', sprintf('%d', str2double(tk{1}) - str2double(tk{2})));  mac('SwMissN', tk{3});
    P = regexp(fileread(g), 'PULSE scn=\S+ f_Hz=(\d+) t=([\d.]+) miss_m=([\d.]+) hit=0', 'tokens');
    C = regexp(fileread(g), 'CASE scn=\S+ theta0_deg=(\d+) f_Hz=(\d+) first_hit=([\d.]+)', 'tokens');
    nm = {'A', 'B'};
    for q = 1:min([numel(P), numel(C), 2])
      mac(['SwFreq' nm{q}], P{q}{1});  mac(['SwMissT' nm{q}], sprintf('%.2f', str2double(P{q}{2})));
      mac(['SwMissD' nm{q}], sprintf('%.2f', str2double(P{q}{3})));  mac(['SwFirst' nm{q}], sprintf('%.2f', str2double(C{q}{3})));
      mac(['SwTheta' nm{q}], C{q}{1});
    end
  end
  % ---- start-angle miss analysis (macros only; parsed from outputs/start_angle_miss.txt) ----------------------------------
  f = fullfile(rd, 'start_angle_miss.txt');
  if exist(f, 'file')
    sa = fileread(f);  gv = @(pat) str2double(regexp(sa, pat, 'tokens', 'once'){1});
    miss = abs(gv('beam error \(true\)\s+(-?[\d.]+) m'));  filt = abs(gv('= filter bearing error\s+(-?[\d.]+) m'));
    seen = abs(gv('offset the controller sees\s+(-?[\d.]+) m'));  sig = gv('posterior sigma \(beam\)\s+([\d.]+) m');
    mac('SaMiss', sprintf('%.2f', miss));  mac('SaFilt', sprintf('%.2f', filt));  mac('SaSeen', sprintf('%.2f', seen));
    mac('SaSig', sprintf('%.2f', sig));  mac('SaRatio', sprintf('%.1f', filt/sig));
    mac('SaP', sprintf('%.3f', gv('hit probability\s+p_raw [\d.]+\s+p ([\d.]+)')));
    mac('SaPsure', sprintf('%.2f', gv('\(p_sure ([\d.]+),')));
    tk = regexp(sa, '\n\s*80d\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)\s+(\d+)/(\d+)', 'tokens', 'once');
    mac('SaDrawMin', sprintf('%.2f', str2double(tk{1})));  mac('SaDrawMax', sprintf('%.2f', str2double(tk{3})));
    mac('SaDrawsOver', tk{4});  mac('SaDraws', tk{5});
  end
  % ---- report macros: plant constants (from tc_params), estimation figure, final-evaluation figure, robustness ----------------
  Kp_ = tc_params();  pl = roots(Kp_.den);  pl = pl(abs(imag(pl)) > 0);  pl = pl(imag(pl) > 0);
  mac('PlJeq', sprintf('%g', Kp_.J_eq));  mac('PlA', sprintf('%g', Kp_.den(1)*1000/Kp_.gn));  mac('PlB', sprintf('%g', Kp_.den(2)*1000/Kp_.gn));
  mac('PlC', sprintf('%.4f', Kp_.den(3)*1000/Kp_.gn/1e6));  mac('PlGain', sprintf('%g', Kp_.gn));
  mac('PlPoleRe', sprintf('%.2f', -real(pl(1))));  mac('PlPoleIm', sprintf('%.2f', imag(pl(1))));  mac('PlWn', sprintf('%.1f', abs(pl(1))));
  mac('PlHz', sprintf('%.2f', abs(pl(1))/2/pi));  mac('PlZeta', sprintf('%.3f', -real(pl(1))/abs(pl(1))));
  mac('FfOne', sprintf('%.2f', Kp_.ff(1)));  mac('FfTwo', sprintf('%.2f', Kp_.ff(2)));  mac('FfThree', sprintf('%.1f', Kp_.ff(3)));
  mac('Vlim', sprintf('%g', Kp_.V_lim));  mac('OmegaCruise', sprintf('%.3f', Kp_.omega_cruise));  mac('AcqAccel', sprintf('%.1f', Kp_.acq_accel));
  mac('GenPole', sprintf('%g', Kp_.p_dev));  mac('PiTi', sprintf('%g', Kp_.ti));  mac('PiTauF', sprintf('%g', Kp_.tau_f));
  mac('SigJerk', sprintf('%g', Kp_.sigma_j));  mac('TauJerk', sprintf('%g', Kp_.tau_j));  mac('Tp', sprintf('%g', Kp_.tp));
  mac('PsureV', sprintf('%.2f', Kp_.p_sure));  mac('ParmV', sprintf('%.2f', Kp_.p_arm));  mac('PcoastV', sprintf('%.2f', Kp_.p_coast));  mac('TwaitV', sprintf('%.1f', Kp_.t_wait));
  Ad_ = tc_plot_design_analysis();  f = fullfile(rd, 'analysis.txt');
  if exist(f, 'file')
    tk = regexp(fileread(f), 'overshoot ([\d.]+) %, 10-90 % rise ([\d.]+) s, 2 % settling ([\d.]+) s', 'tokens', 'once');
    mac('StepOs', tk{1});  mac('StepRise', tk{2});  mac('StepTs', tk{3});
  end
  mac('DesRatio', sprintf('%.1f', abs(pl(1))/Ad_.wc));  mac('PsureAfterV', sprintf('%.3f', Kp_.p_sure_after));
  f = fullfile(rd, 'suite_100Hz.txt');
  if exist(f, 'file')
    su = fileread(f);  fh = zeros(1, 6);  rm_ = fh;  inr = fh;  allhit = true;
    for q = 1:6
      tk = regexp(su, ['\nD' num2str(q) '\s+([\d.]+)s\s+(\d+)/(\d+)\s+([\d.]+)\s+([\d.]+)%'], 'tokens', 'once');
      fh(q) = str2double(tk{1});  rm_(q) = str2double(tk{4});  inr(q) = str2double(tk{5});  allhit = allhit && strcmp(tk{2}, tk{3});
    end
    mac('SuDFirstLo', sprintf('%.2f', min(fh)));  mac('SuDFirstHi', sprintf('%.2f', max(fh)));  mac('SuDRmsHi', sprintf('%.2f', max(rm_)));  mac('SuDInLo', sprintf('%.0f', min(inr)));
    if allhit, mac('SuDAllHit', 'every pulse hit'); else, mac('SuDAllHit', 'not every pulse hit'); end
  end
  f = fullfile(rd, 'report_scenarios.txt');
  if exist(f, 'file')
    st = fileread(f);  sc = {'A', 'B', 'C'};
    for q = 1:3
      tk = regexp(st, ['\n' sc{q} '\s+\S+\s+\S+\s+\S+\s+\S+\s+(\S+)'], 'tokens', 'once');
      mac(['ScMax' sc{q}], sprintf('%.2f', str2double(tk{1})));
      tk2 = regexp(su, ['\n' sc{q} '\s+([\d.]+)s\s+(\d+)/(\d+)\s+([\d.]+)'], 'tokens', 'once');      % from the printed suite table
      mac(['ScFirst' sc{q}], tk2{1});  mac(['ScPulses' sc{q}], tk2{2});  mac(['ScRms' sc{q}], tk2{4});
    end
  end
  f = fullfile(rd, 'report_estimation.txt');
  if exist(f, 'file')
    et = fileread(f);  sc = {'A', 'B', 'C'};
    for q = 1:3
      tk = regexp(et, ['\n' sc{q} '\s+\S+\s+\|\s+(\S+)\s+(\S+)\s+(\S+)\s+\|\s+(\S+)\s+(\S+)\s+(\S+)\s+\|\s+(\S+)\s+(\S+)'], 'tokens', 'once');
      nm = {'EstNowRms', 'EstNowMax', 'EstSig', 'EstPredRms', 'EstPredMax', 'EstPredBlk', 'EstVelRms', 'EstVelSig'};
      for j = 1:8
        v = str2double(tk{j});  if isnan(v), continue; end
        mac([nm{j} sc{q}], sprintf('%.2f', v));
      end
    end
    tk = regexp(et, 'nowcast 1-sigma ([\d.]+) m at its first call, ([\d.]+) m at its last call; \|nowcast error\| at the last call ([\d.]+) m; \|t_p-ahead error\| at the last call ([\d.]+) m', 'tokens', 'once');
    if ~isempty(tk)
      mac('EstBlkSigStart', sprintf('%.2f', str2double(tk{1})));  mac('EstBlkSigEnd', sprintf('%.2f', str2double(tk{2})));
      mac('EstBlkErrEnd', sprintf('%.2f', str2double(tk{3})));  mac('EstBlkPredEnd', sprintf('%.2f', str2double(tk{4})));
    end
  end
  f = fullfile(rd, 'report_final_eval.txt');
  if exist(f, 'file')
    fe = fileread(f);
    tk = regexp(fe, 'first hit  min ([\d.]+)  median ([\d.]+)  mean ([\d.]+)  p95 ([\d.]+)  max ([\d.]+) s;  share with first hit <= 1.5 s: ([\d.]+) %', 'tokens', 'once');
    mac('FeMin', sprintf('%.2f', str2double(tk{1})));  mac('FeMedian', sprintf('%.2f', str2double(tk{2})));  mac('FeLeShare', sprintf('%.0f', str2double(tk{6})));
    tk = regexp(fe, 'pointing RMS per run: median ([\d.]+)  p95 ([\d.]+)  max ([\d.]+) m;  seeds with RMS > 1 m: (\d+) of (\d+)', 'tokens', 'once');
    mac('FeRmsMax', sprintf('%.2f', str2double(tk{3})));  mac('FeRmsOver', tk{4});
    hz = {'20', '100', '1000'};  hn = {'Lo', 'Mid', 'Hi'};
    for q = 1:3
      tk = regexp(fe, ['\n\s+' hz{q} ' Hz: n (\d+)  mean ([\d.]+)  p95 ([\d.]+)  max ([\d.]+)'], 'tokens', 'once');
      mac(['FeInd' hn{q} 'Mean'], sprintf('%.2f', str2double(tk{2})));  mac(['FeInd' hn{q} 'PNN'], sprintf('%.2f', str2double(tk{3})));  mac(['FeInd' hn{q} 'Max'], sprintf('%.2f', str2double(tk{4})));
    end
  end
  f = fullfile(rd, 'ablations.mat');
  if exist(f, 'file')
    Aa = load(f);  Ta = Aa.T;  nma = {Ta.name};
    for pr = {{'guard_on', 'AblGuard'}, {'brake_slew', 'AblBrake'}, {'p_dev_10', 'AblPdevTen'}, {'rate_20Hz', 'AblRate'}, {'lead_on', 'AblLead'}, {'no_adapt_q', 'AblNoAdapt'}, {'no_freeze', 'AblNoFreeze'}}
      q = find(strcmp(nma, pr{1}{1}), 1);  if isempty(q), continue; end
      mac([pr{1}{2} 'Dt'], sprintf('%+.2f', Ta(q).dt.mean));
      mac([pr{1}{2} 'Miss'], sprintf('%d', sum(Ta(q).R.n_pulses - Ta(q).R.n_hits)));
      mac([pr{1}{2} 'Pt'], sprintf('%.3f', Ta(q).pt));
    end
  end
  f = fullfile(rd, 'robustness.mat');
  if exist(f, 'file')
    Ar = load(f);  Tr = Ar.T;  nmr = {Tr.name};
    ix = @(s) find(strcmp(nmr, s), 1);
    q = ix('reference 100 Hz');  mac('RobRefRms', sprintf('%.2f', median(Tr(q).R.rms)));  mac('RobRefFirst', sprintf('%.2f', mean(Tr(q).R.t_first(Tr(q).R.success))));
    q = ix('rate 20 Hz');  mac('RobLowRateRms', sprintf('%.2f', median(Tr(q).R.rms)));  mac('RobLowRateFirst', sprintf('%.2f', mean(Tr(q).R.t_first(Tr(q).R.success))));
    q = ix('Kt -20 %, Ke +20 %');  if isempty(q), q = find(~cellfun(@isempty, strfind(nmr, 'Kt -20')) & cellfun(@isempty, strfind(nmr, 'adaptive')), 1); end
    mac('RobKtKeRms', sprintf('%.2f', median(Tr(q).R.rms)));  mac('RobKtKeFirst', sprintf('%.2f', mean(Tr(q).R.t_first(Tr(q).R.success))));
    mac('RobAllInt', sprintf('%d', sum(arrayfun(@(x) sum(x.R.success), Tr))));  mac('RobAllN', sprintf('%d', sum(arrayfun(@(x) numel(x.R.success), Tr))));
    mac('RobRows', sprintf('%d', numel(Tr)));
  end
  % ---- course-concept studies (macros only; numbers come from outputs/tc_study_*.mat) -----------------------
  tc_course_macros(mac, pd, rd);
  fclose(fid);
  fprintf('wrote report/generated/numbers.tex and tables\n');
  make_short_robust_table(pd);
end

function make_short_robust_table(pd)
% Write robust_table_short.tex: selected rows of the generated robust_table.tex (all numbers are copied, none
% computed).  The full table stays in robust_table.tex and outputs/robustness.txt.  Each key must match the START
% of exactly one non-blank row; rows are written in key order.  (Replaces a former Python helper.)
  keep = {'reference 100 Hz', 'rate 20 Hz', 'rate 1 kHz', 'held last sample', 'irregular call times', 'start at -10 deg', ...
          'Kt +20 \%, Ke -20 \% &', 'Kt -20 \%, Ke +20 \% &', '  same, adaptive FF', 'Coulomb 300', 'HB adversarial family', 'MC + glint'};
  rows = regexp(fileread(fullfile(pd, 'robust_table.tex')), '\r\n|\n|\r', 'split');
  rows = rows(~cellfun(@isempty, strtrim(rows)));
  out = cell(1, numel(keep));
  for q = 1:numel(keep)
    m = rows(strncmp(rows, keep{q}, numel(keep{q})));
    if numel(m) ~= 1, error('row not found or ambiguous: ''%s'' (%d matches)', keep{q}, numel(m)); end
    out{q} = m{1};
  end
  fid = fopen(fullfile(pd, 'robust_table_short.tex'), 'w');
  fprintf(fid, '%s\n', out{:});
  fclose(fid);
  fprintf('wrote report/generated/robust_table_short.tex with %d of %d rows\n', numel(out), numel(rows));
end
