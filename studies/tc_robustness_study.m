function T = tc_robustness_study(seeds, tag)
% TC_ROBUSTNESS_STUDY  Frozen design against everything the evaluation script might do differently:
% call rate, measurement convention, initial turret angle, plant mismatch (with and without the
% adaptive velocity feed-forward), friction/dead-zone, adversarial HB family, out-of-spec sensor
% models.  Same seeds in every row.  Writes outputs/<tag>.mat and outputs/<tag>.txt.
  here = fileparts(mfilename('fullpath'));  addpath(fileparts(here));  tc_setup();
  if nargin < 1, seeds = 5000:5029; end
  if nargin < 2, tag = 'robustness'; end
  d2r = pi/180;
  rows = {
    'reference 100 Hz',        'MC', struct(), struct();
    'rate 20 Hz',              'MC', struct('f_ctrl', 20), struct();
    'rate 50 Hz',              'MC', struct('f_ctrl', 50), struct();
    'rate 400 Hz',             'MC', struct('f_ctrl', 400), struct();
    'rate 1 kHz',              'MC', struct('f_ctrl', 1000), struct();
    'held last sample',        'MC', struct('convention', 'held'), struct();
    'correlated copies',       'MC', struct('convention', 'drift'), struct();
    'irregular call times',    'MC', struct('convention', 'jitter'), struct();
    'start at -10 deg',        'MC', struct('theta0', -10*d2r), struct();
    'start at 45 deg',         'MC', struct('theta0', 45*d2r), struct();
    'start at 60 deg',         'MC', struct('theta0', 60*d2r), struct();
    'forward Euler plant',     'MC', struct('integrator', 'euler'), struct();
    'R,L,J,b +20 %',           'MC', struct('plant_scale', struct('R',1.2,'L',1.2,'J',1.2,'b',1.2)), struct();
    'R,L,J,b -20 %',           'MC', struct('plant_scale', struct('R',0.8,'L',0.8,'J',0.8,'b',0.8)), struct();
    'Kt +20 %, Ke -20 %',      'MC', struct('plant_scale', struct('Kt',1.2,'Ke',0.8)), struct();
    'Kt -20 %, Ke +20 %',      'MC', struct('plant_scale', struct('Kt',0.8,'Ke',1.2)), struct();
    '  same, adaptive FF',     'MC', struct('plant_scale', struct('Kt',0.8,'Ke',1.2)), struct('adapt_ff', true);
    'Kt +20 %, Ke -20 %, adaptive FF', 'MC', struct('plant_scale', struct('Kt',1.2,'Ke',0.8)), struct('adapt_ff', true);
    'Coulomb 300 Nm + 20 V dead-zone', 'MC', struct('friction', struct('coulomb', 300, 'deadzone', 20), 'integrator', 'rk4'), struct();
    'HB adversarial family',   'HB', struct(), struct();
    'MC + glint (out of spec)','MC+glint', struct(), struct();
    'MC + range-scaled noise', 'MC+range', struct(), struct();
    'MC + radar bias (2 m, 0.02 deg)', 'MC+bias', struct(), struct()};
  T = struct('name', {}, 'R', {});
  rd = fullfile(fileparts(here), 'outputs');  if ~exist(rd, 'dir'), mkdir(rd); end
  fid = fopen(fullfile(rd, [tag '.txt']), 'w');
  hdr = sprintf('%-36s %7s %6s %9s %9s %9s %8s\n', 'condition', 'icpt', 'miss', 't_first', 'RMS med', 'RMS p95', 'I_peak?');
  fprintf(fid, 'seeds %d..%d (n = %d), frozen design\n%s', seeds(1), seeds(end), numel(seeds), hdr);  fprintf('%s', hdr);
  for q = 1:size(rows, 1)
    tc_settings('clear');  tc_settings('set', rows{q,4});
    fam = rows{q,2};  plus = '';  ip = strfind(fam, '+');
    if ~isempty(ip), plus = fam(ip:end);  fam = fam(1:ip-1); end
    opts = rows{q,3};  opts.keep_log = false;  n = numel(seeds);
    R = struct('success', false(1,n), 't_first', NaN(1,n), 'rms', NaN(1,n), 'n_pulses', zeros(1,n), 'n_hits', zeros(1,n), 'I_peak', NaN(1,n));
    for s = 1:n
      o = tc_run_engagement(sprintf('%s%d%s', fam, seeds(s), plus), opts);  m = o.metrics;
      R.success(s) = m.success;  R.t_first(s) = m.t_first_hit;  R.rms(s) = m.point_rms_m;
      R.n_pulses(s) = m.n_pulses;  R.n_hits(s) = m.n_hits;  R.I_peak(s) = m.I_peak;
    end
    tc_settings('clear');
    line = sprintf('%-36s %3d/%-3d %6d %8.2fs %9.3f %9.3f %7.1fA\n', rows{q,1}, sum(R.success), n, sum(R.n_pulses - R.n_hits), ...
                   mean(R.t_first(R.success)), median(R.rms), tc_percentile(R.rms, 95), max(R.I_peak));
    fprintf('%s', line);  fprintf(fid, '%s', line);  fflush(stdout);
    T(end+1) = struct('name', rows{q,1}, 'R', R); %#ok<AGROW>
  end
  fclose(fid);  save('-text', fullfile(rd, [tag '.mat']), 'T', 'seeds');
end
