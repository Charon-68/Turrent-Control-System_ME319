function t = tc_timing_study(n)
% TC_TIMING_STUDY  Wall-clock cost per controller call on scenario B at 1 kHz.
  here = fileparts(mfilename('fullpath'));  addpath(fileparts(here));  tc_setup();
  if nargin < 1, n = 5000; end
  clear turret_controller;  th = 0;  thd = 0;  t = zeros(1, n);
  for k = 0:n-1
    tt = k*1e-3;  [r, a, v] = target_motion_generator(tt, 'B');
    t0 = tic;  turret_controller(tt, r, a, v, th, thd);  t(k+1) = toc(t0);
  end
  line = sprintf('turret_controller: mean %.3f ms, median %.3f ms, p99 %.3f ms over %d calls (scenario B, 1 kHz, GNU Octave %s)\n', 1e3*mean(t), 1e3*median(t), 1e3*tc_percentile(t, 99), n, version());
  fprintf('%s', line);
  rd = fullfile(fileparts(here), 'outputs');  if ~exist(rd, 'dir'), mkdir(rd); end
  fid = fopen(fullfile(rd, 'time.txt'), 'w');  fprintf(fid, '%s', line);  fprintf(fid, '%.6f %.6f %.6f\n', mean(t), median(t), tc_percentile(t, 99));  fclose(fid);
end
