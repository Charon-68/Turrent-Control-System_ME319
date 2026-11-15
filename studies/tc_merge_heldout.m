function R = tc_merge_heldout(tags, out)
% TC_MERGE_HELDOUT  Concatenate held-out chunks (run in parallel processes) into one result:
% outputs/<out>.mat / .txt, same format as tc_heldout_campaign.
  here = fileparts(mfilename('fullpath'));  addpath(fileparts(here));  tc_setup();
  rd = fullfile(fileparts(here), 'outputs');  R = [];
  for q = 1:numel(tags)
    A = load(fullfile(rd, [tags{q} '.mat']));  r = A.R;
    if isempty(R), R = r; else
      for f = {'seeds','success','t_first','rms','n_pulses','n_hits','in_reng','V_peak','miss_max'}
        R.(f{1}) = [R.(f{1}), r.(f{1})];
      end
    end
  end
  R.sum = tc_campaign_summary(R);  s = R.sum;
  fid = fopen(fullfile(rd, [out '.txt']), 'w');
  fprintf(fid, 'MC seeds %d..%d (n = %d), frozen design, 100 Hz, turret at 0 deg (chunks: %s)\n', R.seeds(1), R.seeds(end), s.n, strjoin(tags, ', '));
  fprintf(fid, 'intercepted %d/%d  Wilson95 [%.4f, %.4f]  failure bound < %.3f %%\n', s.intercepted, s.n, s.success_ci(1), s.success_ci(2), 100*s.failure_upper);
  fprintf(fid, 'pulses %d, hits %d, hit rate %.4f  Wilson95 [%.4f, %.4f]\n', s.pulses, s.pulse_hits, s.pulse_hit_rate, s.pulse_ci(1), s.pulse_ci(2));
  fprintf(fid, 'first hit (successful runs): mean %.3f s, p95 %.3f s, max %.3f s\n', s.t_first_mean, s.t_first_p95, s.t_first_max);
  fprintf(fid, 'pointing RMS: median %.3f m, p95 %.3f m;  time in R_eng %.2f %%;  V_peak %.0f V\n', s.rms_median, s.rms_p95, 100*s.in_reng_mean, s.V_peak);
  fprintf(fid, 'censored (no hit) seeds: %s\n', mat2str(R.seeds(~R.success)));
  fprintf(fid, 'worst miss distance over all pulses: %.3f m\n', max(R.miss_max));
  fclose(fid);  save('-text', fullfile(rd, [out '.mat']), 'R');  type(fullfile(rd, [out '.txt']));
end
