function eval_adapter_misses()
% EVAL_ADAPTER_MISSES  Re-run the cases that eval_adapter_sweep flagged MISS (read from
% outputs/eval_adapter_sweep.txt) and write every pulse of those runs, with its miss distance, to
% outputs/eval_adapter_misses.txt.  The start angle printed in the sweep (4 decimals) is snapped to the
% whole degree when it is within 1e-3 rad of one (the sweep used 80 deg = 1.3963 rad).
  here = fileparts(mfilename('fullpath'));  root = fileparts(here);  addpath(root);  addpath(here);
  L = strsplit(fileread(fullfile(root, 'outputs', 'eval_adapter_sweep.txt')), "\n");
  fid = fopen(fullfile(root, 'outputs', 'eval_adapter_misses.txt'), 'w');
  fprintf(fid, 'octave-cli --quiet --eval "tc_setup(); eval_adapter_misses()"\n');
  fprintf(fid, 'Cases flagged MISS in outputs/eval_adapter_sweep.txt, re-run; every pulse listed (t [s], miss distance [m], hit).\n');
  for k = 1:numel(L)
    if isempty(strfind(L{k}, 'MISS')), continue; end
    tk = regexp(L{k}, '^(\S+)\s+([\d.]+)\s+(\d+)\s*\|', 'tokens', 'once');
    scn = tk{1};  th = str2double(tk{2});  f = str2double(tk{3});
    if abs(th - round(th*180/pi)*pi/180) < 1e-3, th = round(th*180/pi)*pi/180; end
    o = eval_adapter(scn, f, th);
    fprintf(fid, 'CASE scn=%s theta0_deg=%.0f f_Hz=%d first_hit=%.3f pulses=%d misses=%d\n', scn, th*180/pi, f, o.first_hit, o.n_pulses, o.n_miss);
    for q = 1:size(o.pulses, 1)
      fprintf(fid, 'PULSE scn=%s f_Hz=%d t=%.3f miss_m=%.3f hit=%d\n', scn, f, o.pulses(q,1), o.pulses(q,2), o.pulses(q,3));
    end
  end
  fclose(fid);
end
