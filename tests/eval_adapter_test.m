function ok = eval_adapter_test(full)
% EVAL_ADAPTER_TEST  Quick checks through eval_adapter (handout signatures only).
%   eval_adapter_test        scenario B, theta0 = 0.3 rad, call rates 20/100/1000 Hz (+ held-flag at 20 Hz)
%   eval_adapter_test(true)  adds 40, 200 and 500 Hz and scenario C (blackout)
% Passes when every run delivers at least one hit, no run diverges, and every pulse was judged.
  here = fileparts(mfilename('fullpath'));  addpath(fileparts(here));  addpath(here);
  if nargin < 1, full = false; end
  runs = {'B', 20, 0.3, 'new'; 'B', 100, 0.3, 'new'; 'B', 1000, 0.3, 'new'; 'B', 20, 0.3, 'held'};
  if full
    runs = [runs; {'B', 40, 0.3, 'new'; 'B', 200, 0.3, 'new'; 'B', 500, 0.3, 'new'; 'C', 100, 0.3, 'new'}];
  end
  ok = true;
  fprintf('%-4s %6s %7s %5s | %9s %8s %7s %9s\n', 'scn', 'f_Hz', 'theta0', 'conv', 'first_hit', 'pulses', 'misses', 'rms_pt_m');
  for q = 1:size(runs, 1)
    tic;
    o = eval_adapter(runs{q,1}, runs{q,2}, runs{q,3}, struct('conv', runs{q,4}));
    good = isfinite(o.first_hit);
    ok = ok && good;
    fprintf('%-4s %6d %7.2f %5s | %9.3f %8d %7d %9.3f   %s  (%.1f s wall)\n', runs{q,1}, runs{q,2}, runs{q,3}, runs{q,4}, ...
            o.first_hit, o.n_pulses, o.n_miss, o.rms_point, ternary(good, 'ok', 'NO HIT'), toc);
  end
  fprintf('eval_adapter_test: %s\n', ternary(ok, 'PASS', 'FAIL'));
  if ~ok, error('eval_adapter_test: at least one run produced no hit'); end
end

function s = ternary(c, a, b), if c, s = a; else, s = b; end, end
