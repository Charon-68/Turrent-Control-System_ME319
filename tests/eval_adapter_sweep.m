function T = eval_adapter_sweep(scenarios, thetas, rates)
% EVAL_ADAPTER_SWEEP  eval_adapter over scenarios x start angles x call rates; prints one row per run
% and a miss/no-hit summary.  Defaults: A, B, C, MC201..MC203 (exploration seeds, never held-out),
% start angle 0, 0.3 rad and 80 deg, call rates 20, 100 and 1000 Hz.
  here = fileparts(mfilename('fullpath'));  addpath(fileparts(here));  addpath(here);
  if nargin < 1, scenarios = {'A', 'B', 'C', 'MC201', 'MC202', 'MC203'}; end
  if nargin < 2, thetas = [0, 0.3, 80*pi/180]; end
  if nargin < 3, rates = [20, 100, 1000]; end
  fprintf('%-6s %7s %6s | %9s %7s %7s %9s %9s\n', 'scn', 'theta0', 'f_Hz', 'first_hit', 'pulses', 'misses', 'rms_pt_m', 'max_pt_m');
  n = 0;  nohit = 0;  nmiss = 0;  T = {};
  for s = 1:numel(scenarios)
    for a = thetas
      for f = rates
        o = eval_adapter(scenarios{s}, f, a);
        n = n + 1;  nohit = nohit + ~isfinite(o.first_hit);  nmiss = nmiss + o.n_miss;
        T(end+1, :) = {scenarios{s}, a, f, o.first_hit, o.n_pulses, o.n_miss, o.rms_point, o.max_point}; %#ok<AGROW>
        flag = '';  if ~isfinite(o.first_hit), flag = '  NO HIT'; elseif o.n_miss > 0, flag = '  MISS'; end
        fprintf('%-6s %7.4f %6d | %9.3f %7d %7d %9.3f %9.3f%s\n', scenarios{s}, a, f, o.first_hit, o.n_pulses, o.n_miss, o.rms_point, o.max_point, flag);
        fflush(stdout);
      end
    end
  end
  fprintf('runs %d, runs without a hit %d, pulses that missed %d\n', n, nohit, nmiss);
end
