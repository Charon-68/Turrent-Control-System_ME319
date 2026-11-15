function R = tc_independent_campaign(family, ncases)
% TC_INDEPENDENT_CAMPAIGN  Frozen design on ncases independent cases of one family, at 20/100/1000 Hz and
% with both meas_valid conventions (same trajectory, blackout, noise and start angle in all six
% configurations).  Seeds: 70000 + 1000*family + i, i = 1..ncases.  Writes outputs/indep_<family>.mat
  here = fileparts(mfilename('fullpath'));  root = fileparts(here);  addpath(root);  tc_setup();
  if nargin < 2, ncases = 100; end
  tc_settings('clear');                       % frozen design: no overrides
  rates = [20 100 1000];  convs = {'new', 'held'};
  nc = numel(rates)*numel(convs);
  R = struct('family', family, 'seeds', 70000 + 1000*family + (1:ncases), 'rates', rates, 'convs', {convs}, ...
             'success', false(nc, ncases), 't_first', NaN(nc, ncases), 'n_pulses', zeros(nc, ncases), ...
             'n_miss', zeros(nc, ncases), 'rms', NaN(nc, ncases), 'attempts', zeros(1, ncases), ...
             'theta0', zeros(1, ncases), 'bo', zeros(2, ncases), 'jerk_max', zeros(1, ncases), 'a_max', zeros(1, ncases));
  tic;
  for i = 1:ncases
    c = tc_independent_generator(family, R.seeds(i));
    R.attempts(i) = c.attempts;  R.theta0(i) = c.theta0;  R.bo(:, i) = c.bo(:);
    R.jerk_max(i) = c.jerk_max;  R.a_max(i) = c.extremes(4);
    j = 0;
    for f = rates
      for cv = 1:numel(convs)
        j = j + 1;
        o = tc_independent_run(c, f, convs{cv});
        R.success(j, i) = o.success;  R.t_first(j, i) = o.t_first;  R.n_pulses(j, i) = o.n_pulses;
        R.n_miss(j, i) = o.n_miss;  R.rms(j, i) = o.rms;
      end
    end
    if mod(i, 10) == 0, fprintf('family %d: %d/%d (%.0f s)\n', family, i, ncases, toc); fflush(stdout); end
  end
  rd = fullfile(root, 'outputs');  if ~exist(rd, 'dir'), mkdir(rd); end
  save('-text', fullfile(rd, sprintf('indep_%d.mat', family)), 'R');
end
