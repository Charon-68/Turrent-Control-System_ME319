function tc_independent_report()
% TC_INDEPENDENT_REPORT  Merge outputs/indep_1..5.mat into outputs/independent_check.txt.
  here = fileparts(mfilename('fullpath'));  root = fileparts(here);  addpath(root);  tc_setup();
  rd = fullfile(root, 'outputs');  fam = {'weave', 'step', 'bangbang', 'arc', 'popup'};
  fid = fopen(fullfile(rd, 'independent_check.txt'), 'w');
  say = @(varargin) fprintf(fid, varargin{:});
  say('INDEPENDENT CHECK of the frozen design (no design change was made because of these results)\n');
  say('Generator: simulation/tc_independent_generator.m (written from the assignment text only); runner: simulation/tc_independent_run.m (turret_controller +\n');
  say('turret_plant via RK4, own pulse judging).  Seeds 70000 + 1000*family + i, i = 1..100 (family 1..5).\n');
  say('Per case random: start bearing/range, speed, heading, maneuver parameters, ONE blackout (onset >= 1 s, dur <= 0.5 s),\n');
  say('initial turret angle in [-10, 85] deg.  Same case in all six configurations.  "new" = meas_valid 1 only on a new sample;\n');
  say('"held" = meas_valid 1 on every call with the last sample held (stale through blackouts).\n');
  say('Pointing RMS = cross-range error over t >= 2 s at the call instants.  first hit = mean over intercepted cases.\n\n');
  cfg = {}; Rs = cell(1, 5);
  for f = 1:5, A = load(fullfile(rd, sprintf('indep_%d.mat', f)));  Rs{f} = A.R; end
  rates = Rs{1}.rates;  convs = Rs{1}.convs;  j = 0;
  for f = 1:5, say('family %d (%s): %d cases, generator attempts mean %.1f max %d, |a| max %.1f m/s^2, jerk max %.0f m/s^3\n', ...
      f, fam{f}, numel(Rs{f}.seeds), mean(Rs{f}.attempts), max(Rs{f}.attempts), max(Rs{f}.a_max), max(Rs{f}.jerk_max)); end
  say('\n%-9s %-12s %5s %9s %9s %9s %6s %6s %9s %9s\n', 'family', 'config', 'n', 'intercept', 'first hit', 'p95 first', 'pulses', 'miss', 'RMS med', 'RMS p95');
  tot = zeros(6, 3);
  for f = 1:5
    R = Rs{f};  j = 0;
    for rt = 1:numel(rates)
      for cv = 1:numel(convs)
        j = j + 1;  n = numel(R.seeds);  ok = R.success(j,:);
        say('%-9s %4dHz %-5s %5d %5d/%-3d %8.2fs %8.2fs %6d %6d %9.3f %9.3f\n', fam{f}, rates(rt), convs{cv}, n, sum(ok), n, ...
            mean(R.t_first(j, ok)), tc_percentile(R.t_first(j, ok), 95), sum(R.n_pulses(j,:)), sum(R.n_miss(j,:)), median(R.rms(j,:)), tc_percentile(R.rms(j,:), 95));
        tot(j,:) = tot(j,:) + [sum(ok), n, sum(R.n_miss(j,:))];
      end
    end
  end
  say('\nPOOLED over the five families (500 cases per configuration):\n');
  j = 0;
  for rt = 1:numel(rates)
    for cv = 1:numel(convs)
      j = j + 1;  ci = tc_wilson_interval(tot(j,1), tot(j,2));
      say('  %4d Hz %-5s intercepted %d/%d (Wilson95 [%.4f, %.4f], failure < %.2f %%), pulse misses %d\n', rates(rt), convs{cv}, tot(j,1), tot(j,2), ci(1), ci(2), 100*(1 - ci(1)), tot(j,3));
    end
  end
  say('\nFAILURES (cases with no interception), by configuration:\n');
  nfail = 0;
  for f = 1:5
    R = Rs{f};  j = 0;
    for rt = 1:numel(rates)
      for cv = 1:numel(convs)
        j = j + 1;  bad = find(~R.success(j,:));
        for b = bad
          nfail = nfail + 1;
          say('  seed %d  family %s  %d Hz %s  start %.1f deg  blackout [%.2f s, %.2f s]  pulses %d (misses %d)\n', R.seeds(b), fam{f}, rates(rt), convs{cv}, R.theta0(b)*180/pi, R.bo(1,b), R.bo(2,b), R.n_pulses(j,b), R.n_miss(j,b));
        end
      end
    end
  end
  if nfail == 0, say('  none\n'); end
  say('\nCASES WITH AT LEAST ONE PULSE MISS (seed: configurations with misses -> count):\n');
  for f = 1:5
    R = Rs{f};  any_miss = find(any(R.n_miss > 0, 1));
    for b = any_miss
      js = find(R.n_miss(:, b) > 0)';  s = '';
      for j = js, [rt, cv] = ind2sub([numel(convs), numel(rates)], j); cv_i = mod(j-1, numel(convs)) + 1; rt_i = ceil(j/numel(convs));
        s = [s sprintf(' %dHz/%s->%d', rates(rt_i), convs{cv_i}, R.n_miss(j, b))]; end %#ok<AGROW>
      say('  seed %d (%s):%s\n', R.seeds(b), fam{f}, s);
    end
  end
  fclose(fid);  type(fullfile(rd, 'independent_check.txt'));
end
