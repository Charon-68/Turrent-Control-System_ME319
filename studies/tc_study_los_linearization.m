function R = tc_study_los_linearization()
% TC_STUDY_LOS_LINEARIZATION  Linearise theta = atan2(y, x), propagate radar noise to cross-range error.
%   d(theta) = (x dy - y dx)/r^2  (first-order Taylor term of atan2).
%   1. numerical check: exact increment vs linear map for shrinking perturbations (error ~ eps^2);
%   2. radar noise (sigma_r, sigma_theta) -> Cartesian -> linear map: cross-range error = r sigma_theta,
%      range noise drops out to first order; compared with the allowed 2 m / r budget over 800..2000 m;
%   3. the existing error-budget numbers (outputs/error_budget.txt) are READ, never overwritten.
%   Writes outputs/tc_study_los_linearization.txt, plots/tc_study_los_linearization.{pdf,png}.
  here = fileparts(mfilename('fullpath'));  addpath(fileparts(here));  tc_setup();
  K = tc_params();  rd = fullfile(fileparts(here), 'outputs');
  [say, done] = tc_course_report('tc_study_los_linearization');
  say('LOS linearisation: theta = atan2(y,x), r^2 = x^2 + y^2\n');
  say('  d theta/dx = -y/r^2, d theta/dy = x/r^2  =>  d theta = (x dy - y dx)/r^2 = n.d / r,  n = [-sin th; cos th] (unit normal to the LOS)\n');
  say('  second-order term: -(x dx + y dy)(x dy - y dx)/r^4 + ..., so the relative error of the linear map is O(|d|/r)\n\n');

  % ---- 1. linear vs exact -------------------------------------------------------------------
  cases = [800 85; 800 30; 2000 30; 2000 -10; 1400 45]*diag([1 pi/180]);        % [r th]
  eps_list = logspace(-3, 2, 26);                                                % perturbation size [m]
  ang = [0 pi/4 pi/2 3*pi/4 pi 5*pi/4 3*pi/2 7*pi/4];                           % perturbation directions
  E = zeros(numel(eps_list), size(cases, 1));  Ref = zeros(size(E));
  for c = 1:size(cases, 1)
    r = cases(c,1);  th = cases(c,2);  x = r*cos(th);  y = r*sin(th);
    for q = 1:numel(eps_list)
      dx = eps_list(q)*cos(ang);  dy = eps_list(q)*sin(ang);
      ex = atan2(y + dy, x + dx) - atan2(y, x);  ln = (x*dy - y*dx)/r^2;
      E(q,c) = max(abs(ex - ln));  Ref(q,c) = max(abs(ln));
    end
  end
  say('1. Linear map vs exact atan2 increment (worst of 8 perturbation directions)\n');
  say('   %-16s', 'eps [m]');  for c = 1:size(cases,1), say('  r=%4.0f th=%3.0f deg', cases(c,1), cases(c,2)*180/pi); end, say('\n');
  for q = 1:3:numel(eps_list)
    say('   %-16.3g', eps_list(q));  for c = 1:size(cases,1), say('  %8.2e (%6.1e)  ', E(q,c), E(q,c)/Ref(q,c)); end, say('\n');
  end
  say('   (entries: absolute error [rad] and, in brackets, error / |linear increment|)\n');
  slope = zeros(1, size(cases,1));
  for c = 1:size(cases,1)
    i = find(E(:,c) > 1e-14 & eps_list(:) < 10);  pf = polyfit(log10(eps_list(i)), log10(E(i,c)), 1);  slope(c) = pf(1);
  end
  say('   fitted log-log slope of the error versus eps (expect 2, i.e. quadratic): %s\n', mat2str(slope, 3));
  q1 = find(eps_list >= 1, 1);
  say('   at a 1 m perturbation the relative error is at most %.2e (r >= 800 m); at 10 m %.2e\n', ...
      max(E(q1,:)./Ref(q1,:)), max(E(find(eps_list >= 10, 1),:)./Ref(find(eps_list >= 10, 1),:)));
  R.slope = slope;  R.rel_err_1m = max(E(q1,:)./Ref(q1,:));

  % ---- 2. radar noise through the linear map -------------------------------------------------------
  rr = 800:100:2000;  th0 = 30*pi/180;  rng_seed = 20440;  rand('state', rng_seed);  randn('state', rng_seed);
  N = 200000;  T = zeros(numel(rr), 8);
  for q = 1:numel(rr)
    r = rr(q);  x = r*cos(th0);  y = r*sin(th0);
    dr = K.sig_r*randn(1, N);  dth = K.sig_th*randn(1, N);
    dx = cos(th0)*dr - r*sin(th0)*dth;  dy = sin(th0)*dr + r*cos(th0)*dth;     % polar -> Cartesian, first order
    lin = (x*dy - y*dx)/r^2;                                                    % linear LOS map
    exa = atan2(y + dy, x + dx) - th0;                                         % exact LOS from the perturbed point
    lin_r = (x*(sin(th0)*dr) - y*(cos(th0)*dr))/r^2;                           % range noise alone through the map
    [~, Rz] = tc_convert_measurement(r, th0, K.sig_r, K.sig_th);  nb = [-sin(th0); cos(th0)];
    sig_conv = sqrt(nb'*Rz*nb)/r;                                              % what the filter assumes, as an angle
    T(q,:) = [r, K.sig_th, std(lin), std(exa), max(abs(lin_r)), r*K.sig_th, atan(K.r_eng/r), sig_conv];
  end
  say('\n2. Radar noise sigma_r = %g m, sigma_theta = %.4f deg through d theta = (x dy - y dx)/r^2  (theta = 30 deg, %d draws, seed %d)\n', K.sig_r, K.sig_th*180/pi, N, rng_seed);
  say('   In polar form dx = cos th dr - r sin th d th, dy = sin th dr + r cos th d th, so\n');
  say('   (x dy - y dx)/r^2 = d th exactly: range noise has NO first-order effect on the bearing, and the cross-range error is r sigma_theta.\n');
  say('   %6s %12s %12s %12s %14s %12s %12s %9s\n', 'r [m]', 'sd lin [mrad]', 'sd exact', 'range-only', 'r*sig_th [m]', 'allowed [m]', 'budget use', 'conv/raw');
  for q = 1:size(T,1)
    say('   %6.0f %12.5f %12.5f %12.2e %14.3f %12.3f %11.1f%% %9.5f\n', T(q,1), 1e3*T(q,3), 1e3*T(q,4), T(q,5), T(q,6), K.r_eng, 100*T(q,6)/K.r_eng, T(q,8)/T(q,2));
  end
  say('   sd lin/exact agree to %.2e relative (max over r); range-only contribution at most %.1e rad (round-off)\n', max(abs(T(:,3) - T(:,4))./T(:,4)), max(T(:,5)));
  say('   The raw radar cross-range error uses %.0f %% of the 2 m budget at 800 m and %.0f %% at 2000 m (1 sigma).\n', 100*T(1,6)/K.r_eng, 100*T(end,6)/K.r_eng);
  say('   Allowed angle atan(R_eng/r) is %.3f deg at 800 m and %.3f deg at 2000 m; sigma_theta = %.3f deg, i.e. %.1fx and %.1fx smaller.\n', ...
      atand(K.r_eng/800), atand(K.r_eng/2000), K.sig_th*180/pi, atand(K.r_eng/800)/(K.sig_th*180/pi), atand(K.r_eng/2000)/(K.sig_th*180/pi));
  say('   tc_convert_measurement covariance projected on the normal agrees with sigma_theta to within %.1e relative.\n', max(abs(T(:,8)/K.sig_th - 1)));

  % ---- 3. existing error-budget numbers (read, not overwritten) -------------------------------------
  fn = fullfile(rd, 'error_budget.txt');  EB = [];
  if exist(fn, 'file')
    L = strsplit(strtrim(fileread(fn)), "\n");
    for k = 2:numel(L)
      v = sscanf(L{k}, '%f');  if numel(v) == 5, EB(end+1,:) = v'; end %#ok<AGROW>
    end
  end
  say('\n3. Existing error budget (outputs/error_budget.txt, read-only here): angles in degrees, filtered = steady-state Kalman, blackout = after 0.5 s\n');
  if size(EB,1) == numel(rr) && all(EB(:,1) == rr(:))
    say('   %6s %10s %10s %10s %10s | %11s %11s %11s\n', 'r [m]', 'allowed', 'raw', 'filtered', 'blackout', 'filt [m]', 'blackout [m]', 'blk/allowed');
    for q = 1:size(EB,1)
      say('   %6.0f %10.4f %10.4f %10.4f %10.4f | %11.3f %11.3f %11.2f\n', EB(q,:), EB(q,4)*pi/180*EB(q,1), EB(q,5)*pi/180*EB(q,1), EB(q,5)/EB(q,2));
    end
    say('   The raw column of that file (%.4f deg) equals the sigma_theta used above (%.4f deg).\n', EB(1,3), K.sig_th*180/pi);
    say('   Filtered cross-range error is %.2f-%.2f of the raw value; after a 0.5 s blackout it is %.2f m (800 m) to %.2f m (2000 m) 1-sigma versus the 2 m budget.\n', ...
        min(EB(:,4)./EB(:,3)), max(EB(:,4)./EB(:,3)), EB(1,5)*pi/180*800, EB(end,5)*pi/180*2000);
  else
    say('   outputs/error_budget.txt not found or in an unexpected layout: not compared.\n');
  end
  done();

  % ---- figure ---------------------------------------------------------------------------------------
  h = figure('visible', 'off', 'position', [50 50 1100 430], 'color', 'w');
  subplot(1,2,1);  loglog(eps_list, E, 'linewidth', 1.4);  hold on;
  loglog(eps_list, 0.5*eps_list.^2/800^2, 'k--', 'linewidth', 1.2);  hold off;  grid on;
  xlabel('perturbation size |d| [m]');  ylabel('| exact - linear | [rad]');
  title('LOS map: linear error grows as |d|^2 (dashed: |d|^2 / 2 r^2, r = 800 m)');
  legend([arrayfun(@(c) sprintf('r=%.0f m, th=%.0f deg', cases(c,1), cases(c,2)*180/pi), 1:size(cases,1), 'UniformOutput', false), {'|d|^2/(2 \cdot 800^2)'}], 'location', 'southeast');
  subplot(1,2,2);  plot(T(:,1), T(:,6), 'r--', 'linewidth', 1.6);  hold on;
  plot(T(:,1), K.r_eng*ones(size(T(:,1))), 'k-', 'linewidth', 2);
  if ~isempty(EB)
    plot(EB(:,1), EB(:,4)*pi/180.*EB(:,1), 'b-', 'linewidth', 1.6);  plot(EB(:,1), EB(:,5)*pi/180.*EB(:,1), 'm-.', 'linewidth', 1.6);
  end
  hold off;  grid on;  xlabel('range r [m]');  ylabel('cross-range error, 1\sigma [m]');
  title('Cross-range error r \sigma_\theta against the 2 m engagement radius');
  legend({'raw radar r\sigma_\theta', 'R_{eng} = 2 m', 'filtered (existing budget)', 'after 0.5 s blackout (existing)'}, 'location', 'northwest');
  tc_save_figure(h, 'tc_study_los_linearization');
  R.table = T;  R.eb = EB;
  save('-mat', fullfile(rd, 'tc_study_los_linearization.mat'), 'R');
end
