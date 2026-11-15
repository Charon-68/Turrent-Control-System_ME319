function R = tc_study_plant_response()
% TC_STUDY_PLANT_RESPONSE  Impulse and step response of G(s) from its poles (partial fractions) and the
%   electrical-mechanical analogy table (docs/ANALOGY_TABLE.md).
%   G(s) = gn / ( s (a2 s^2 + a1 s + a0) ),  poles 0 and -sigma +- j wd.
%   Cross-check: the same responses from a direct RK4 integration of the physical states (theta, omega, i).
%   Writes outputs/tc_study_plant_response.txt, plots/tc_study_plant_response.{pdf,png}, docs/ANALOGY_TABLE.md.
  here = fileparts(mfilename('fullpath'));  root = fileparts(here);  addpath(root);  tc_setup();
  M = tc_course_model();  K = M.K;  a = M.a;
  [say, done] = tc_course_report('tc_study_plant_response');
  p = roots(M.Gd);  [~, i0] = min(abs(p));  pc = p(imag(p) > 0);          % pole at 0 and the pair
  sg = -real(pc);  wd = imag(pc);  wn = abs(pc);  zt = sg/wn;
  say('G(s) = %.0f / ( s (%.4g s^2 + %.4g s + %.6g) )\n', M.gn, a);
  say('Poles: 0 and %.4f %+.4fj  (conjugate pair)\n', real(pc), imag(pc));
  say('  natural frequency wn = %.3f rad/s (%.3f Hz), damping ratio zeta = %.4f\n', wn, wn/2/pi, zt);
  say('  damped oscillation frequency wd = %.3f rad/s (%.3f Hz), period %.4f s\n', wd, wd/2/pi, 2*pi/wd);
  say('  decay rate sigma = %.3f 1/s: envelope time constant 1/sigma = %.3f s, 2%% settling of the envelope 4/sigma = %.2f s, amplitude factor per cycle exp(-sigma T) = %.3f\n', sg, 1/sg, 4/sg, exp(-sg*2*pi/wd));
  say('  check wn^2 = a0/a2 = %.2f (pole modulus^2 %.2f); 2 zeta wn = a1/a2 = %.4f (pole pair %.4f)\n', a(3)/a(1), wn^2, a(2)/a(1), 2*sg);
  [r, pp, ~, ee] = residue(M.gn, M.Gd);
  ip = find(imag(pp) > 0);  iz = find(abs(pp) < 1e-9);
  A = 2*abs(r(ip));  ph = angle(r(ip));
  say('\nImpulse response g(t) (unit impulse of volts, V s):\n');
  say('  g(t) = %.4e + %.4e exp(-%.3f t) cos(%.3f t %+.4f rad)   [rad per V s]\n', real(r(iz)), A, sg, wd, ph);
  say('  The constant is the final angle per volt-second gn/a0 = %.4e rad; the oscillation (%.1f %% of it at peak) rides on top and is gone by about t = %.2f s.\n', M.gn/a(3), 100*A/(M.gn/a(3)), 4/sg);
  [rs, ps, ~, es] = residue(M.gn, conv(M.Gd, [1 0]));
  jz = find(abs(ps) < 1e-9 & es == 2);  jz1 = find(abs(ps) < 1e-9 & es == 1);  jp = find(imag(ps) > 0);
  say('Step response y(t) (1 V step):\n');
  say('  y(t) = %.4e t %+.4e + %.4e exp(-%.3f t) cos(%.3f t %+.4f rad)   [rad]\n', real(rs(jz)), real(rs(jz1)), 2*abs(rs(jp)), sg, wd, angle(rs(jp)));
  say('  -> asymptotically a ramp at %.4e rad/s per volt (integrator: a constant voltage gives a constant turret rate, cruise at 600 V = %.4f rad/s)\n', real(rs(jz)), K.omega_cruise);

  t = 0:0.0005:1.5;
  g = tc_lin_response(M.Gn, M.Gd, 'imp', 1, t);  y = tc_lin_response(M.Gn, M.Gd, 'step', 1, t);
  % direct integration of the physical states: theta' = w, J w' = N Kt i - b w, L i' = V - R i - N Ke w
  J = K.J_eq;  f = @(x, V) [x(2); (K.N*K.Kt*x(3) - K.b*x(2))/J; (V - K.R*x(3) - K.N*K.Ke*x(2))/K.L];
  h = 1e-4;  nst = round(t(end)/h);  X1 = zeros(3, nst+1);  X2 = X1;  X2(:,1) = [0; 0; 1/K.L];  x1 = zeros(3,1);  x2 = X2(:,1);
  for k = 1:nst
    k1 = f(x1, 1); k2 = f(x1 + h/2*k1, 1); k3 = f(x1 + h/2*k2, 1); k4 = f(x1 + h*k3, 1);  x1 = x1 + h/6*(k1 + 2*k2 + 2*k3 + k4);  X1(:,k+1) = x1;
    k1 = f(x2, 0); k2 = f(x2 + h/2*k1, 0); k3 = f(x2 + h/2*k2, 0); k4 = f(x2 + h*k3, 0);  x2 = x2 + h/6*(k1 + 2*k2 + 2*k3 + k4);  X2(:,k+1) = x2;
  end
  tt = (0:nst)*h;  ys = interp1(tt, X1(1,:), t);  gs = interp1(tt, X2(1,:), t);
  say('\nCross-check against direct RK4 of the physical states (h = %g s, %g s window): max |step difference| = %.2e rad (%.2e of peak), max |impulse difference| = %.2e rad (%.2e of peak)\n', ...
      h, t(end), max(abs(ys - y)), max(abs(ys - y))/max(abs(y)), max(abs(gs - g)), max(abs(gs - g))/max(abs(g)));
  say('  (the impulse is the zero-input response from i(0+) = 1/L = %.0f A, theta = omega = 0)\n', 1/K.L);
  yo = y - M.gn/a(3)*t;  [pk, ipk] = max(yo);  
  say('Step minus its ramp asymptote: first peak %.4e rad at t = %.4f s (half period %.4f s); minimum later %.4e rad\n', pk, t(ipk), pi/wd, min(yo(t > t(ipk))));
  done();

  hf = figure('visible', 'off', 'position', [50 50 1500 450], 'color', 'w');
  env = A*exp(-sg*t);  env2 = 2*abs(rs(jp))*exp(-sg*t);
  ramp = M.gn/a(3)*t + real(rs(jz1));  tm = t <= 0.6;
  subplot(1,3,1);  plot(t(tm), g(tm), 'b', 'linewidth', 1.5);  hold on;  plot(t(tm), M.gn/a(3) + env(tm), 'r--', 'linewidth', 1.0);  plot(t(tm), M.gn/a(3) - env(tm), 'r--', 'linewidth', 1.0, 'handlevisibility', 'off');
  plot(t(tm), gs(tm), 'k:', 'linewidth', 1.0);  hold off;  grid on;  xlabel('t [s]');  ylabel('g(t) [rad / (V s)]');
  title(sprintf('Impulse: wd = %.1f rad/s, decay %.2f 1/s', wd, sg));  legend({'g(t)', 'envelope', 'RK4 states'}, 'location', 'northeast');
  subplot(1,3,2);  plot(t, y, 'b', 'linewidth', 1.5);  hold on;  plot(t, ys, 'k:', 'linewidth', 1.0);  hold off;  grid on;
  xlabel('t [s]');  ylabel('y(t) [rad per 1 V step]');  title('Step response (ramp at gn/a0 rad/s per V)');  legend({'partial fractions', 'RK4 states'}, 'location', 'northwest');
  subplot(1,3,3);  plot(t(tm), (y(tm) - ramp(tm))*1e6, 'b', 'linewidth', 1.5);  hold on;  plot(t(tm), env2(tm)*1e6, 'r--', t(tm), -env2(tm)*1e6, 'r--', 'linewidth', 1.0);  hold off;  grid on;
  xlabel('t [s]');  ylabel('y - ramp asymptote [10^{-6} rad]');  title('Step minus ramp: the damped oscillation');
  tc_save_figure(hf, 'tc_study_plant_response');

  % ---- analogy table ------------------------------------------------------------------------------
  tau_e = K.L/K.R;  tau_m = J/K.b;  c2 = K.N^2*K.Kt*K.Ke;
  fid = fopen(fullfile(root, 'docs', 'ANALOGY_TABLE.md'), 'w');
  fprintf(fid, '# Electrical-mechanical analogy for the elevation drive\n\n');
  fprintf(fid, 'Generated by `studies/tc_study_plant_response.m` from `core/tc_params.m` (nothing typed by hand). Plant: `G(s) = %.0f / ( s (%.4g s^2 + %.4g s + %.6g) )`.\n\n', M.gn, a);
  fprintf(fid, '## Element-by-element (torque-voltage / mass-inductor analogy)\n\n');
  fprintf(fid, '| Mechanical side (turret) | Electrical side (armature) | Mechanical value | Electrical value | Role in G(s) |\n|---|---|---|---|---|\n');
  fprintf(fid, '| inertia `J_eq = J_t + N^2 J_m` | inductance `L` | %.0f kg m^2 | %.2f H | both store kinetic / magnetic energy; leading coefficient `a2 = L J_eq` = %.0f |\n', J, K.L, a(1));
  fprintf(fid, '| viscous damping `b` | resistance `R` | %.0f N m s/rad | %.2f ohm | dissipate energy; `a0` contains `R b` = %.0f |\n', K.b, K.R, K.R*K.b);
  fprintf(fid, '| torque `T` | voltage `V` | `T = N K_t i` | `V` (input) | `N K_t` = %.0f N m/A scales current into turret torque |\n', K.N*K.Kt);
  fprintf(fid, '| angular velocity `omega` | current `i` | `omega = theta''` | `i` | the two flow variables |\n');
  fprintf(fid, '| (no spring) | (no capacitor) | -- | -- | no restoring torque, hence the free integrator pole at s = 0 |\n');
  fprintf(fid, '| coupling | back-emf / motor torque | `N K_t` = %.0f N m/A and `N K_e` = %.0f V s/rad | -- | gyrator: the loop term `N^2 K_t K_e` = %.4g is what turns two lags into a resonance |\n\n', K.N*K.Kt, K.N*K.Ke, c2);
  fprintf(fid, '## Time constants and the resonance\n\n');
  fprintf(fid, '| Quantity | Formula | Value |\n|---|---|---|\n');
  fprintf(fid, '| electrical time constant | `L/R` | %.3f s |\n| mechanical time constant | `J_eq/b` | %.3f s |\n', tau_e, tau_m);
  fprintf(fid, '| coupling ratio | `N^2 K_t K_e / (R b)` | %.1f (strongly coupled: back-emf dominates the damping) |\n', c2/(K.R*K.b));
  fprintf(fid, '| undamped natural frequency | `sqrt((R b + N^2 K_t K_e)/(L J_eq))` | %.2f rad/s (%.2f Hz) |\n', wn, wn/2/pi);
  fprintf(fid, '| damping ratio | `(L b + R J_eq) / (2 sqrt(L J_eq (R b + N^2 K_t K_e)))` | %.4f |\n', zt);
  fprintf(fid, '| decay rate of the oscillation | `(L b + R J_eq)/(2 L J_eq)` | %.3f 1/s (time constant %.3f s) |\n', sg, 1/sg);
  fprintf(fid, '| integrator gain (rate per volt) | `N K_t / a0` | %.4e rad/s per V |\n\n', M.gn/a(3));
  fprintf(fid, '## Mechanics reflected into the armature circuit\n\n');
  fprintf(fid, 'Eliminating `omega` gives `V = (L s + R) i + Z_m i`, with motional impedance `Z_m = N^2 K_t K_e / (J_eq s + b)`, i.e. a capacitor and a conductance in parallel:\n\n');
  fprintf(fid, '| Reflected element | Formula | Value |\n|---|---|---|\n');
  fprintf(fid, '| series inductance | `L` | %.2f H |\n| series resistance | `R` | %.2f ohm |\n', K.L, K.R);
  fprintf(fid, '| parallel capacitance (the inertia) | `J_eq/(N^2 K_t K_e)` | %.4g F |\n', J/c2);
  fprintf(fid, '| parallel resistance (the damping) | `N^2 K_t K_e / b` | %.1f ohm |\n\n', c2/K.b);
  fprintf(fid, 'So the inertia appears as a capacitor seen from the electrical terminals and as an inductor in the torque-voltage analogy above: both are correct, they use different analogies (impedance versus mobility). The series `L` with this parallel `C` is the lightly damped %.1f rad/s resonance.\n\n', wn);
  fprintf(fid, '## Time response read-off\n\n');
  fprintf(fid, 'Impulse response oscillates at wd = %.2f rad/s (%.2f Hz) with the envelope decaying as exp(-%.3f t); the step response is a ramp at %.4e rad/s per volt plus the same oscillation (see `outputs/tc_study_plant_response.txt`, `plots/tc_study_plant_response.*`).\n', wd, wd/2/pi, sg, M.gn/a(3));
  fclose(fid);
  R.wn = wn;  R.zeta = zt;  R.wd = wd;  R.sigma = sg;
end
