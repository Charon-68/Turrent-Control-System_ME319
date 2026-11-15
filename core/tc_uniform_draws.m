function u = tc_uniform_draws(seed, n)
% TC_UNIFORM_DRAWS  n reproducible uniform(0,1) draws from an integer seed WITHOUT disturbing the
% caller's random stream (state saved and restored).  Scenario generation uses this so a
% user's own rand/randn sequence is never consumed by the test harness.
  st = rand('twister');
  rand('twister', double(mod(seed, 2^32)));
  u = rand(n, 1);
  rand('twister', st);
end
