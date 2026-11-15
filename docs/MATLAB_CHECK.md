# MATLAB check (UNTESTED)

**Status: none of the steps below has been run in MATLAB.** The project was developed and tested in GNU Octave 11.3.0 only.
The submission files were checked for MATLAB compatibility *by reading* (`tests/tc_test_all.m` guard `t_submission_portable`,
`tests/eval_syntax_check.m`, `docs/DECISIONS.md` D9). This file is a procedure for someone with MATLAB to run; any
difference from the Octave numbers below is a finding to report, not something already known.

## 1. What to upload to MATLAB Online

Create one project folder (any name) and upload, keeping the folder layout:

```
turret_controller.m
turret_plant.m
target_motion_generator.m
tc_setup.m
core/            (all 33 .m files)
tests/eval_adapter.m
tests/eval_adapter_test.m
tests/eval_syntax_check.m
```
Nothing else is needed for the three submission files. `simulation/`, `studies/` and the rest of `tests/` are optional: `tc_setup.m` adds
them to the path only if they exist. (`eval_adapter.m` itself calls only the three submission functions.)
Set the MATLAB Online *Current Folder* to the project root (the folder containing `turret_controller.m`).

## 2. Commands (type in the Command Window, in this order)

```matlab
addpath(pwd); addpath(fullfile(pwd,'tests'));
tc_setup();

% (a) the static syntax scan, runs under MATLAB too
eval_syntax_check()

% (b) one engagement through the three submission files only
o = eval_adapter('B', 100, 0);
fprintf('first_hit %.3f s, pulses %d, misses %d, rms %.4f m\n', o.first_hit, o.n_pulses, o.n_miss, o.rms_point);

% (c) the short sweep (B from 0.3 rad at 20/100/1000 Hz and a 20 Hz held-flag run)
eval_adapter_test()
```
Run `clear all` between attempts if a path or persistent-variable problem is suspected (the controller keeps persistent state;
`eval_adapter` clears it at the start of each run).

## 3. What to compare with the Octave results

These are the recorded Octave 11.3.0 numbers (re-run them in Octave first if you need current values):

| Command | Octave result recorded |
|---|---|
| `eval_syntax_check()` | 37 files scanned (full tree; the count differs with the upload subset), 0 findings, self-test 9 bad / 8 good lines flagged correctly |
| `eval_adapter('B',100,0)` | first hit 0.940 s, 9 pulses, rms 0.5127 m (same as `tc_run_engagement`) |
| `eval_adapter_test()` rows (B, theta0 = 0.3 rad) | first hit 0.900 s at 20 Hz, 0.510 s at 100 Hz, 0.550 s at 1000 Hz, 0.900 s at 20 Hz held flag; 0 misses; final line `PASS` |

The numbers should agree to the printed precision. A different first-hit time, a different pulse count or an error message is a
real discrepancy. Slow runs are expected (the 1000 Hz row is the longest).

## 4. If something fails

- Copy the full error text and the line it names; report which file and which construct.
- Likely places (not confirmed): syntax that only Octave accepts, function-handle or cell-array indexing differences, `persistent`
  behaviour after `clear`. D9 lists the two Octave-only constructs already found and fixed.
- Do not edit the three handout files to work around it without saying so.

## 5. Not covered by this check

The full test-suite (`make test`), the Monte Carlo campaigns and the studies are Octave-only workflows (they use the Makefile);
this file covers only the submission path and the evaluation stand-in.
