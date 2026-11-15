function ok = eval_syntax_check()
% EVAL_SYNTAX_CHECK  Static scan: the submission files (and everything they call) use no Octave-only syntax.
%
% Scans turret_controller.m, turret_plant.m, target_motion_generator.m, tc_setup.m and core/*.m after
% removing comments and single-quoted strings.  This is a text check, NOT a MATLAB run: it cannot prove
% MATLAB compatibility, it only catches the usual Octave-only constructs.  A self-test first feeds the
% scanner deliberately bad lines and requires every one to be flagged, and good lines to pass.
  here = fileparts(mfilename('fullpath'));  root = fileparts(here);
  pats = {'\)\(',  'chained call or index'; ...
          '\}\{',  'chained brace index'; ...
          '\+=|-=|\*=|/=|\^=', 'compound assignment'; ...
          '\+\+|--[ \t]*;',    'increment/decrement'; ...
          '(^|[^fs])printf\(', 'printf'; ...
          '\<end(if|for|while|function|switch|_try_catch|_unwind_protect)\>', 'Octave block terminator'; ...
          '\<(unwind_protect|do)\>[ \t]*$', 'Octave-only block'; ...
          '\<until\>', 'do-until'; ...
          '#', 'hash comment'; ...
          '(^|[^~=<>])!(?!=)', '! negation'; ...
          '!=', '!= operator'; ...
          '\*\*', '** power'; ...
          '"', 'double-quoted string'; ...
          '\<(puts|fputs|fdisp|disp_)\(', 'Octave output helper'};
  bad_lines = {'x += 1;', 'y = a(1)(2);', 'if !ok', 'endif', 'printf("hi")', 'a != b', 'z = 2**3;', '# comment', 'q = "str";'};
  good_lines = {'x = x + 1;', 'if ~ok', 'end', 'fprintf(''hi'')', 'a ~= b', 'z = 2^3;', '% comment', 'q = ''str'';  % "x"'};
  for q = 1:numel(bad_lines)
    if isempty(scan_line(bad_lines{q}, pats)), error('eval_syntax_check self-test: not flagged: %s', bad_lines{q}); end
  end
  for q = 1:numel(good_lines)
    if ~isempty(scan_line(good_lines{q}, pats)), error('eval_syntax_check self-test: wrongly flagged: %s', good_lines{q}); end
  end
  files = {fullfile(root, 'turret_controller.m'), fullfile(root, 'turret_plant.m'), ...
           fullfile(root, 'target_motion_generator.m'), fullfile(root, 'tc_setup.m')};
  d = dir(fullfile(root, 'core', '*.m'));
  for q = 1:numel(d), files{end+1} = fullfile(root, 'core', d(q).name); end %#ok<AGROW>
  bad = {};
  for q = 1:numel(files)
    L = strsplit(fileread(files{q}), "\n");
    for k = 1:numel(L)
      hit = scan_line(L{k}, pats);
      for j = 1:numel(hit), bad{end+1} = sprintf('%s:%d (%s)', files{q}, k, hit{j}); end %#ok<AGROW>
    end
  end
  ok = isempty(bad);
  fprintf('eval_syntax_check: %d files scanned, %d findings, self-test %d bad/%d good lines\n', numel(files), numel(bad), numel(bad_lines), numel(good_lines));
  if ~ok, error('eval_syntax_check: Octave-only syntax: %s', strjoin(bad, '; ')); end
end

function hit = scan_line(line, pats)
  ln = regexprep(line, '''[^'']*''', '''''');      % drop single-quoted strings
  ln = regexprep(ln, '%.*$', '');                  % drop comments
  hit = {};
  for p = 1:size(pats, 1)
    if ~isempty(regexp(ln, pats{p,1}, 'once')), hit{end+1} = pats{p,2}; end %#ok<AGROW>
  end
end
