function out = tc_settings(action, patch)
% TC_SETTINGS  Session-wide parameter overrides (experiments only).
%   tc_settings('set', struct('p_dev', 30))   merge overrides
%   tc_settings('get')                         current overrides
%   tc_settings('clear')                       back to the frozen design
% The frozen design is whatever tc_params() returns with no overrides; the
% submission never calls this function.  Experiments and ablations do, and
% must clear afterwards (the test-suite and simulation/ scripts do it in try/catch).
  persistent ovr
  if isempty(ovr), ovr = struct(); end
  if nargin < 1, action = 'get'; end
  switch action
    case 'set'
      f = fieldnames(patch);
      for k = 1:numel(f), ovr.(f{k}) = patch.(f{k}); end
    case 'clear'
      ovr = struct();
    case 'get'
    otherwise
      error('tc_settings: unknown action %s', action);
  end
  out = ovr;
end
