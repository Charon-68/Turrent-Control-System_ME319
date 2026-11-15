function [say, done] = tc_course_report(name)
% TC_COURSE_REPORT  Open outputs/<name>.txt for a course study; returns a printf-like writer and a closer
%   that also echoes the file to the console.  Study scripts write everything they report through it.
  here = fileparts(mfilename('fullpath'));  rd = fullfile(fileparts(here), 'outputs');
  if ~exist(rd, 'dir'), mkdir(rd); end
  fn = fullfile(rd, [name '.txt']);  fid = fopen(fn, 'w');
  say = @(varargin) fprintf(fid, varargin{:});
  done = @() close_and_type(fid, fn);
end

function close_and_type(fid, fn)
  fclose(fid);  type(fn);
end
