function tc_save_figure(h, name)
% TC_SAVE_FIGURE  Write plots/<name>.pdf and .png.  GNU Octave with the fltk/qt toolkits can only
% print a VISIBLE figure, so a window may flash briefly; it is closed afterwards.
  here = fileparts(mfilename('fullpath'));  fd = fullfile(fileparts(here), 'plots');
  if ~exist(fd, 'dir'), mkdir(fd); end
  pos = get(h, 'position');  w = pos(3)/100;  hh = pos(4)/100;      % size the page to the figure: no cropping
  set(h, 'paperunits', 'inches', 'papersize', [w hh], 'paperposition', [0 0 w hh]);
  set(h, 'visible', 'on');  drawnow;
  print(h, fullfile(fd, [name '.pdf']), '-dpdf');
  print(h, fullfile(fd, [name '.png']), '-dpng', '-r130');
  close(h);
end
