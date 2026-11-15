function tc_setup()
% TC_SETUP  Put the project's folders on the path (idempotent, cheap after the first call).
% The submission files (turret_controller, turret_plant, target_motion_generator) sit in the
% project root and call this themselves, so copying the root plus core/ is all an evaluator needs.
  persistent done
  if isempty(done)
    here = fileparts(mfilename('fullpath'));
    for d = {'core', 'simulation', 'studies', 'tests'}
      if exist(fullfile(here, d{1}), 'dir'), addpath(fullfile(here, d{1})); end
    end
    done = true;
  end
end
