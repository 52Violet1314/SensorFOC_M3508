function smoke_test()
%SMOKE_TEST  Prove that a MATLAB Function block can be created, scripted and
% simulated from the command line before building the real model.
mdl = 'SmokeEmChart';
if bdIsLoaded(mdl)
    close_system(mdl, 0);
end
if exist([mdl '.slx'], 'file')
    delete([mdl '.slx']);
end
new_system(mdl);
add_block('simulink/Sources/Constant', [mdl '/C'], 'Value', '2', 'Position', [40 100 80 130]);
blk = [mdl '/MF'];
add_block('simulink/User-Defined Functions/MATLAB Function', blk, 'Position', [150 90 250 140]);
ch = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', blk);
fprintf('SH: charts found = %d\n', numel(ch));
ch(1).Script = sprintf(['function y = fcn(u)\n' ...
    '%%#codegen\n' ...
    'persistent s\n' ...
    'if isempty(s)\n' ...
    '    s = 0;\n' ...
    'end\n' ...
    's = s + u;\n' ...
    'y = 2 * s;\n' ...
    'end\n']);
pr = get_param(blk, 'Ports');
fprintf('SH: smoke chart ports in=%d out=%d\n', pr(1), pr(2));
add_block('simulink/Sinks/To Workspace', [mdl '/W'], 'VariableName', 'smoke_y', ...
    'SaveFormat', 'Structure With Time', 'Position', [300 90 380 140]);
add_line(mdl, 'C/1', 'MF/1', 'autorouting', 'on');
add_line(mdl, 'MF/1', 'W/1', 'autorouting', 'on');
set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
    'FixedStep', '1e-3', 'StopTime', '0.01');
out = sim(mdl);
[tv, yv] = tw_norm(out.smoke_y);
fprintf('SH: smoke final value = %g (expect 44 = 4*k at k=11), samples = %d\n', yv(end), numel(tv));
try
    set_param(mdl, 'Dirty', 'off');
    close_system(mdl, 0);
catch
end
if exist([mdl '.slx'], 'file')
    delete([mdl '.slx']);
end
end
