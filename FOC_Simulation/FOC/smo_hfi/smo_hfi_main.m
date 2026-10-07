function smo_hfi_main()
%SMO_HFI_MAIN  Smoke test, build the model and simulate, in one session.
here = fileparts(mfilename('fullpath'));
cd(here);
logf = fopen(fullfile(here, 'run_log.txt'), 'w', 'n', 'UTF-8');
if logf < 0
    error('smo_hfi_main:log', 'cannot open run_log.txt');
end
cleanup = onCleanup(@() fclose(logf));

say(logf, 'SH: === stage 1 smoke test ===\n');
try
    smoke_test();
    say(logf, 'SH: stage 1 OK\n');
catch e
    say(logf, 'SH: stage 1 FAILED\n%s\n', getReport(e, 'extended'));
    return;
end

say(logf, 'SH: === stage 2 build model ===\n');
try
    build_smo_hfi_model();
    say(logf, 'SH: stage 2 OK\n');
catch e
    say(logf, 'SH: stage 2 FAILED\n%s\n', getReport(e, 'extended'));
    return;
end

say(logf, 'SH: === stage 3 simulate ===\n');
try
    run_smo_hfi_sim();
    say(logf, 'SH: stage 3 OK\n');
catch e
    say(logf, 'SH: stage 3 FAILED\n%s\n', getReport(e, 'extended'));
    return;
end

say(logf, 'SH: === all stages done ===\n');
end

function say(fid, varargin)
fprintf(fid, varargin{:});
fprintf(varargin{:});
end
