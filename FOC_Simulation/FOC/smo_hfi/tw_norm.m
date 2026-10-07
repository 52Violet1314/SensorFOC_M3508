function [t, y] = tw_norm(v)
%TW_NORM  Normalise a To Workspace value into time and data arrays.
if isa(v, 'timeseries')
    t = v.Time;
    y = squeeze(v.Data);
elseif isstruct(v) && isfield(v, 'time') && isfield(v, 'signals')
    t = v.time;
    y = v.signals.values;
elseif isstruct(v) && isfield(v, 'time')
    t = v.time;
    y = v.data;
else
    t = (0:size(v,1)-1)';
    y = v;
end
if size(y,1) == 1 && size(y,2) > 1
    y = y';
end
if size(t,1) == 1 && size(t,2) > 1
    t = t';
end
end
