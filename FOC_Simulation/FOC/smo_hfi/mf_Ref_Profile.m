function [w_ref, TL] = mf_Ref_Profile(t)
%#codegen
% Mechanical speed reference [rad/s] and load torque [N m].
% 0.00 - 0.05 s : standstill, HFI has to hold the angle on its own
% 0.05 - 0.35 s : ramp up to 120 rad/s
% 0.35 - 0.50 s : hold, with a load torque step at 0.42 s
T_STAND = 0.05;
T_RAMP  = 0.35;
W_MAX   = 90;
T_LOAD  = 0.42;
TL_STEP = 0.03;

if t < T_STAND
    w_ref = 0;
elseif t < T_RAMP
    w_ref = W_MAX * (t - T_STAND) / (T_RAMP - T_STAND);
else
    w_ref = W_MAX;
end

if t >= T_LOAD
    TL = TL_STEP;
else
    TL = 0;
end
end
