function [ia, ib, ic, i_alpha, i_beta, theta_true, wm_true] = mf_PMSM_Plant(u_alpha, u_beta, TL, cfg)
%#codegen
% Discrete dq model of a salient PMSM fed by an average value inverter.
% Forward Euler at Ts, currents and angle are states of the chart.
R  = cfg(1);
Ld = cfg(2);
Lq = cfg(3);
psi= cfg(4);
pp = cfg(5);
J  = cfg(6);
Ff = cfg(7);
Ts = cfg(8);

persistent id iq wm thm
if isempty(id)
    id = 0; iq = 0; wm = 0; thm = 0;
end

th_e = pp * thm;
c = cos(th_e);
s = sin(th_e);
ud =  u_alpha * c + u_beta * s;
uq = -u_alpha * s + u_beta * c;

we = pp * wm;
did = (ud - R * id + we * Lq * iq) / Ld;
diq = (uq - R * iq - we * Ld * id - we * psi) / Lq;
Te  = 1.5 * pp * (psi * iq + (Ld - Lq) * id * iq);
dwm = (Te - TL - Ff * wm) / J;

id  = id  + Ts * did;
iq  = iq  + Ts * diq;
wm  = wm  + Ts * dwm;
thm = thm + Ts * wm;

th_e = pp * thm;
c = cos(th_e);
s = sin(th_e);
i_alpha = id * c - iq * s;
i_beta  = id * s + iq * c;
ia = i_alpha;
ib = (sqrt(3) * i_beta - i_alpha) / 2;
ic = -ia - ib;

theta_true = mod(th_e, 2 * pi);
wm_true    = wm;
end
