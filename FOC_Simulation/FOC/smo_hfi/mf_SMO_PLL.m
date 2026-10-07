function [theta_smo, w_smo, E_smo] = mf_SMO_PLL(u_alpha, u_beta, i_alpha, i_beta, cfg)
%#codegen
% Sliding mode current observer in alpha beta plus a PLL on the estimated
% back EMF.  The PLL output is advanced by the phase lag of the back EMF
% low pass filter, otherwise the estimate would trail by atan(we/wc).
R  = cfg(1);
Ld = cfg(2);
Lq = cfg(3);
Ts = cfg(8);

L       = 0.5 * (Ld + Lq);
K_SLIDE = 40;
FC_EMF  = 200;
E_MIN   = 1.5;   % do not trust the EMF estimate while the carrier dominates
KP_PLL  = 300;
KI_PLL  = 20000;
XI_MAX  = 4000;

FC_W = 100;      % output speed filter [Hz]

persistent ia_h ib_h ea_f eb_f xi th w wf
if isempty(ia_h)
    ia_h = 0; ib_h = 0; ea_f = 0; eb_f = 0; xi = 0; th = 0; w = 0; wf = 0;
end

ea_r = ia_h - i_alpha;
eb_r = ib_h - i_beta;
za = K_SLIDE * sign(ea_r);
zb = K_SLIDE * sign(eb_r);

ia_h = ia_h + Ts * (u_alpha - R * ia_h - za) / L;
ib_h = ib_h + Ts * (u_beta  - R * ib_h - zb) / L;

al   = 1 - exp(-2 * pi * FC_EMF * Ts);
ea_f = (1 - al) * ea_f + al * za;
eb_f = (1 - al) * eb_f + al * zb;

E = sqrt(ea_f * ea_f + eb_f * eb_f);

if E > E_MIN
    eps_s = (-ea_f * cos(th) - eb_f * sin(th)) / E;
    xi = xi + KI_PLL * eps_s * Ts;
    if xi >  XI_MAX
        xi =  XI_MAX;
    end
    if xi < -XI_MAX
        xi = -XI_MAX;
    end
    w  = KP_PLL * eps_s + xi;
    th = th + w * Ts;
    th = mod(th, 2 * pi);
end

% The sliding mode term chatters, so the raw PLL speed is filtered before it
% leaves the observer.  The angle itself is an integral and stays smooth.
al_w = 2 * pi * FC_W * Ts;
wf   = wf + al_w * (w - wf);

theta_smo = mod(th + atan2(wf, 2 * pi * FC_EMF), 2 * pi);
w_smo = wf;
E_smo = E;
end
