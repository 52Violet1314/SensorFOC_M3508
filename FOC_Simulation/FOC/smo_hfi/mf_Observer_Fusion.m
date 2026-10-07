function [theta_hat, w_hat, wgt] = mf_Observer_Fusion(theta_smo, w_smo, E_smo, theta_hfi, w_hfi, cfg)
%#codegen
% Continuous fusion.  The injection observer owns standstill and low speed,
% the back EMF observer takes over as the flux linkage produces a usable EMF.
% The output angle is held by an NCO so that the output never jumps when the
% two inputs disagree, but the reported speed is the blended observer speed
% only: feeding the NCO trim back as speed would inject a phantom speed into
% the speed loop whenever the angle estimate is off.
psi  = cfg(4);
pp   = cfg(5);
Ts   = cfg(8);
% The membership is driven by the EMF magnitude, which grows monotonically
% with speed.  The old speed based test flickered because the SMO speed is
% noisy, and the flicker dragged the fused angle with it.
E1   = cfg(12) * psi;   % fade in  at this EMF [V]
E2   = cfg(13) * psi;   % fade out at this EMF [V]
E_OK = E1;
KNCO = 400;     % angle trimming gain [1/s]
WCMAX= 300;     % trim limit [electrical rad/s]
FC_W = 30;      % speed output filter [Hz]

persistent th wf
if isempty(th)
    th = theta_hfi;
    wf = 0;
end

if E_smo <= E1
    wgt = 0;
elseif E_smo >= E2
    wgt = 1;
else
    u   = (E_smo - E1) / (E2 - E1);
    wgt = u * u * (3 - 2 * u);
end

d_hfi = mod(theta_hfi - th + pi, 2 * pi) - pi;
d_smo = mod(theta_smo - th + pi, 2 * pi) - pi;

% The injection observer only resolves the rotor angle modulo pi.  As soon
% as the back EMF observer is trusted, the carrier estimate is aligned to the
% same pole: this is the standard way to remove the 180 deg ambiguity.
if (E_smo > E1) && (abs(d_hfi) > pi/2)
    d_hfi = mod(d_hfi + 2 * pi, 2 * pi) - pi;
end

w_obs  = (1 - wgt) * w_hfi + wgt * w_smo;
w_corr = KNCO * ((1 - wgt) * d_hfi + wgt * d_smo);
if w_corr >  WCMAX
    w_corr =  WCMAX;
end
if w_corr < -WCMAX
    w_corr = -WCMAX;
end

th = mod(th + (w_obs + w_corr) * Ts, 2 * pi);

% The reported speed is filtered: the injection loop has a fast, lightly
% damped speed channel and the speed regulator must not chase it.
al = 2 * pi * FC_W * Ts;
wf = wf + al * (w_obs - wf);

theta_hat = th;
w_hat     = wf / pp;
end
