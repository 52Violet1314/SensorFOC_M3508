function [u_alpha, u_beta, theta_hfi, w_hfi, eps_hfi] = mf_HFI_Observer(u_alpha_pi, u_beta_pi, i_alpha, i_beta, t, cfg)
%#codegen
% Pulsating carrier injection along the estimated d axis, band pass around
% the carrier and synchronous demodulation of the q axis carrier current.
%   i_qh = Vh*(1/Ld-1/Lq)/(2*wh) * sin(2*dtheta) * sin(wh*t)
% The average of i_qh*sin(wh*t) over one carrier period gives
%   eps = Kth * sin(2*dtheta),  Kth = Vh*(1/Ld-1/Lq)/(4*wh)
% Two cascaded band pass sections centred exactly on the carrier remove the
% fundamental current, which is what otherwise swamps the demodulation as
% soon as the rotor turns.  A band pass has zero phase at its centre, so the
% demodulation stays in quadrature and the loop gain keeps its sign.
% N must equal 1/(fh*Ts).
Ld = cfg(2);
Lq = cfg(3);
Ts = cfg(8);
fh = cfg(10);
Vh = cfg(11);

WH     = 2 * pi * fh;
N      = 50;      % = 1/(fh*Ts)
QBP    = 3;       % band pass quality factor
W_GATE = 500;     % carrier is switched off above this electrical speed
KP_H   = 250;
KI_H   = 25000;
XI_MAX = 200;     % integrator limit: |w| stays below W_GATE so that the
                  % loop always keeps updating and can recover from a slip

persistent th w xi buf idx b0 b1 b2 a1 a2 x1 x2 y1 y2 x1b x2b y1b y2b
if isempty(th)
    th  = 0;
    w   = 0;
    xi  = 0;
    buf = zeros(N, 1);
    idx = 0;
    w0  = 2 * pi * fh * Ts;
    al  = sin(w0) / (2 * QBP);
    a0  = 1 + al;
    b0  = al / a0;
    b1  = 0;
    b2  = -al / a0;
    a1  = -2 * cos(w0) / a0;
    a2  = (1 - al) / a0;
    x1 = 0; x2 = 0; y1 = 0; y2 = 0;
    x1b = 0; x2b = 0; y1b = 0; y2b = 0;
end

if abs(w) > W_GATE
    Vh_e = 0;
else
    Vh_e = Vh;
end

c = cos(th);
s = sin(th);

% ---- carrier voltage along the estimated d axis ----
uc = Vh_e * cos(WH * t);
u_alpha = u_alpha_pi + uc * c;
u_beta  = u_beta_pi  + uc * s;

% ---- measured current in the estimated frame ----
id_h =  i_alpha * c + i_beta * s;
iq_h = -i_alpha * s + i_beta * c;

% ---- two cascaded band pass sections at the carrier ----
ys = b0 * iq_h + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
x2 = x1; x1 = iq_h; y2 = y1; y1 = ys;
yf = b0 * ys + b1 * x1b + b2 * x2b - a1 * y1b - a2 * y2b;
x2b = x1b; x1b = ys; y2b = y1b; y1b = yf;

% ---- synchronous demodulation, average over one carrier period ----
idx = idx + 1;
if idx > N
    idx = 1;
end
buf(idx) = yf * sin(WH * t);

acc = 0;
for k = 1:N
    acc = acc + buf(k);
end
eps_hfi = acc / N;

if abs(w) <= W_GATE
    Kth = Vh * (1 / Ld - 1 / Lq) / (4 * WH);
    if Kth == 0
        eps_n = 0;
    else
        eps_n = eps_hfi / Kth;
    end
    if eps_n >  1
        eps_n =  1;
    end
    if eps_n < -1
        eps_n = -1;
    end
    xi = xi + KI_H * eps_n * Ts;
    if xi >  XI_MAX
        xi =  XI_MAX;
    end
    if xi < -XI_MAX
        xi = -XI_MAX;
    end
    w  = KP_H * eps_n + xi;
    th = mod(th + w * Ts, 2 * pi);
end

theta_hfi = th;
w_hfi = w;
end
