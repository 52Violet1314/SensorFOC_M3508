function [u_alpha_pi, u_beta_pi, id_m, iq_m, iq_ref] = mf_FOC_Controller(w_ref, w_hat, ia, ib, ic, theta_hat, cfg)
%#codegen
% Speed loop (mechanical rad/s) plus the two current loops, all in the
% estimated dq frame.  cfg = [R Ld Lq psi pp J F Ts Udc fh Vh wlo whi].
R   = cfg(1);
Ld  = cfg(2);
Lq  = cfg(3);
psi = cfg(4);
pp  = cfg(5);
Ts  = cfg(8);
Udc = cfg(9);

% The speed loop must stay well below the observer bandwidth, otherwise it
% chases the injection PLL transients.
Kp_w  = 0.5;
Ki_w  = 5;
Kp_i  = 0.24;
Ki_i  = 490;
I_MAX = 4.0;

persistent xi_w int_d int_q
if isempty(xi_w)
    xi_w  = 0;
    int_d = 0;
    int_q = 0;
end

% ---- speed loop ----
e_w  = w_ref - w_hat;
xi_w = xi_w + Ki_w * e_w * Ts;
iq_c = Kp_w * e_w + xi_w;
if iq_c > I_MAX
    iq_c = I_MAX;
    xi_w = xi_w - Ki_w * e_w * Ts;
elseif iq_c < -I_MAX
    iq_c = -I_MAX;
    xi_w = xi_w - Ki_w * e_w * Ts;
end
iq_ref = iq_c;

% ---- Clarke, same convention as the plant ----
i_al = ia;
i_be = (ia + 2 * ib) / sqrt(3);

% ---- Park with the estimated angle ----
c = cos(theta_hat);
s = sin(theta_hat);
id_m =  i_al * c + i_be * s;
iq_m = -i_al * s + i_be * c;

% ---- current loops ----
e_d = 0 - id_m;
e_q = iq_ref - iq_m;
int_d = int_d + Ki_i * e_d * Ts;
int_q = int_q + Ki_i * e_q * Ts;
ud = Kp_i * e_d + int_d;
uq = Kp_i * e_q + int_q;

% ---- decoupling feed forward ----
we = pp * w_hat;
ud = ud - we * Lq * iq_m;
uq = uq + we * (Ld * id_m + psi);

% ---- linear modulation limit with back calculation ----
u_max = Udc / sqrt(3);
u_m   = sqrt(ud * ud + uq * uq);
if u_m > u_max
    k  = u_max / u_m;
    ud = ud * k;
    uq = uq * k;
    int_d = (ud + we * Lq * iq_m) - Kp_i * e_d;
    int_q = (uq - we * (Ld * id_m + psi)) - Kp_i * e_q;
end

% ---- inverse Park ----
u_alpha_pi = ud * c - uq * s;
u_beta_pi  = ud * s + uq * c;
end
