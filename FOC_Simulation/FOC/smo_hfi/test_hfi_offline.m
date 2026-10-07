function test_hfi_offline()
%TEST_HFI_OFFLINE  Step the controller, plant and HFI observer outside
%Simulink to check the demodulator sign, gain and bias in seconds.
p   = pmsm_params();
cfg = [p.R p.Ld p.Lq p.psi p.pp p.J p.F p.Ts p.Udc p.fh p.Vh p.wlo p.whi];
Ts  = p.Ts;
T   = 0.12;
n   = round(T / Ts);

clear mf_PMSM_Plant mf_FOC_Controller mf_HFI_Observer mf_Observer_Fusion mf_SMO_PLL

ua = 0; ub = 0; TL = 0;
th_hat = 0; w_hat = 0;
rec = zeros(n, 12);
for k = 1:n
    t = (k-1)*Ts;
    [ia, ib, ic, ial, ibe, tht, wm] = mf_PMSM_Plant(ua, ub, TL, cfg);
    [uap, ubp, idm, iqm, iqr]    = mf_FOC_Controller(0, w_hat, ia, ib, ic, th_hat, cfg);
    [ua, ub, thh, whh, epsi]     = mf_HFI_Observer(uap, ubp, ial, ibe, t, cfg);
    [th_hat, w_hat, g]           = mf_Observer_Fusion(tht, 0, 0, thh, whh, cfg);
    rec(k,:) = [t tht thh th_hat whh epsi idm iqm iqr wm uap ubp];
end

wr = @(x) mod(x + pi, 2*pi) - pi;
d2r = 180/pi;
m = rec(:,1) >= T-0.03;
dh = wr(rec(m,2) - rec(m,3));
ep = rec(m,6);
Kth = p.Vh*(1/p.Ld - 1/p.Lq)/(4*2*pi*p.fh);
fprintf('SH: --- last 30 ms ---\n');
fprintf('SH: mean dtheta(hfi) = %8.3f deg, rms = %8.3f deg\n', mean(dh)*d2r, sqrt(mean(dh.^2))*d2r);
fprintf('SH: mean eps = %10.6f A,  rms eps = %10.6f A\n', mean(ep), sqrt(mean(ep.^2)));
fprintf('SH: theory Kth*sin(2*dtheta) mean = %10.6f A (Kth = %10.6f)\n', mean(Kth*sin(2*dh)), Kth);
cc = mean(sin(2*dh).*ep) / (std(sin(2*dh))*std(ep) + eps);
fprintf('SH: corr(sin(2*dtheta), eps) = %6.3f\n', cc);
fprintf('SH: mean id_m = %7.3f A, iq_m = %7.3f A\n', mean(rec(m,7)), mean(rec(m,8)));
fprintf('SH: mean w_hfi = %8.3f elec rad/s, w_true = %8.3f mech rad/s\n', mean(rec(m,5)), mean(rec(m,10)));
fprintf('SH: eps min/max = %10.6f / %10.6f\n', min(rec(:,6)), max(rec(:,6)));
fprintf('SH: ia range over run: %10.5f .. %10.5f\n', min(rec(:,7)), max(rec(:,7)));
fprintf('SH: uap range over run: %10.5f .. %10.5f\n', min(rec(:,11)), max(rec(:,11)));
fprintf('SH: first rows (t, tht, thh, eps, iqm):\n');
for k = [1 2 3 10 100 1000]
    fprintf('SH:  %9.6f %9.5f %9.5f %10.6f %9.5f\n', rec(k,1), rec(k,2), rec(k,3), rec(k,6), rec(k,8));
end
fprintf('SH: final th_true = %8.3f deg, th_hfi = %8.3f deg, th_hat = %8.3f deg\n', rec(end,2)*d2r, rec(end,3)*d2r, rec(end,4)*d2r);
end
