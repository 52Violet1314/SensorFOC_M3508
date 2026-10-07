function test_hfi_locked()
%TEST_HFI_LOCKED  Locked rotor test: does the injection observer converge to
%the true rotor angle, with and without a fundamental current?
p   = pmsm_params();
cfg = [p.R p.Ld p.Lq p.psi p.pp p.J p.F p.Ts p.Udc p.fh p.Vh p.wlo p.whi];
Ts  = p.Ts;
T   = 0.06;
n   = round(T / Ts);
d2r = 180/pi;
wr  = @(x) mod(x + pi, 2*pi) - pi;

for wref = [0 100]
    clear mf_FOC_Controller mf_HFI_Observer mf_SMO_PLL mf_Observer_Fusion
    th_lock = deg2rad(10);
    id = 0; iq = 0;
    ua = 0; ub = 0; th_hat = 0; w_hat = 0;
    tr = zeros(n, 9);
    for k = 1:n
        t = (k-1)*Ts;
        c = cos(th_lock); s = sin(th_lock);
        ia = id*c - iq*s;
        ib = (sqrt(3)*(id*s + iq*c) - ia)/2;
        ic = -ia - ib;
        ial = ia; ibe = (ia + 2*ib)/sqrt(3);
        [uap, ubp, idm, iqm, iqr] = mf_FOC_Controller(wref, w_hat, ia, ib, ic, th_hat, cfg);
        [ua, ub, thh, whh, epsi] = mf_HFI_Observer(uap, ubp, ial, ibe, t, cfg);
        [th_hat, w_hat, g] = mf_Observer_Fusion(th_lock, 0, 0, thh, whh, cfg);
        ud =  ua*c + ub*s;
        uq = -ua*s + ub*c;
        id = id + Ts*(ud - p.R*id)/p.Ld;
        iq = iq + Ts*(uq - p.R*iq)/p.Lq;
        tr(k,:) = [t wr(th_lock-thh)*d2r wr(th_lock-th_hat)*d2r epsi iqm idm iqr whh th_hat];
    end
    m = tr(:,1) >= T-0.01;
    fprintf('SH: === wref = %g ===\n', wref);
    fprintf('SH:  final dtheta(hfi) = %8.3f deg,  dtheta(hat) = %8.3f deg\n', tr(end,2), tr(end,3));
    fprintf('SH:  last 10 ms: mean eps = %10.6f, mean iq_m = %8.4f, dtheta rms = %8.3f deg\n', mean(tr(m,4)), mean(tr(m,5)), sqrt(mean(tr(m,2).^2)));
    Kth = p.Vh*(1/p.Ld - 1/p.Lq)/(4*2*pi*p.fh);
    fprintf('SH:  theory Kth*sin(2*dtheta) mean = %10.6f\n', mean(Kth*sin(2*deg2rad(tr(m,2)))));
    fprintf('SH:  first 0.6 ms: k, eps, dtheta_hfi(deg), id_m, iq_m, iq_ref, w_hfi\n');
    for k = [50 60 70 80 90 100 110 120 150 200 300 500 1000 2000]
        fprintf('SH:   %5d %10.6f %8.3f %8.4f %8.4f %8.4f %8.2f\n', k, tr(k,4), tr(k,2), tr(k,6), tr(k,5), tr(k,7), tr(k,8));
    end
end
end
function y = thu_dummy(x)
y = x;
end
