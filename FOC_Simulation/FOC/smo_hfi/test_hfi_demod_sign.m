function test_hfi_demod_sign()
%TEST_HFI_DEMOD_SIGN  Feed a synthetic carrier current into the HFI chart and
%check the sign and gain of the demodulated output.
p   = pmsm_params();
cfg = [p.R p.Ld p.Lq p.psi p.pp p.J p.F p.Ts p.Udc p.fh p.Vh p.wlo p.whi];
Ts  = p.Ts;
WH  = 2*pi*p.fh;
A   = 0.05;
clear mf_HFI_Observer
n = 200;
ep = zeros(n,1); th = zeros(n,1);
for k = 1:n
    t = (k-1)*Ts;
    % with theta_hat close to zero this makes i_q_hat = A*sin(WH*t)
    ial = 0;
    ibe = A*sin(WH*t);
    [~, ~, thh, whh, epsi] = mf_HFI_Observer(0, 0, ial, ibe, t, cfg);
    ep(k) = epsi; th(k) = thh;
end
fprintf('SH: synthetic i_q = %g*sin(wh*t), expected eps = %g (positive)\n', A, A/2);
fprintf('SH: eps at k = 60, 80, 100, 150, 200: %10.6f %10.6f %10.6f %10.6f %10.6f\n', ep(60), ep(80), ep(100), ep(150), ep(200));
fprintf('SH: theta_hat at k = 100, 200 = %8.4f %8.4f rad\n', th(100), th(200));
end
