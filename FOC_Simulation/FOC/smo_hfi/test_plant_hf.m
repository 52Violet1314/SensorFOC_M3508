function test_plant_hf()
%TEST_PLANT_HF  Plant response to a carrier along d_hat with a fixed angle
%error: compares the measured q_hat carrier current and its demodulation with
%the textbook result.
p  = pmsm_params();
Ts = p.Ts; WH = 2*pi*p.fh; Vh = p.Vh; N = 50;
d2r = 180/pi;
for dth = [10 -10]
    th_true = 0;
    thh     = th_true - dth/d2r;      % estimator angle
    id = 0; iq = 0;
    buf = zeros(N,1); idx = 0; ep = 0;
    n = round(0.02/Ts);
    for k = 1:n
        t = (k-1)*Ts;
        uc = Vh*cos(WH*t);
        ua = uc*cos(thh);
        ub = uc*sin(thh);
        ud =  ua*cos(th_true) + ub*sin(th_true);
        uq = -ua*sin(th_true) + ub*cos(th_true);
        id = id + Ts*(ud - p.R*id)/p.Ld;
        iq = iq + Ts*(uq - p.R*iq)/p.Lq;
        ial = id*cos(th_true) - iq*sin(th_true);
        ibe = id*sin(th_true) + iq*cos(th_true);
        iqh = -ial*sin(thh) + ibe*cos(thh);
        idx = idx + 1; if idx > N, idx = 1; end
        buf(idx) = iqh*sin(WH*t);
        ep = sum(buf)/N;
    end
    Xth = Vh*sin(dth/d2r)*cos(dth/d2r)*(1/p.Ld - 1/p.Lq)/WH;
    fprintf('SH: dtheta = %+5.1f deg: measured eps = %10.6f A, theory eps = %10.6f A, i_d = %8.4f, i_q = %8.4f\n', ...
        dth, ep, 0.5*Xth, id, iq);
end
end
