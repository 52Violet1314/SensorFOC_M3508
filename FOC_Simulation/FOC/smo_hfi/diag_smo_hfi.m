function diag_smo_hfi()
%DIAG_SMO_HFI  Print a short time table from the last run for debugging.
here = fileparts(mfilename('fullpath'));
S = load(fullfile(here, 'smo_hfi_results.mat'));
ta = S.ta; A = S.A; ts = S.ts; Sv = S.S; ti = S.ti; I = S.I; td = S.td; D = S.D;
d2r = 180/pi;
wr = @(x) mod(x + pi, 2*pi) - pi;
fprintf('SH:      t    th_true   th_hfi   th_smo   th_fus |  w_true  w_hfi  w_smo  w_fus |   eps     wgt\n');
for tc = [0.05 0.06 0.07 0.08 0.10 0.12 0.14 0.18 0.22 0.30 0.40 0.49]
    a = interp1(ta, A, tc);
    s = interp1(ts, Sv, tc);
    d = interp1(td, D, tc);
    fprintf('SH: %7.3f  %8.2f %8.2f %8.2f %8.2f | %7.2f %7.2f %7.2f %7.2f | %8.4f %5.2f\n', ...
        tc, a(1)*d2r, a(3)*d2r, a(2)*d2r, a(4)*d2r, s(2), s(4)/4, s(3)/4, s(5), d(2), d(3));
end
m = ta >= 0.06 & ta <= 0.14;
dh = wr(A(m,1) - A(m,3));
ep = interp1(td, D(:,2), ta(m));
fprintf('SH: window 0.06-0.14 s: std(th_hfi-th_true) = %7.3f deg, eps rms = %8.5f A\n', std(dh)*d2r, sqrt(mean(ep.^2)));
c = corrcoef(sin(2*dh), ep);
fprintf('SH: corr( sin(2*dtheta_hfi), eps ) = %6.3f\n', c(1,2));
fprintf('SH: eps amplitude max = %8.5f A, Kth expected = %8.5f A\n', max(abs(ep)), S.p.Vh*(1/S.p.Ld-1/S.p.Lq)/(4*2*pi*S.p.fh));
fprintf('SH: theta_true span 0-0.05 s = %8.4f deg, at 0.5 s = %8.2f deg\n', (max(A(ta<=0.05,1))-min(A(ta<=0.05,1)))*d2r, A(end,1)*d2r);
fprintf('SH: max |w_true| = %7.2f rad/s\n', max(abs(Sv(:,2))));
end
