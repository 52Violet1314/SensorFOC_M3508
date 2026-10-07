function run_smo_hfi_sim()
%RUN_SMO_HFI_SIM  Simulate SMO_HFI_FOC.slx, print metrics, save figure and mat.
here = fileparts(mfilename('fullpath'));
cd(here);
mdl  = 'SMO_HFI_FOC';
p    = pmsm_params();

if ~exist([mdl '.slx'], 'file')
    build_smo_hfi_model();
end
load_system(mdl);
set_param(mdl, 'StopTime', num2str(p.Tstop));
out = sim(mdl);

[ta, A] = tw_norm(out.log_angles);
[ts, S] = tw_norm(out.log_speeds);
[ti, I] = tw_norm(out.log_iabc);
[td, D] = tw_norm(out.log_diag);   % [E_smo, eps_hfi, fusion weight]

wr    = @(x) mod(x + pi, 2*pi) - pi;
e_smo = wr(A(:,1) - A(:,2));
e_hfi = wr(A(:,1) - A(:,3));
e_fus = wr(A(:,1) - A(:,4));

m1 = ta >= 0.02 & ta <= 0.05;
m2 = ta >= 0.30 & ta <= 0.40;
m3 = ta >= 0.44 & ta <= 0.50;
mh = ta >= 0.36 & ta <= 0.50;
d2r = 180/pi;

fprintf('SH: standstill HFI   err max = %8.3f deg  rms = %8.3f deg\n', max(abs(e_hfi(m1)))*d2r, sqrt(mean(e_hfi(m1).^2))*d2r);
fprintf('SH: standstill fused err max = %8.3f deg\n', max(abs(e_fus(m1)))*d2r);
fprintf('SH: highspeed  SMO   err max = %8.3f deg  rms = %8.3f deg\n', max(abs(e_smo(m2)))*d2r, sqrt(mean(e_smo(m2).^2))*d2r);
fprintf('SH: highspeed  HFI   err max = %8.3f deg\n', max(abs(e_hfi(m2)))*d2r);
fprintf('SH: highspeed  fused err max = %8.3f deg  rms = %8.3f deg\n', max(abs(e_fus(m2)))*d2r, sqrt(mean(e_fus(m2).^2))*d2r);
fprintf('SH: loadstep   fused err max = %8.3f deg\n', max(abs(e_fus(m3)))*d2r);
fprintf('SH: speed ref end = %8.2f rad/s, true = %8.2f rad/s, fused = %8.2f rad/s\n', S(end,1), S(end,2), S(end,5));
fprintf('SH: speed err rms (0.36-0.50 s) = %8.3f rad/s, max = %8.3f\n', sqrt(mean((S(mh,1)-S(mh,2)).^2)), max(abs(S(mh,1)-S(mh,2))));
fprintf('SH: ramp tracking err at 0.20 s = %8.3f rad/s\n', interp1(ts, S(:,1)-S(:,2), 0.20));
fprintf('SH: final id = %8.3f A, iq = %8.3f A\n', I(end,4), I(end,5));
fprintf('SH: SMO bias (0.30-0.40 s) = %8.3f deg, ripple rms = %8.3f deg\n', mean(e_smo(m2))*d2r, std(e_smo(m2))*d2r);
fprintf('SH: E_smo at 0.04 s = %7.3f V, at 0.20 s = %7.3f V, at end = %7.3f V\n', ...
    interp1(td, D(:,1), 0.04), interp1(td, D(:,1), 0.20), D(end,1));
fprintf('SH: fusion weight at 0.04 / 0.20 / 0.40 s = %5.2f %5.2f %5.2f\n', ...
    interp1(td, D(:,3), 0.04), interp1(td, D(:,3), 0.20), interp1(td, D(:,3), 0.40));
for tc = [0.05 0.15 0.25 0.35 0.45 0.499]
    ia = interp1(ta, A, tc);
    is = interp1(ts, S, tc);
    fprintf('SH: t=%5.3f  therr smo=%8.3f hfi=%8.3f fus=%8.3f deg   w ref=%7.2f true=%7.2f smo=%7.2f hfi=%7.2f\n', ...
        tc, wr(ia(1)-ia(2))*d2r, wr(ia(1)-ia(3))*d2r, wr(ia(1)-ia(4))*d2r, is(1), is(2), is(3)/p.pp, is(4)/p.pp);
end

set(0, 'DefaultFigureVisible', 'off');
f = figure('Position', [60 60 1250 900], 'Visible', 'off');
subplot(4,2,1);
plot(ta, A(:,1)*d2r, ta, A(:,4)*d2r, ta, A(:,3)*d2r); grid on;
legend('true','fused','HFI','Location','best'); title('electrical angle [deg]'); xlabel('t [s]');
subplot(3,2,2);
plot(ta, e_fus*d2r, ta, e_smo*d2r, ta, e_hfi*d2r); grid on;
legend('fused-true','SMO-true','HFI-true','Location','best'); title('angle error [deg]'); xlabel('t [s]');
subplot(3,2,3);
plot(ts, S(:,1), ts, S(:,2), ts, S(:,5)); grid on;
legend('ref','true','fused','Location','best'); title('mechanical speed [rad/s]'); xlabel('t [s]');
subplot(3,2,4);
plot(ts, S(:,3)/p.pp, ts, S(:,4)/p.pp); grid on;
legend('SMO','HFI','Location','best'); title('observer speed [mech rad/s]'); xlabel('t [s]');
subplot(3,2,5);
plot(ti, I(:,1), ti, I(:,2), ti, I(:,3)); grid on;
legend('ia','ib','ic','Location','best'); title('phase currents [A]'); xlabel('t [s]');
subplot(3,2,6);
plot(ti, I(:,4), ti, I(:,5)); grid on;
legend('id','iq','Location','best'); title('dq currents [A]'); xlabel('t [s]');
subplot(4,2,7);
plot(td, D(:,1)); grid on; title('SMO back EMF magnitude [V]'); xlabel('t [s]');
subplot(4,2,8);
plot(td, D(:,3)); grid on; title('fusion weight (1 = SMO)'); xlabel('t [s]'); ylim([-0.1 1.1]);

print(f, '-dpng', '-r110', fullfile(here, 'smo_hfi_results.png'));
save(fullfile(here, 'smo_hfi_results.mat'), 'ta', 'A', 'ts', 'S', 'ti', 'I', 'td', 'D', 'p');
fprintf('SH: figure and mat saved\n');
end
