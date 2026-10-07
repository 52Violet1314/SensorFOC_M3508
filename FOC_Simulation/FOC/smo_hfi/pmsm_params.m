function p = pmsm_params()
%PMSM_PARAMS  Motor and drive data for the SMO + PLL + HFI demo model.
% R, Ld, psi, pp, J, F are taken from the Permanent Magnet Synchronous
% Machine block of ../SVPWM.slx.  Lq is deliberately larger than Ld: the
% rotor saliency is what the high frequency injection observes.
p.R    = 0.194;      % phase resistance [Ohm]
p.Ld   = 0.097e-3;   % d axis inductance [H]
p.Lq   = 0.145e-3;   % q axis inductance [H]  (saliency ratio 1.49)
p.psi  = 0.015152;   % PM flux linkage [Wb]
p.pp   = 4;          % pole pairs
p.J    = 6.214e-4;   % inertia [kg m^2]
p.F    = 3.034e-4;   % viscous friction [N m s/rad]
p.Udc  = 24;         % dc bus voltage [V]
p.Ts   = 2e-5;       % sample time [s]  (50 kHz)
p.fh   = 1000;       % HF injection frequency [Hz]
p.Vh   = 0.5;        % HF injection amplitude [V]
p.wlo  = 150;        % fusion lower fade, electrical rad/s
p.whi  = 280;        % fusion upper fade, electrical rad/s
p.Tstop= 0.5;        % simulation stop time [s]
end
