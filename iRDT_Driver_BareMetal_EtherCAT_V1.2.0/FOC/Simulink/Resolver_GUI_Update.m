function Resolver_GUI_Update(gcbh,omega1,zeta1,paramFreq1,T_FLU1,Resolver_PolePairs1)
%set_param('TOP/Motor/Permanent Magnet Synchronous Machine', 'MechanicalLoad', 'Torque TM');

%% gcbh为当前正在配置的block
omega=str2num(get_param(gcbh,omega1));%获取mask 中"paramShow"的值(on/off)
zeta =str2num(get_param(gcbh,zeta1));%获取mask 中"paramShow"的值(on/off)
Tresolver=1/str2num(get_param(gcbh,paramFreq1));%获取mask 中"paramShow"的值(on/off)
T_FLU=str2num(get_param(gcbh,T_FLU1));%获取mask 中"paramShow"的值(on/off)
Resolver_PolePairs=str2num(get_param(gcbh,Resolver_PolePairs1));%获取mask 中"paramShow"的值(on/off)



K3s = (Tresolver/T_FLU)*(2*pi)/(Tresolver*2^16);
K2s = omega^2/K3s; 
K1s = zeta*2*sqrt(K2s/K3s);
K1 = K1s;                                         % Proportional gain of PI controller for speed calculation 
K2 = K2s*Tresolver;                               % Integral gain of PI controller for speed calculation      
K3 = (T_FLU/Tresolver)*K3s*Tresolver*2^16/(2*pi); % Integral gain for position calculation
speed_gain = K3s;                                 % Sensor speed = Sensor speed x speed_gain

set_param(gcbh,'K1s',num2str(K1s));
set_param(gcbh,'K2s',num2str(K2s));
set_param(gcbh,'K3s',num2str(K3s));
set_param(gcbh,'K1',num2str(K1));
set_param(gcbh,'K2',num2str(K2));
set_param(gcbh,'K3',num2str(K3));
set_param(gcbh,'speed_gain',num2str(speed_gain));
end
