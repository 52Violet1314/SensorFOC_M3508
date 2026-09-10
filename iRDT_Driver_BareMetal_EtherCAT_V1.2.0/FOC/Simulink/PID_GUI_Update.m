function PID_GUI_Update(gcbh,param)
%set_param('TOP/Motor/Permanent Magnet Synchronous Machine', 'MechanicalLoad', 'Torque TM');

%% gcbh为当前正在配置的bloc
WidthBits = str2num(get_param(gcbh,'paramWidthBits'));
QF_IN     = str2num(get_param(gcbh,'paramFracInputs'));
QF_PRM    = str2num(get_param(gcbh,'paramFracParams'));
QF_OUT    = str2num(get_param(gcbh,'paramFracOut'));

RangeCheck(WidthBits,QF_IN,QF_PRM,QF_OUT);

if strcmp(param,'paramFracInputs')||strcmp(param,'paramFracParams')||strcmp(param,'paramFracOut')
    barrelShift = QF_IN+QF_PRM-QF_OUT;
    if barrelShift>=0
        set_param(gcbh,'paramBarrel',num2str(barrelShift));
    else
        error('Barrel Shift必须大于0');
    end
end
end

function RangeCheck(WidthBits,QF_IN,QF_PRM,QF_OUT)
    if (QF_IN<0)||(QF_IN>WidthBits-1)
        error('Inputs Fraction Part 设置错误');
    end
    if (QF_PRM<0)||(QF_PRM>WidthBits-1)
        error('Parameters Fraction Part 设置错误');
    end
    if (QF_OUT<0)||(QF_OUT>min(WidthBits-1,QF_IN+QF_PRM))
        error('Outputs Fraction Part 设置错误');
    end
end

% function ShowAndOff(gcbh,param,portName,portIndex)

% showPort=get_param(gcbh,param);
% currentBlockPath = getfullname(gcbh);%获取当前模块的路径，根据命名得到需要修改block名
% if strcmp(showPort,'External')
%     replace_block(currentBlockPath,'FollowLinks', 'on','Name',portName,'Inport','noprompt');
%     switch(portName)
%         case 'Reference'
%             set_param(strcat(getfullname(gcbh),'/',portName),'Port','1');
%         case 'Input'
%             set_param(strcat(getfullname(gcbh),'/',portName),'Port','2');
%         case 'Kp'
%             set_param(strcat(getfullname(gcbh),'/',portName),'Port','3');
%         case 'Ki'
%             set_param(strcat(getfullname(gcbh),'/',portName),'Port','4');
%         case 'Kd'
%             set_param(strcat(getfullname(gcbh),'/',portName),'Port','5');
%         case 'SatMax'
%             set_param(strcat(getfullname(gcbh),'/',portName),'Port','6');
%         case 'SatMin'
%             set_param(strcat(getfullname(gcbh),'/',portName),'Port','7');
%         case 'InitInt'
%             set_param(strcat(getfullname(gcbh),'/',portName),'Port','8');
%         case 'InitInt_Enable'
%             set_param(strcat(getfullname(gcbh),'/',portName),'Port','9');
%     end
%     
%     p=Simulink.Mask.get(gcbh);%获取Mask参数
%     p2=p.Parameters(portIndex);
%     p2.set('Enabled','off');%关闭Angle的初始值
% else%Internal
%     replace_block(currentBlockPath,'FollowLinks', 'on','Name',portName,'Constant','noprompt');%,'noprompt'替换端口为subsystem
%     p=Simulink.Mask.get(gcbh);%获取Mask参数
%     p2=p.Parameters(portIndex);
%     p2.set('Enabled','on');%关闭Angle的初始值
%     set_param(strcat(getfullname(gcbh),'/',portName),'Value',p2.Value);%修改为指定值
% end
% end

% function addNewBlock(gcbh,param,portName,portIndex)
% showPort=get_param(gcbh,param);
% currentBlockPath = getfullname(gcbh);%获取当前模块的路径，根据命名得到需要修改block名
% if strcmp(showPort,'External')
%     replace_block(currentBlockPath,'FollowLinks', 'on','Name',portName,'Inport','noprompt');
%     p=Simulink.Mask.get(gcbh);%获取Mask参数
%     p2=p.Parameters(portIndex);
%     p2.set('Enabled','off');%关闭Angle的初始值
% else%Internal
%     delete_line(currentBlockPath,[portName,'/1'],'Conv1/1');%先断开再对Port进行操作
%     replace_block(currentBlockPath,'FollowLinks', 'on','Name',portName,'Subsystem','noprompt');%,'noprompt'替换端口为subsystem
%     add_block('simulink/Sources/Constant',strcat(currentBlockPath,'/',portName,'/Source'));%在subsystem中添加const模块
%     set_param(strcat(currentBlockPath,'/',portName,'/Source'),'Value','0');%设置const数据类型和值
% 
%     p=Simulink.Mask.get(gcbh);%获取Mask参数
%     p2=p.Parameters(portIndex);
%     p2.set('Enabled','on');%关闭Angle的初始值
% 
%     positionSource=get_param(strcat(currentBlockPath,'/',portName,'/Source'),'Position');%获取Source的位置，添加out模块是对齐，返回[left,top,right,bottom]
%     positionOut=positionSource+[100,0,100,0];%添加偏移，得到Out的位置
%     add_block('simulink/Ports & Subsystems/Out1',strcat(currentBlockPath,'/',portName,'/Out'),'Position',positionOut);%在subsystem中添加out模块
%     add_line(strcat(currentBlockPath,'/',portName),'Source/1','Out/1','autorouting','on');%autorouting折线连接
%     add_line(currentBlockPath,[portName,'/1'],'Conv1/1','autorouting','on');%把subsystem连接起来
% end
% end

