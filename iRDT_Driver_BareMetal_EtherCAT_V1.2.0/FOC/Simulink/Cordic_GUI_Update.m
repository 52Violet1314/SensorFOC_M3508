function Cordic_GUI_Update(gcbh,param)
nowIsRotate= find_system(gcb,'LookUnderMasks','on','FollowLinks','on','BlockType','Inport','Name','Angle');%判断当前所处模式
currentBlockPath = getfullname(gcbh);%获取当前模块的路径，根据命名得到需要修改block名
switch(param)
    case 'paramCordicFunction'
        if strcmp(get_param(gcbh,'paramCordicFunction'),'Circular')
            set_param(gcbh,'paramFunctionBool','1');
        else%Hyperbolic
            set_param(gcbh,'paramFunctionBool','0');
        end
    case 'paramCordicMode'
        if strcmp(get_param(gcbh,'paramCordicMode'),'Vector')
            SetPortAction_Vector(gcbh,currentBlockPath,nowIsRotate);
        else%Rotate
            SetPortAction_Rotate(gcbh,currentBlockPath,nowIsRotate);
        end
    case 'paramInitValueAngle'
        if strcmp(get_param(gcbh,'paramCordicMode'),'Vector')
            InitValueAngle=get_param(gcbh,param);
            set_param(strcat(currentBlockPath,'/Angle'),'Value',InitValueAngle,'OutDataTypeStr','fixdt(0,16,0)');%设置const数据类型和值
        end
end
end
% end

function SetPortAction_Vector(gcbh,currentBlockPath,nowIsRotate)
    set_param(gcbh,'paramModeBool','1');
    if ~isempty(nowIsRotate)
        replace_block(currentBlockPath,'FollowLinks', 'on','Name','Angle','Constant','noprompt');%,'noprompt'替换端口为Constant
        replace_block(currentBlockPath,'FollowLinks', 'on','Name','Yf','Terminator','noprompt');
        replace_block(currentBlockPath,'FollowLinks', 'on','Name','Atan','Outport','noprompt');
    end
    %*******input setting*********%
    InitValueAngle=get_param(gcbh,'paramInitValueAngle');
    set_param(strcat(currentBlockPath,'/Angle'),'Value',InitValueAngle,'OutDataTypeStr','fixdt(0,16,0)');
    p=Simulink.Mask.get(gcbh);%获取Mask参数
    p2=p.Parameters(5);
    p2.set('Enabled','on');%关闭Angle的初始值
    %*******output setting*********%
    set_param(strcat(currentBlockPath,'/Atan'),'OutDataTypeStr','fixdt(0,16,0)');
    set_param(strcat(currentBlockPath,'/Atan'),'AttributesFormatString','%<OutDataTypeStr>');%显示数据类型
end
function SetPortAction_Rotate(gcbh,currentBlockPath,nowIsRotate)
    set_param(gcbh,'paramModeBool','0');
    if isempty(nowIsRotate)
        replace_block(currentBlockPath,'FollowLinks', 'on','Name','Angle','Inport','noprompt');
        replace_block(currentBlockPath,'FollowLinks', 'on','Name','Xf','Outport','noprompt');
        replace_block(currentBlockPath,'FollowLinks', 'on','Name','Yf','Outport','noprompt');
        replace_block(currentBlockPath,'FollowLinks', 'on','Name','Atan','Terminator','noprompt');
    end
    %*******input setting*********%
    set_param(strcat(currentBlockPath,'/Angle'),'OutDataTypeStr','fixdt(0,16,0)');
    set_param(strcat(currentBlockPath,'/Angle'),'AttributesFormatString','%<OutDataTypeStr>');
    p=Simulink.Mask.get(gcbh);%获取Mask参数
    p2=p.Parameters(5);
    p2.set('Enabled','off');%关闭Angle的初始值
    %*******output setting*********%
    set_param(strcat(currentBlockPath,'/Xf'),'OutDataTypeStr','fixdt(1,32,paramCQF)');
    set_param(strcat(currentBlockPath,'/Xf'),'AttributesFormatString','%<OutDataTypeStr>');
    set_param(strcat(currentBlockPath,'/Yf'),'OutDataTypeStr','fixdt(1,32,paramCQF)');
    set_param(strcat(currentBlockPath,'/Yf'),'AttributesFormatString','%<OutDataTypeStr>');
    
end
%% 替换端口
% position1=get_param(Input_1,'Position');%返回[left,top,right,bottom]
% handle = add_block('simulink/Sources/Ground', 'FOCfilter_ui/FOC_Filter/Ground','Position',position1);%下次在添加前需删除
% handle = add_block('simulink/Sources/Ground', 'FOCfilter_ui/FOC_Filter/Ground','MakeNameUnique', 'on');%下次添加会自动改名字
% set_param(handle,'Position',position1);
% get_param('FOCfilter_ui/FOC_Filter/Ground1','Position')
% replace_block('sys', 'old_blk', 'new_blk')

% systemName = gcs;
% index=strfind(systemName,'/');
% if ~isempty(index)
%     systemName=a(1:index(1)-1);
% end
% isLibrary = bdIsLibrary(systemName);
% %%********防止仿真时端口断开**********%%
% if isLibrary==true
%     modelState='stopped';%library 下能修改
% else%not library
%     modelState=get_param(systemName,'SimulationStatus');%model下，仅在stop状态下才能修改
% end
% modelState=get_param(systemName,'SimulationStatus');%model下，仅在stop状态下才能修改
