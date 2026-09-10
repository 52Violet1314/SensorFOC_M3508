function Matrix_GUI_Update(gcbh,param)



currentBlockPath = getfullname(gcbh);%获取当前模块的路径，根据命名得到需要修改block名
B1Exist= find_system(gcb,'LookUnderMasks','on','FollowLinks','on','BlockType','Inport','Name','B1');%判断当前所处模式
B2Exist= find_system(gcb,'LookUnderMasks','on','FollowLinks','on','BlockType','Inport','Name','B2');%判断当前所处模式
B3Exist= find_system(gcb,'LookUnderMasks','on','FollowLinks','on','BlockType','Inport','Name','B3');%判断当前所处模式
B4Exist= find_system(gcb,'LookUnderMasks','on','FollowLinks','on','BlockType','Inport','Name','B4');%判断当前所处模式
B5Exist= find_system(gcb,'LookUnderMasks','on','FollowLinks','on','BlockType','Inport','Name','B5');%判断当前所处模式
if strcmp(param,'paramMIterations')
    switch(get_param(gcbh,param))
        case '1'
            if ~isempty(B1Exist)
                replace_block(currentBlockPath,'FollowLinks', 'on','Name','B0','Inport','noprompt');
                replace_block(currentBlockPath,'FollowLinks', 'on','Name','B1','Constant','noprompt');
                replace_block(currentBlockPath,'FollowLinks', 'on','Name','B2','Constant','noprompt');
                replace_block(currentBlockPath,'FollowLinks', 'on','Name','B3','Constant','noprompt');
                replace_block(currentBlockPath,'FollowLinks', 'on','Name','B4','Constant','noprompt');
                replace_block(currentBlockPath,'FollowLinks', 'on','Name','B5','Constant','noprompt');
            end
        case '2'    
            if ~isempty(B2Exist) || isempty(B1Exist)
                replace_block(currentBlockPath,'FollowLinks', 'on','Name','B0','Inport','noprompt');
                replace_block(currentBlockPath,'FollowLinks', 'on','Name','B1','Inport','noprompt');
                replace_block(currentBlockPath,'FollowLinks', 'on','Name','B2','Constant','noprompt');
                replace_block(currentBlockPath,'FollowLinks', 'on','Name','B3','Constant','noprompt');
                replace_block(currentBlockPath,'FollowLinks', 'on','Name','B4','Constant','noprompt');
                replace_block(currentBlockPath,'FollowLinks', 'on','Name','B5','Constant','noprompt');
            end
        case '3'
            if ~isempty(B3Exist) || isempty(B2Exist)
                replace_block(currentBlockPath,'FollowLinks', 'on','Name','B0','Inport','noprompt');
                replace_block(currentBlockPath,'FollowLinks', 'on','Name','B1','Inport','noprompt');
                replace_block(currentBlockPath,'FollowLinks', 'on','Name','B2','Inport','noprompt');
                replace_block(currentBlockPath,'FollowLinks', 'on','Name','B3','Constant','noprompt');
                replace_block(currentBlockPath,'FollowLinks', 'on','Name','B4','Constant','noprompt');
                replace_block(currentBlockPath,'FollowLinks', 'on','Name','B5','Constant','noprompt');
            end
        case '4'
            if ~isempty(B4Exist) || isempty(B3Exist)
                replace_block(currentBlockPath,'FollowLinks', 'on','Name','B0','Inport','noprompt');
                replace_block(currentBlockPath,'FollowLinks', 'on','Name','B1','Inport','noprompt');
                replace_block(currentBlockPath,'FollowLinks', 'on','Name','B2','Inport','noprompt');
                replace_block(currentBlockPath,'FollowLinks', 'on','Name','B3','Inport','noprompt');
                replace_block(currentBlockPath,'FollowLinks', 'on','Name','B4','Constant','noprompt');
                replace_block(currentBlockPath,'FollowLinks', 'on','Name','B5','Constant','noprompt');
            end
        case '5'
            if ~isempty(B5Exist) || isempty(B4Exist)
                replace_block(currentBlockPath,'FollowLinks', 'on','Name','B0','Inport','noprompt');
                replace_block(currentBlockPath,'FollowLinks', 'on','Name','B1','Inport','noprompt');
                replace_block(currentBlockPath,'FollowLinks', 'on','Name','B2','Inport','noprompt');
                replace_block(currentBlockPath,'FollowLinks', 'on','Name','B3','Inport','noprompt');
                replace_block(currentBlockPath,'FollowLinks', 'on','Name','B4','Inport','noprompt');
                replace_block(currentBlockPath,'FollowLinks', 'on','Name','B5','Constant','noprompt');
            end
        case '6'
            if isempty(B5Exist)
                replace_block(currentBlockPath,'FollowLinks', 'on','Name','B0','Inport','noprompt');
                replace_block(currentBlockPath,'FollowLinks', 'on','Name','B1','Inport','noprompt');
                replace_block(currentBlockPath,'FollowLinks', 'on','Name','B2','Inport','noprompt');
                replace_block(currentBlockPath,'FollowLinks', 'on','Name','B3','Inport','noprompt');
                replace_block(currentBlockPath,'FollowLinks', 'on','Name','B4','Inport','noprompt');
                replace_block(currentBlockPath,'FollowLinks', 'on','Name','B5','Inport','noprompt');
            end
    end
    DataType = strcat('fixdt(1,',get_param(gcbh,'paramBWidthBits'),',',get_param(gcbh,'paramFracB'),')');
    set_param(strcat(currentBlockPath,'/B0'),'OutDataTypeStr',DataType);%set Input B dataTpye to fixdt(1,paramBWidthBits,paramFracB)
    set_param(strcat(currentBlockPath,'/B1'),'OutDataTypeStr',DataType);
    set_param(strcat(currentBlockPath,'/B2'),'OutDataTypeStr',DataType);
    set_param(strcat(currentBlockPath,'/B3'),'OutDataTypeStr',DataType);
    set_param(strcat(currentBlockPath,'/B4'),'OutDataTypeStr',DataType);
    set_param(strcat(currentBlockPath,'/B5'),'OutDataTypeStr',DataType);
end

if strcmp(param,'paramValueA0')||strcmp(param,'paramValueA1')
    a=str2num(get_param(gcbh,'paramValueA0'));%获取Mask参数
    b=str2num(get_param(gcbh,'paramValueA1'));
    if (length(a)~=6)||(length(b)~=6)
        error('参数A矩阵维数错误');
    end
    set_param(strcat(currentBlockPath,'/A0'),'Value',num2str(a(1)));
    set_param(strcat(currentBlockPath,'/A1'),'Value',num2str(a(2)));%修改为指定值
    set_param(strcat(currentBlockPath,'/A2'),'Value',num2str(a(3)));%修改为指定值
    set_param(strcat(currentBlockPath,'/A3'),'Value',num2str(a(4)));%修改为指定值
    set_param(strcat(currentBlockPath,'/A4'),'Value',num2str(a(5)));%修改为指定值
    set_param(strcat(currentBlockPath,'/A5'),'Value',num2str(a(6)));%修改为指定值
    set_param(strcat(currentBlockPath,'/A6'),'Value',num2str(b(1)));%修改为指定值
    set_param(strcat(currentBlockPath,'/A7'),'Value',num2str(b(2)));%修改为指定值
    set_param(strcat(currentBlockPath,'/A8'),'Value',num2str(b(3)));%修改为指定值
    set_param(strcat(currentBlockPath,'/A9'),'Value',num2str(b(4)));%修改为指定值
    set_param(strcat(currentBlockPath,'/A10'),'Value',num2str(b(5)));%修改为指定值
    set_param(strcat(currentBlockPath,'/A11'),'Value',num2str(b(6)));%修改为指定值
end

if strcmp(param,'paramFracA')||strcmp(param,'paramFracB')||strcmp(param,'paramFracOut')
    barrelShift = str2num(get_param(gcbh,'paramFracA'))+str2num(get_param(gcbh,'paramFracB'))-str2num(get_param(gcbh,'paramFracOut'));
    if barrelShift>=0
%      p=Simulink.Mask.get(gcbh);%获取Mask参数
%      p2=p.Parameters(9);
%      p2.set('Value',paramTemp);%这种方式设置Prompt有用，设置Value没用，值是改变了但不会实时显示
       set_param(gcbh,'paramBarrelShift',num2str(barrelShift));
    else
        error('Fraction Part 设置错误');
    end
end

end