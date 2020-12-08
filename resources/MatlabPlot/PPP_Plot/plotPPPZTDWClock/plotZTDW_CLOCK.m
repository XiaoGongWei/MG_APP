clc
clear
close all
fileNames = dir('*.txt');
lenName = numel(fileNames);
tic
for i = 1:lenName
    [num2str(i) '/' num2str(lenName) ' -> ' fileNames(i).name]
    fid = fopen(fileNames(i).name,'r');
    dX=[];dY=[];dZ=[];Num=[];
    while ~feof(fid)
        linestr = fgetl(fid);
        dNumi = linestr(37:48);
        Num = [Num str2double(dNumi)];
        dXi = linestr(49:59);
        dYi = linestr(61:76);
        %dZi = linestr(78:88);
        dX = [dX str2double(dXi)];
        dY = [dY str2double(dYi)];
        %dZ = [dZ str2double(dZi)];
    end
    fclose(fid);
%     flags = find(Num < 4);
%     dX(flags)=[];
%     dY(flags)=[];
%     dZ(flags)=[];
%     Num(flags)=[];
    h1 = figure;
    plot(dX,'b-')
    xlabel(['历元数'])
    ylabel('单位(m)')
    legend('天顶湿延迟')
    title(['天顶湿延迟趋势图'])
    grid on
     %坐标轴约束
    %resultVector = [dX(end - 10)];
    %ylim([resultVector-0.03  resultVector+0.03])
    
    
    h2 = figure
    plot(dY,'b-')
    xlabel(['历元数'])
    ylabel('单位(m)')
    legend('接收机钟差')
    title(['接收机钟差趋势图'])
    grid on
   
   % axis([0  3000 -5 0.5])
   %print(h,'-dpng',[fileNames(i).name '.png'])
    saveas(h1,['一天天顶湿延迟变化' '.fig'])
    saveas(h2,['一天接收机钟差' '.fig'])
   %close(h1)
   %close(h2)
end
