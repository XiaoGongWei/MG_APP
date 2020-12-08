clc
clear
close all

SatSys = 'G';%选择系统：G,C,R,E 只能选择一个系统

fileNames = dir('*.txt');
lenName = numel(fileNames);
maxEpoch = -1;
tic
for i = 1:lenName
    [num2str(i) '/' num2str(lenName) ' -> ' fileNames(i).name]
    fid = fopen(fileNames(i).name,'r');
    dX=[];dY=[];dZ=[];Num=[];
    while ~feof(fid)
        linestr = fgetl(fid);
        dXi = linestr(1:10);
        dYi = linestr(14:15);
        SatType =  linestr(13);
        
        dXi = str2double(dXi);
        dYi = str2double(dYi);
        if SatType ~= SatSys
            continue;
        end
        if ( dXi> maxEpoch)
            maxEpoch = dXi;
        end
        %dZi = linestr(78:88);
        dX = [dX dXi];
        dY = [dY dYi];
        %dZ = [dZ str2double(dZi)];
    end
    fclose(fid);
%     flags = find(Num < 4);
%     dX(flags)=[];
%     dY(flags)=[];
%     dZ(flags)=[];
%     Num(flags)=[];
    h1 = figure;
    plot(dX,dY,'.')
   
    strSystem = '';
    
    switch SatSys
        case 'G'
            strSystem ='GPS';
        case 'R'
            strSystem ='GLONASS';
        case 'C'
            strSystem ='BDS';
        case 'E'
            strSystem ='Galieo';
        otherwise 
            strSystem = '系统不存在';
    end
    legend(strSystem);
    title([strSystem  '系统观测卫星分布']);
    grid on;
 
    Ylabel = cell(32,1);
    for i = 1:32
        if i < 10
            Ylabel{i} = [SatSys '0' num2str(i) ];
        else
            Ylabel{i} = [SatSys num2str(i)];
        end
    end
    
    set(gca,'YTick',1:1:32)  
    set(gca,'YTickLabel',Ylabel)  
    
   
    xlabel(['历元数']);
    ylabel('卫星PRN');
   
   xlim([0 maxEpoch+121]);
   %print(h,'-dpng',[fileNames(i).name '.png']);
   saveas(h1,['一天观测卫星分布图' '.fig']);
   %close(h1);
   %close(h2);
end
