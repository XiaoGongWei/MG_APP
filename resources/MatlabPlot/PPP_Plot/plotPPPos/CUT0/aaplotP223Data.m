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
        linestr = linestr(48:end);
        dXi = linestr(1:11);
        dYi = linestr(13:23);
        dZi = linestr(24:35);
        dX = [dX str2double(dXi)];
        dY = [dY str2double(dYi)];
        dZ = [dZ str2double(dZi)];
    end
    fclose(fid);
    dX = abs(dX);
    dY = abs(dY);
    dZ = abs(dZ);
    
    h = figure('Name',fileNames(i).name);
    plot(dX,'r-')
    hold on
    plot(dY,'g-')
    hold on
    plot(dZ,'b-');
    grid on
    legend('dE','dN','dU')
    ylabel('Unit: (m)')
    title(['PPP Algorithm: ' fileNames(i).name])
    %�����Լ��
    resultVector = [dX(end - 10) dY(end - 10) dZ(end - 10)];
    minResult = min(resultVector);
    maxResult = max(resultVector);
    dmax_min = maxResult - minResult;
    deltaMaxMin = dmax_min./2;
    ylim([-0.1  0.2])
    %ylim([minResult-deltaMaxMin  maxResult+deltaMaxMin])
    %ȷ���ڼ�����Ԫ����(��N����Ԫ�����Сƫ����ֵ��deltaM����λm����)
    N = 500;
    deltaM = 0.01;%1cm
    lenXYZ = length(dX);
    bestEpoch = 0;
    store_X = [];
    store_Y = [];
    store_Z = [];
    for epoch =1:lenXYZ
        if epoch + N > lenXYZ
             store_X = dX(epoch:end);
             store_Y = dY(epoch:end);
             store_Z = dZ(epoch:end);
        else
            store_X = dX(epoch:epoch+N);
            store_Y = dY(epoch:epoch+N);
            store_Z = dZ(epoch:epoch+N);
        end
        
        minX = min(store_X);
        maxX = max(store_X);
        minY = min(store_Y);
        maxY = max(store_Y);
        minZ = min(store_Z);
        maxZ = max(store_Z);
        
        bestEpoch = epoch;
        %�������������deltaM
        if (abs(maxX - minX) < deltaM && abs(maxY - minY) < deltaM && abs(maxZ - minZ) < deltaM)
            break;
        end

    end
    %�ҵ�������Ԫ�����---END
    line([bestEpoch bestEpoch],[minResult-deltaMaxMin maxResult+deltaMaxMin],'LineWidth',3,'Color',[1,0,0]);
    text(bestEpoch,(maxResult+minResult)/2,['Convergent straight line, at ' num2str(bestEpoch) ' epoch']);
    xlabel(['Convergent time: ' num2str(bestEpoch*30/3600) 'h.' ])
    saveas(h,[fileNames(i).name '.fig']);
   % axis([0  3000 -5 0.5])
   %print(h,'-dpng',[fileNames(i).name '.png'])
   %close(h)
end
