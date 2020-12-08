function [bestEpoch, best_XYZ] = plot_products(file_path, file_name, snx_xyz)
lenName = 1;
fileNames = [file_path file_name];
for i = 1:lenName
    fid = fopen(fileNames,'r');
    if fid == -1 
        return ;
    end
    dX=[];dY=[];dZ=[];Num=[];
    while ~feof(fid)
        linestr = fgetl(fid);
        dNumi = linestr(57:end);
        Num = [str2num(dNumi)];
        dXi = Num(4);
        dYi = Num(5);
        dZi = Num(6);
        if dXi*dYi*dZi ~=0
            dX = [dX (dXi)];
            dY = [dY (dYi)];
            dZ = [dZ (dZi)];
        end      
    end
    fclose(fid);
    % get best (end) dX dY dZ
    best_XYZ = [dX(end) dY(end) dZ(end)];
    if(sum(snx_xyz) == 0)
        snx_xyz = best_XYZ;
    end
    
    dX = snx_xyz(1) - dX;
    dY = snx_xyz(2) - dY;
    dZ = snx_xyz(3) - dZ;
    
    h = figure('Name',fileNames);
    plot(dX,'r-')
    hold on
    plot(dY,'g-')
    hold on
    plot(dZ,'b-');
    grid on
    legend('dX', 'dY', 'dZ')
    ylabel('Unit: (m)')
    title(['PPP Algorithm: ' file_name])
    %�����Լ��
    resultVector = [dX(end - 10) dY(end - 10) dZ(end - 10)];
    minResult = min(resultVector);
    maxResult = max(resultVector);
    dmax_min = maxResult - minResult;
    deltaMaxMin = dmax_min./2;
    %ylim([-0.05  0.1])
    %ylim([minResult-deltaMaxMin  maxResult+deltaMaxMin])
    %ȷ���ڼ�����Ԫ����(��N����Ԫ�����Сƫ����ֵ��deltaM����λm����)
    N = 100;
    deltaM = 0.1;%10cm
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
    saveas(h,[fileNames '.jpg']);
   % axis([0  3000 -5 0.5])
   %print(h,'-dpng',[fileNames(i).name '.png'])
    close(h)
end
