clear
clc
% close all
% read ppp file
[ epochData, epoch_sat_num, static_GPS ] = read_ppp( 'Satellite_info.ppp', 'R' );

% plot P code and L Carrier
data_len = numel(epochData);
P_all = zeros(data_len, 32);
L_all = zeros(data_len, 32);
for i = 1: data_len
    allSat = epochData(i).SatData;
    for j = 1:numel(allSat)
        ep_Sat = allSat(j);
        L_all(i, ep_Sat.PRN) = ep_Sat.Correct(1);
        P_all(i, ep_Sat.PRN) = ep_Sat.Correct(2);
    end
end

% plot figure
figure;
plot(L_all, '*')
title([' Carrier residual error'])
xlabel('epoch number')
ylabel('Error (m)')
ylim([-0.1 0.1])
legend_cell = cell(1,32);
for i = 1:32
    legend_cell{i} = num2str(i);
end
legend(legend_cell)

figure;
plot(P_all, '.')
title([' Pseudo-range residuals error'])
xlabel('epoch number')
ylabel('Error (m)')





