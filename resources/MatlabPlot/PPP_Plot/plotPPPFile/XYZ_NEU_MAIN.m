clear
clc
% close all;
%% XYZ_NEU
load true_pos_BRST.mat
vpa(true_pos)
[ spp_pos, ppp_pos,spp_vel, ppp_vel, time_vct ] = readText( 'position.txt' );


sol_PPP = ppp_pos;
bad_flag = find(sol_PPP(:,1)==0);

sol_PPP(bad_flag,:) = [];
time_vct(bad_flag,:) = [];

data_len = size(sol_PPP,1);
% Switch to NEU
NEU1 = [];
for nk = 1:data_len
    dNEU1 = XYZ_NEU(sol_PPP(nk,:) , true_pos - sol_PPP(nk,:));
    NEU1 = [NEU1;dNEU1'];
end
X = time_vct;
% X = (1:data_len);
figure;
hold on
plot(X, NEU1(:,1), 'r')
plot(X, NEU1(:,2), 'g')
plot(X, NEU1(:,3), 'b')
legend('N', 'E', 'U')
xlabel('GPS time(hour)')
ylabel('Error(m)')
ylim([-0.5 0.5])
grid on
line([X(1) X(end)], [0 0], 'color', 'k')
legend('N', 'E', 'U')

RMSdata = NEU1(data_len-5000:data_len,:);
rms(RMSdata)

NEU3d = [];
for i = 1:size(NEU1,1)
    NEU3d = [NEU3d; norm(NEU1(i, :))];
end
figure;
hold on
plot(X, NEU3d, 'r')
legend('3D')
ylim([0 0.5])
xlabel('GPS time(min)')
ylabel('Error(m)')












