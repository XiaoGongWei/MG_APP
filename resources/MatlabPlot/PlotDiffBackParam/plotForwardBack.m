clear
clc
close all;

forwardFloder = './Products_Kalman_Static/';
BackFloder = './Products_BackSmooth_Kalman_Static/';
save_floder = './plotResualt';
savefile_type = '.jpg';
%% plot ZTDW_Clock.txt
ZTD_Clk_name = 'ZTDW_Clock.txt';
ZTDW_Clock_Forward = fullfile(forwardFloder, ZTD_Clk_name);
ZTDW_Clock_Back = fullfile(BackFloder, ZTD_Clk_name);
forw_ZTD_Clk_DATA = readmyTxt(ZTDW_Clock_Forward, 52, [1,2]);
back_ZTD_Clk_DATA = readmyTxt(ZTDW_Clock_Back, 52, [1,2]);
error_ZTD_Clk = back_ZTD_Clk_DATA - forw_ZTD_Clk_DATA;

% plot error of ZTD
h_ZTD = figure('name', 'ZTD Error');
hold on
plot(error_ZTD_Clk(:,1))
xlabel('epoch 30s')
ylabel('Error Unit: m')
title(' diff of ZTD')
xlim([1, 100])
% save image
save_filename = [fullfile(save_floder, 'ZTD_Error') savefile_type];
saveas(h_ZTD, save_filename);

% plot error of Clk
h_Clk = figure('name', 'Clk Error');
hold on
plot(error_ZTD_Clk(:,2))
xlabel('epoch 30s')
ylabel('Error Unit: m')
title(' diff of Clk')
xlim([1, 100])
% save image
save_filename = [fullfile(save_floder, 'Clk_Error') savefile_type];
saveas(h_Clk, save_filename);

%% plot position.txt
Position_name = 'postion.txt';
Position_Forward = fullfile(forwardFloder, Position_name);
Position_Back = fullfile(BackFloder, Position_name);
forw_Position_DATA = readmyTxt(Position_Forward, 108, [1,2,3]);
back_Position_DATA = readmyTxt(Position_Back, 108, [1,2,3]);
true_pos = forw_Position_DATA(end,:);
error_Position = forw_Position_DATA - repmat(true_pos, size(forw_Position_DATA,1), 1);

% plot error of Position
h_Position = figure('name', 'Position Error');
hold on
plot(error_Position)
xlabel('epoch 30s')
ylabel('Error Unit: m')
title(' diff of Position')
xlim([1, 100])
% save image
save_filename = [fullfile(save_floder, 'Position_Error') savefile_type];
saveas(h_Position, save_filename);

%% plot Ambiguity.txt
eposchs_num = size(forw_Position_DATA,1);
GPS_lamda3 = 1;
Amb_path = fullfile(forwardFloder, 'Ambiguity/*.txt');
Gi_txt_list = dir(Amb_path);


for i = 1:length(Gi_txt_list)
    Ambiguity_name = 'Ambiguity';
    Gi_name = Gi_txt_list(i).name;
    Ambiguity_Forward = fullfile(forwardFloder, Ambiguity_name, Gi_name);
    Ambiguity_Back = fullfile(BackFloder, Ambiguity_name, Gi_name);
    forw_Ambiguity_DATA = readmyTxt(Ambiguity_Forward, 1, [1, 4]);
    back_Ambiguity_DATA = readmyTxt(Ambiguity_Back, 1, [1, 4]);
    error_Ambiguity = (forw_Ambiguity_DATA - back_Ambiguity_DATA).*GPS_lamda3;
    % jump epoch least 100
    if(forw_Ambiguity_DATA(1,1) > 100)
        continue;
    end

    % plot error of Ambiguity
    h_Ambiguity = figure('name', ['Ambiguity Error -> ' Gi_name]);
    hold on
    plot(forw_Ambiguity_DATA(:,1), error_Ambiguity(:,2), '-*')
    xlabel('epoch 30s')
    ylabel('Error Unit: zhou')
    title([' diff of Ambiguity -> ' Gi_name])
    xlim([1, 100])
    % save image
    save_filename = [fullfile(save_floder, Gi_name) savefile_type];
    saveas(h_Ambiguity, save_filename);
    close(h_Ambiguity);
end

