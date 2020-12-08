%%contrase mutyply station PPP converge about Kalman and SRIF
% get multiply station observe path
destin_dir = '/media/david/DavidPassport/MyFiles/MyPapers/GNSS_PPP_Paper/Paper_Data/MG_APPS_DATA/MG_APPS_DATA/COCO/allStations/';
file_names = dir(destin_dir);
flag = 1;
floder_names = [];
station_names = [];
for k = 1 : length(file_names)
    if(file_names(k).isdir &&~strcmp( file_names(k).name, '.') && ~strcmp( file_names(k).name, '..'))
        flood_name = file_names(k).name;
        temp_path = [destin_dir file_names(k).name '/'];
        station_names = [station_names; flood_name(1:4)];
        floder_names{flag} = temp_path;
        flag = flag + 1;
    end
end
% read snx coord 
[snx_coord]=readsnx('/media/david/DavidPassport/MyFiles/MyPapers/GNSS_PPP_Paper/Paper_Data/MG_APPS_DATA/MG_APPS_DATA/igs18P2030.snx');

% set file_type and plot
file_type = 'position.txt';
set(0,'DefaultFigureVisible', 'off')
store_Kalman = [];
store_SRIF = [];
store_Kalman_Pos=[];
store_SRIF_Pos=[];
store_SNX_Pos=[];
rms_3D_m = [];
len_total = numel(floder_names);
for k = 1 : len_total
    sprintf('iteam: %g, total: %g, folder: %s', k, len_total, floder_names{k})
    path_Kalman_pos = [floder_names{k} 'Products_Kalman_Static_G_1e6/'];
    path_SRIF_pos = [floder_names{k} 'Products_SRIF_Static_G_1e8/'];
    kalman_file = [path_Kalman_pos file_type];
    srif_file = [path_Kalman_pos file_type];
    % get snx pos
    snx_xyz = search_snx( snx_coord, station_names(k, :) );
    % get best epoch
    kalman_best_epoch = 0;
    if exist(kalman_file, 'file')
        [kalman_best_epoch, best_XYZ_kalman] = plot_products(path_Kalman_pos, file_type, snx_xyz);
    end
    srif_best_epoch = 0;
    if exist(srif_file, 'file')
        [srif_best_epoch, best_XYZ_srif] = plot_products(path_SRIF_pos, file_type, snx_xyz);
    end
    % store best epoch
    store_Kalman = [store_Kalman; kalman_best_epoch];
    store_SRIF = [store_SRIF; srif_best_epoch];
    % store filter pos
    store_Kalman_Pos = [store_Kalman_Pos; best_XYZ_kalman];
    store_SRIF_Pos = [store_SRIF_Pos; best_XYZ_srif];
    % store snx
    store_SNX_Pos = [store_SNX_Pos; snx_xyz];
    if(sum(snx_xyz) ~= 0)
        rms_3D_m = [rms_3D_m; norm(snx_xyz - best_XYZ_kalman) norm(snx_xyz - best_XYZ_srif)];
    end
end
%plot best epoch
set(0,'DefaultFigureVisible', 'on')
h1 = figure('Name','Kalman vs SRIF for Convergent epoch');
hold on
plot(store_Kalman, 'r-s');
plot(store_SRIF, 'b-o');
legend('Kalman', 'SRIF');
xlabel('station number');
ylabel('Convergent epoch');
title('Kalman vs SRIF for Convergent epoch');
saveas(h1, 'Kalman_SRIF.jpg', 'jpg')
% saveas(h1, 'Kalman_SRIF.eps', 'psc2')
% saveas(h1, 'Kalman_SRIF.tiff', 'tiffn')

%analysis Convergent epoch: SRIF > Kalman
srif_gt_kalman = find(store_SRIF > (store_Kalman+40));
SRIF_gt_Kalman = [];
SRIF_gt_Kalman1 = [];
fid = fopen('SRIF_gt_Kalman_stations.txt', 'w');
for i = 1 : length(srif_gt_kalman)
    linstr = floder_names{srif_gt_kalman(i)};
    SRIF_gt_Kalman{i} = floder_names{srif_gt_kalman(i)};
    SRIF_gt_Kalman1 = [SRIF_gt_Kalman1; linstr];
    % write to txt
    writStr = [num2str(srif_gt_kalman(i)) ' -> ' linstr '\r\n'];
    fprintf(fid, writStr);
end
fclose(fid);

save all_data.mat
% find kalman solver error stations
a = store_SNX_Pos - store_Kalman_Pos;
a_index = [];
for i = 1 : size(a,1)
    a_index = [a_index; norm(a(i, :))];
end
srif_gt_kalman = find(a_index > 0.05 & a_index < 100);
SRIF_gt_Kalman = [];
SRIF_gt_Kalman1 = [];
fid = fopen('Kalman_Error.txt', 'w');
for i = 1 : length(srif_gt_kalman)
    linstr = station_names(srif_gt_kalman(i), :);
    sum_3D = a_index(srif_gt_kalman(i));
    % write to txt
    writStr = [num2str(srif_gt_kalman(i)) ' -> ' linstr ' -> Error: ' num2str(sum_3D) 'm. ' '\r\n'];
    fprintf(fid, writStr);
%     fwrite(fid, writStr);
end
fclose(fid);
% find srif solver error stations
a = store_SNX_Pos - store_SRIF_Pos;
a_index = [];
for i = 1 : size(a,1)
    a_index = [a_index; norm(a(i, :))];
end
srif_gt_kalman = find(a_index > 0.05 & a_index < 100);
SRIF_gt_Kalman = [];
SRIF_gt_Kalman1 = [];
fid = fopen('SRIF_Error.txt', 'w');
for i = 1 : length(srif_gt_kalman)
    linstr = station_names(srif_gt_kalman(i), :);
    sum_3D = a_index(srif_gt_kalman(i));
    % write to txt
    writStr = [num2str(srif_gt_kalman(i)) ' -> ' linstr ' -> Error: ' num2str(sum_3D) 'm. ' '\r\n'];
    fprintf(fid, writStr);
%     fwrite(fid, writStr);
end
fclose(fid);


