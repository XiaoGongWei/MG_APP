%%contrase mutyply station PPP converge about Kalman and SRIF
% get multiply station observe path
destin_dir = '/media/david/DavidPassport/MyGNSS/GNSSData/Observation/IGS_DATA/Fineshed/allStations/';
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

len_total = numel(floder_names);
for k = 1 : len_total
    sprintf('iteam: %g, total: %g, folder: %s', k, len_total, floder_names{k})
    path_Kalman_pos = [floder_names{k} 'Products_Kalman_Static/'];
    path_SRIF_pos = [floder_names{k} 'Products_KalmanOu_Static/'];
    if 0 ~= exist(path_Kalman_pos)
        disp(path_Kalman_pos)
        rmdir(path_Kalman_pos, 's') %rmdir
    end
    if 0 ~= exist(path_SRIF_pos)
        disp(path_SRIF_pos)
        rmdir(path_SRIF_pos, 's') %rmdir
    end
    
end
