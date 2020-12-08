function [ epochData, epoch_sat_num, static_GPS ] = read_ppp( ppp_file , sysType)
%READ_PPP Summary of this function goes here
%   Detailed explanation goes here
epochData = [];
static_GPS = zeros(1,32);
epoch_sat_num = [];
fid = fopen(ppp_file, 'r');
if fid == -1
    errordlg('open file bad!')
    return ;
end
epoch_flag = 1;
while ~feof(fid)
    line_str = fgetl(fid);
    if isempty(line_str)
        break;
    end
    while line_str(1) ~= '>' && ~feof(fid)
        line_str = fgetl(fid);
    end
    % epoch num
    epoch_num = str2num(line_str(12:end)); % >epoch_num:0
    if epoch_num == 9
        a = 0;
    end
%     epoch_num
    line_str = fgetl(fid);% Satellite Number:987,(yyyy-mm-dd-hh-mm-ss):2018- 1- 2  0: 0: 0.0000000 ,ztd:    
    sat_num = str2num(line_str(18:20));
    year = str2num(line_str(44:47));
    mouth = str2num(line_str(49:50));
    day = str2num(line_str(52:53));
    hour = str2num(line_str(55:56));
    minute = str2num(line_str(58:59));
    second = str2num(line_str(61:70));
    % save epoch number and date to struct 
    epochData(epoch_flag).epoch_num = epoch_num;
    epochData(epoch_flag).Year = year; epochData(epoch_flag).Mouth = mouth; epochData(epoch_flag).Day = day;
    epochData(epoch_flag).Hour = hour; epochData(epoch_flag).Minute = minute; epochData(epoch_flag).Second = second;
    SatData = [];
    satNum = 1;
    for in = 1:sat_num
        line_str = fgetl(fid); % G03:    -0.00000000,    -0.85783420, -12175828.5611, -22639807.2781,  -6797319.1982,     -5149.6926,        12.3922,       244.0495,        10.5475,         4.6060,        -0.1830,       -32.0236,        -0.0545,         0.0000,         7.4190,         5.7810,         0.0657,         0.0447,         0.2980             
        SystemType = line_str(1);
        if(sysType ~= SystemType)
            continue;
        end
        PRN = str2num(line_str(2:3));
        cmp_line = line_str(6:end);
        correct_val = str2num(cmp_line);
        SatData(satNum).SysType = SystemType;
        SatData(satNum).PRN = PRN;
        SatData(satNum).Correct = correct_val;
        static_GPS(PRN) = static_GPS(PRN) + 1;
        satNum = satNum + 1;
    end
    % save sat date to struct
    epochData(epoch_flag).SatData = SatData;
    epoch_sat_num = [epoch_sat_num; sat_num];
    epoch_flag = epoch_flag + 1;
    
end





end

