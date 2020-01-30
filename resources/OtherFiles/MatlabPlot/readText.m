function [ spp_pos, ppp_pos,spp_vel, ppp_vel, time_vct ] = readText( txtFilename )
%READTEXT Summary of this function goes here
%   Detailed explanation goes here
spp_pos = [];
ppp_pos = [];
spp_vel = [];
ppp_vel = [];
time_vct = [];

beginFlag = 57;
fid = fopen(txtFilename, 'r');
if fid == -1
    error('open file bad.');
    return ;
end

begin_hour = [];
while ~feof(fid)
    strline_t = fgetl(fid);
    day_time = strline_t(12:23);
    YMD = strsplit(day_time, '-');
    Year = str2num(YMD{1})-2000;
    Mouth = str2num(YMD{2});
    Day = str2num(YMD{3});
    if(isempty(begin_hour))
        begin_hour = Day*24;
    end
    time_strline = strline_t(23:41);
    HMS = strsplit(time_strline, ':');
    hour = str2num(HMS{1});
    Minut = str2num(HMS{2});
    Second = str2num(HMS{3});
    hour_time = Day*24 + hour+Minut/60+Second/3600;
    time_vct = [time_vct; hour_time];
    strline = strline_t(beginFlag:end);
    vct_line = str2num(strline);
    spp_pos = [spp_pos; vct_line(1:3)];
    ppp_pos = [ppp_pos; vct_line(4:6)]; 
    if(length(vct_line)) > 6
        spp_vel = [spp_vel; vct_line(7:9)];
        ppp_vel = [ppp_vel; vct_line(10:12)];
    else
        spp_vel = [spp_vel; 0 0 0];
        ppp_vel = [ppp_vel; 0 0 0];
    end
end
fclose(fid);

time_vct = time_vct - begin_hour;

end

