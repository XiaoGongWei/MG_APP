function [ Out_mat ] = readmyTxt( txtFilename, beginFlag, selectFlag )
%READMYTXT Summary of this function goes here
%   Detailed explanation goes here
Out_mat = [];

fid = fopen(txtFilename, 'r');
if fid == -1
    error('open file bad.');
    return ;
end

while ~feof(fid)
    strline_t = fgetl(fid);
    strline = strline_t(beginFlag:end);
    strline = strrep(strline, ':', ' ');
    vct_line = str2num(strline);
    myvct = vct_line(selectFlag);
    Out_mat = [Out_mat; myvct];   
end

fclose(fid);

end

