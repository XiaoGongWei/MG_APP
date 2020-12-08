function [allStations]=readsnx(filename)


fid=fopen(filename);
if fid==-1
    disp('open File bad!');
    return;
end

num=0;
line=fgetl(fid);
while min(size(findstr(line,'+SOLUTION/ESTIMATE')))==0
    line=fgetl(fid);
end
while min(size(findstr(line,'-SOLUTION/ESTIMATE')))==0
    line=fgetl(fid);
    num=num+1;
end

num1=num-10;
num2=num1/3;

frewind(fid);
while min(size(findstr(line,'+SOLUTION/ESTIMATE')))==0
    line=fgetl(fid);
end
line=fgetl(fid);
line=fgetl(fid);
for i=1:num2
    name=strtrim(line(14:18));
    x=str2num(line(47:68));
    line=fgetl(fid);
    y=str2num(line(47:68));
    line=fgetl(fid);
    z=str2num(line(47:68));
    allStations(i).name = name;
    allStations(i).XYZ = [x, y, z];
    line=fgetl(fid);
end













