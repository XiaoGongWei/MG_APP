function [ station_xyz ] = search_snx( snx_coord, station_name )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
station_xyz = [0, 0, 0];
for i = 1 : length(snx_coord)
    single_station = snx_coord(i);
    single_station_name = upper(single_station.name);
    if( 1 == strcmp(single_station_name, upper(station_name)) )
        station_xyz = single_station.XYZ;
        break;
    end
end

