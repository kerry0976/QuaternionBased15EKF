    function enu = wgsxyz2enu(p_e,ref_lat,ref_lon,ref_alt)
% %===========================================================%
% % function enu = wgsxyz2enu(p_e,ref_lat,ref_lon,ref_alt)    %
% %                                                           %
% %   This function returns the position coordinates of       %
% %   a user at the WGS-84 ECEF coordiantes of p_e in         %
% %   east-north-up coordinates relative to the reference     %
% %   position located at ref_lat (latitude in degrees),      %
% %   ref_lon (longitude in degrees) and ref_alt (in meters). %
% %                                                           %
% %   Programmer:     Demoz Gebre-Egziabher                   %
% %   Created:        March 20, 2009                          %
% %   Last Modified:  March 28, 2009                          %
% %                                                           %
% %     Copywrite 2009 Demoz Gebre-Egziabher                  %
% %     License: BSD, see bsd.txt for details                 %
% %===========================================================%


%   Calculate the relative position vector

p_e_ref = wgslla2xyz(ref_lat, ref_lon, ref_alt);
delta_xyz = p_e - p_e_ref;

%   Calculate ENU coordinates
        
enu(1,1)= -sind(ref_lon)*delta_xyz(1) + cosd(ref_lon)*delta_xyz(2);

enu(2,1)= -sind(ref_lat)*cosd(ref_lon)*delta_xyz(1) - ...
            sind(ref_lat)*sind(ref_lon)*delta_xyz(2) + ...
            cosd(ref_lat)*delta_xyz(3);
        
enu(3,1)= cosd(ref_lat)*cosd(ref_lon)*delta_xyz(1) + ...
            cosd(ref_lat)*sind(ref_lon)*delta_xyz(2) + ... 
            sind(ref_lat)*delta_xyz(3);      
%===========================================================%     
