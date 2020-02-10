      function ned = wgsxyz2ned(p_e,ref_lat,ref_lon,ref_alt)
% %===========================================================%
% %   function ned = wgsxyz2ned(p_e,ref_lat,ref_lon,ref_alt)  %
% %                                                           %
% %   This function returns the position coordinates of       %
% %   a user at the WGS-84 ECEF coordiantes of p_e in         %
% %   north-east-down coordinates relative to the reference   %
% %   position located at ref_lat (latitude in degrees),      %
% %   ref_lon (longitude in degrees) and ref_alt (in meters). %
% %                                                           %
% %   Note: This function relies on (or calls) the function   %
% %         wgsxyz2enu.m                                      %
% %                                                           %
% %   Programmer:     Demoz Gebre-Egziabher                   %
% %   Created:        March 20, 2009                          %
% %   Last Modified:  March 28, 2009                          %
% %                                                           %
% %     Copywrite 2009 Demoz Gebre-Egziabher                  %
% %     License: BSD, see bsd.txt for details                 %
% %===========================================================%


%   Calculate ENU coordinates

enu = wgsxyz2enu(p_e,ref_lat,ref_lon,ref_alt);

%   Define ENU to NED rotation matrix

C = [0 1 0;1 0 0;0 0 -1];

%   From NED position vector

ned = C*enu;

%===========================================================%     
