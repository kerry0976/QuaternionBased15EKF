            function nav_rate = navrate(v_n,p_n)
% %===========================================================%
% %         function nav_rate = navrate(v_n,p_n)              %
% %                                                           %
% %   This function returns the returns the rotation rate     %
% %   of the locally level north-east-down coordinate frame   %
% %   with respect to an earth fixed coordinate system.       %
% %   The inputs are velocity expressed in north-east-down    %
% %   coordinates in units of meters per second and the       %
% %   geodtic position coordinates in radians (lat and lon)   %
% %   meters for altitude.                                    %
% %                                                           %
% %   Programmer:     Demoz Gebre-Egziabher                   %
% %   Created:        March 20, 2009                          %
% %   Last Modified:  March 28, 2009                          %
% %                                                           %
% %     Copywrite 2009 Demoz Gebre-Egziabher                  %
% %     License: BSD, see bsd.txt for details                 %
% %===========================================================%

[R_N,R_E] = earthrad(p_n(1));

nav_rate(1,1) = v_n(2)/(R_E + p_n(3));
nav_rate(2,1) = -v_n(1)/(R_N + p_n(3));
nav_rate(3,1) = -v_n(2)*tan(p_n(1))/(R_E + p_n(3));

%===========================================================%     

