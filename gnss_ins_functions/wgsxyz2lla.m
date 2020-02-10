        function [lat, lon, alt] = wgsxyz2lla(p_e)
% %===========================================================%
% %     function [lat, lon, alt] = wgsxyz2lla(p_e)            %
% %                                                           %
% %   This function returns the geodtic position coordinates  %
% %   for the position vector p_e given in the WGS-84 Earth   %
% %   Centered Earth Fixed (ECEF) coordinate system (in       %
% %   units of meters).  The outputs of the function are      %
% %   latitude (lat) and longitude (lon) in units of          %
% %   degrees and altitude (alt) above the WGS-84 reference   %
% %   ellipsoid in units of meters.                           %
% %                                                           %
% %   c.f.    Farrel and Barth, GPS & Inertal Navigation,     %
% %           McGraw-Hill, 1999. pp. 29.                      %
% %                                                           %
% %   Programmer:     Demoz Gebre-Egziabher                   %
% %   Created:        March 20, 2009                          %
% %   Last Modified:  March 28, 2009                          %
% %                                                           %
% %     Copywrite 2009 Demoz Gebre-Egziabher                  %
% %     License: BSD, see bsd.txt for details                 %
% %===========================================================%

%   Load ellipsoid constants

wgs_84_parameters;

%   Place holder for output and temporary variables

p_n = zeros(3,1);
x = p_e(1);         
y = p_e(2);
z = p_e(3);

%   Calculate longitude

lon = atan2(y,x)*(180/pi);

%   Start computing intermediate variables needed
%   to compute altitude

p = norm([x y]);
E = sqrt(R_0^2 - R_P^2);
F = 54*(R_P*z)^2;
G = p^2 + (1 - e^2)*z^2 - (e*E)^2;
c = e^4*F*p^2/G^3;
s = (1 + c + sqrt(c^2 + 2*c))^(1/3);
P = (F/(3*G^2))/((s + (1/s) + 1)^2);
Q = sqrt(1 + 2*e^4*P);
k_1 = -P*e^2*p/(1 + Q);
k_2 = 0.5*R_0^2*(1 + 1/Q);
k_3 = -P*(1 - e^2)*z^2/(Q*(1 + Q));
k_4 = -0.5*P*p^2;
r_0 = k_1 + sqrt(k_2 + k_3 + k_4);
k_5 = (p - e^2*r_0);
U = sqrt(k_5^2 + z^2);
V = sqrt(k_5^2 + (1 - e^2)*z^2);

alt = U*(1 - (R_P^2/(R_0*V)));

%   Compute additional values required for computing
%   latitude

z_0 = (R_P^2*z)/(R_0*V);
e_p = (R_0/R_P)*e;

lat = atan((z + z_0*(e_p)^2)/p)*(180/pi);

%===========================================================%     
