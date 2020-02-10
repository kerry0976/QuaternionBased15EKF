                function g_n = glocal(L,h)
% %===========================================================%
% %             function g_n = glocal(L,h)                    %
% %                                                           %
% %   This function computes the local gravity vector as a    %
% %   function of lattitude (L, in units of radians)          %
% %   and altitude (h, in units of meters).  The output is    %
% %   the gravity vecotor (in units of m/s/s) expressed       %
% %   north, east and down coordinates.  It computes the      %
% %   magnitude of the of local gravity at the reference      %
% %   elliposid using the Somigliana model and makes          %
% %   corrections for altitude. c.f Equation (6.16).          %
% %                                                           %
% %   Programmer:     Demoz Gebre-Egziabher                   %
% %   Created:        July 2, 1998                            %
% %   Last Modified:  March 26, 2009                          %
% %                                                           %
% %                                                           %
% %     Copywrite 2009 Demoz Gebre-Egziabher                  %
% %     License: BSD, see bsd.txt for details                 %
% %===========================================================%

%   Load ellipsoid constants

%wgs_84_parameters;

f = 1/298.257223563;        %   WGS-84 Flattening.
e = sqrt(f*(2 - f));        %   Eccentricity.
omega_ie = 7.292115e-5;     %   WGS-84 Earth rate (rad/s).
R_0 = 6378137;              %   WGS-84 equatorial radius (m).                            
R_P = R_0*(1 - f);          %   Polar radius (m).
mu_E = 3.986004418e14;      %   WGS-84 Earth's gravitational
% Rew = 6.359058719353925e6;
% Rns = 6.386034030458164e6;
%   Load ellipsoid constants

g_n = zeros(3,1);
g_0 = (9.7803253359/(sqrt(1 - (e*sin(L))^2)))*...
        ( 1 + 0.001931853*(sin(L))^2);
k = 1 - (2*h/R_0)*(1 + f + ((omega_ie*R_0)^2)*(R_P/mu_E))...
        + 3*(h/R_0)^2;
g_n(3,1) = k*g_0;    
%===========================================================%

