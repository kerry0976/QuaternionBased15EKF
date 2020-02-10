            function [R_N, R_E] = earthrad(L)
%===========================================================%
%           function [R_N, R_E] = earthrad(L)               %
%                                                           %
%   This functions computes the north-south and east-west   %
%   radii of curvature (R_N and R_E, respectively) for      %
%   the WGS-84 as a function of geodetic latitude (L).      %
%   The input, L, is in units of radians and the outputs    %
%   R_N and R_E are in units of meters.                     %
%                                                           %
%   Programmer:     Demoz Gebre-Egziabher                   %
%   Created:        July 2, 1998                            %
%   Last Modified:  March 26, 2009                          %
%                                                           %
%                                                           %
%     Copywrite 2009 Demoz Gebre-Egziabher                  %
%     License: BSD, see bsd.txt for details                 %
%===========================================================%

%   Load ellipsoid constants

%wgs_84_parameters;
f = 1/298.257223563;        %   WGS-84 Flattening.
e = sqrt(f*(2 - f));        %   Eccentricity.
%omega_ie = 7.292115e-5;     %   WGS-84 Earth rate (rad/s).
R_0 = 6378137;              %   WGS-84 equatorial radius (m).                            
% R_P = R_0*(1 - f);          %   Polar radius (m).
% mu_E = 3.986004418e14;      %   WGS-84 Earth's gravitational
% Rew = 6.359058719353925e6;
% Rns = 6.386034030458164e6;
%   Compute radii

k = sqrt(1 - (e*sin(L))^2);
R_N = R_0*(1 - e^2)/k^3;
R_E = R_0/k;

%===========================================================%
