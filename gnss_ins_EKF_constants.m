% %===========================================================%
% %                   gnss_ins_constants.m                    %
% %                                                           %
% %   This m-file contains various constants that are used    %
% %   in the MATLAB implementation of the GNSS/INS            %
% %   integration routines.                                   %
% %                                                           %
% %   Programmer:     Demoz Gebre-Egziabher                   %
% %   Created:        March 26, 2009                          %
% %   Last Modified:  June 25, 2009                           %
% %                                                           %
% %  Copywrite 2009 Demoz Gebre-Egziabher                     %
% %  License: BSD, see bsd.txt for details                    %
% %===========================================================%


d2r = pi/180;           %   Degrees to radians
r2d = 1/d2r;            %   Radians to degrees

g = 9.81;               %   Nominal value of the gravity vector
                        %   in m/s^2.  Used for primarily for 
                        %   plotting purposes


wgs_84_parameters;      %   Load in WGS-84 ellipsoid model parameters
