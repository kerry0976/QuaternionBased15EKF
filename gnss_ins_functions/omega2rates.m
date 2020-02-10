        	    function F =omega2rates(psi_nb)
% %===========================================================%
% %             function F =omega2rates(psi_nb)               %
% %                                                           %
% %  This function determines the matrix F which maps angular %
% %  of the body frame into Euler angle rates.  The input     %
% %  is a vector of the 3-2-1 Euler angle sequence which      %
% %  rotate the "n" frame into the "b" frame.  These are the  %
% %  standard aerospace sequence of yaw, pitch and roll.      %
% %  Thus, the input vector psi_nb is defined to be:          %
% %                                                           %
% %               psi_nb = [roll pitch yaw]                   %  
% %                                                           %
% %  where the units for the Euler angles is radians          %
% %  c.f. Equations (6.7) and (6.8)                           %
% %                                                           %
% %   Programmer:     Demoz Gebre-Egziabher                   %
% %   Created:        July 2, 1998                            %
% %   Last Modified:  March 26, 2009                          %
% %                                                           %
% %     Copywrite 2009 Demoz Gebre-Egziabher                  %
% %     License: BSD, see bsd.txt for details                 %
% %===========================================================%

%   Check input arguments for validity

roll = psi_nb(1);
pitch = psi_nb(2);
singular_tol = 1e-4;

if (pitch < 0)
    singular_check = abs(pitch + pi/2);
else
    singular_check = abs(pitch - pi/2);
end

if (singular_check < singular_tol)
    error('Euler angle singularity at pitch = 90 degrees encountered.');
end

F = zeros(3,3);

F(1,1) = 1; F(1,2) = sin(roll)*tan(pitch);  F(1,3) = cos(roll)*tan(pitch);
F(2,1) = 0; F(2,2) = cos(roll);             F(2,3) = -sin(roll);
F(3,1) = 0; F(3,2) = sin(roll)/cos(pitch);  F(3,3) = cos(roll)/cos(pitch);              


