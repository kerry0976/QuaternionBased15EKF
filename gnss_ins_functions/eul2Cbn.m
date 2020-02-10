        	    function Cbn = eul2Cbn(PSI_nb)
% %===========================================================%
% %             function Cbn = eul2Cbn(PSI_nb)                %
% %                                                           %
% %   This functions determines the transformation matrix     %
% %   Cbn which transforms vectors expressed in the           %
% %   "b" frame to vectors expressed in the "n" frame.        %
% %   However, note that the inputs are the 3-2-1 Euler       %
% %   angle sequence which rotate the "n" frame into the      %
% %   "b" frame.  These are the standard aerospace sequence   %  
% %   of yaw, pitch and roll.  Thus, the input vector         %
% %   PSI_nb is defined to be (in radians):                   %
% %                                                           %
% %               PSI_nb = [roll pitch yaw]                   %
% %                      = [phi_nb theta_nb psi_nb]           %
% %                                                           %
% %   c.f. Equations (6.6) and (6.11)                         %
% %                                                           %
% %   Programmer:     Demoz Gebre-Egziabher                   %
% %   Created:        July 2, 1998                            %
% %   Last Modified:  March 26, 2009                          %
% %                                                           %
% %     Copywrite 2009 Demoz Gebre-Egziabher                  %
% %     License: BSD, see bsd.txt for details                 %
% %===========================================================%


C1 = [  1               0           0           ;...
        0       cos(PSI_nb(1))  sin(PSI_nb(1))  ;...
        0       -sin(PSI_nb(1)) cos(PSI_nb(1))] ;
    
    
C2 = [  cos(PSI_nb(2))      0       -sin(PSI_nb(2)) ;...
            0               1           0           ;...
        sin(PSI_nb(2))      0       cos(PSI_nb(2))] ;
    
C3 = [  cos(PSI_nb(3))      sin(PSI_nb(3))      0       ;...
        -sin(PSI_nb(3))     cos(PSI_nb(3))      0       ;...
                0               0               1]      ;

Cbn = (C1*C2*C3)';
%===========================================================%
