            function PSI_nb = Cbn2eul(Cbn)
% %===========================================================%
% %         function PSI_nb =Cbn2eul(Cbn)                     %
% %                                                           %
% %   This functions extracts the 3-2-1 Euler angle sequence  % 
% %   corrosponding to the direction cosine matrix (body to   %
% %   navigation frame).  The output vector PSI_nb is defined %
% %   as follows:                                             %
% %                                                           %
% %               PSI_nb = [roll pitch yaw]                   %  
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

C = Cbn';

PSI_nb(1,1) = atan2(C(2,3),C(3,3));  %     roll
PSI_nb(3,1) = atan2(C(1,2),C(1,1));  %     yaw
PSI_nb(2,1) = -asin(C(1,3));         %     pitch


%*************************************************************************%
