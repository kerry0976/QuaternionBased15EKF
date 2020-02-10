        function Q_k = discrete_process_noise(F,G,dt,Q)
% %===========================================================%
% %     function Q_k = discrete_process_noise(F,G,dt,Q)       %
% %                                                           %
% %   This functions determines the equivalent discrete       %   
% %   process noise matrix Q_k for the continuous system      %
% %                                                           %
% %                  x_dot = Fx + Gw                          %
% %                                                           %
% %   driven by white noise, w, of power spectral density Q.  %
% %   The sampling time is dt.  This is a simplification of   %
% %   the function disrw.m written by Franklin, Powell and    %
% %   Workman described in [1] to enhance speed.              %
% %                                                           %
% %   [1]   G. F. Franklin, J. D. Powell and Workman,         %
% %         Digital Control of Dynamic Systems, 2nd Edition.  %
% %                                                           %
% %   Programmer:     Demoz Gebre-Egziabher                   %
% %   Created:        July 2, 1998                            %
% %   Last Modified:  June 26, 2009                           %
% %                                                           %
% %     Copywrite 2009 Demoz Gebre-Egziabher                  %
% %     License: BSD, see bsd.txt for details                 %
% %===========================================================%

[r,c] = size(F);
Q_k = (eye(r) + dt*F)*(dt*G*Q*G');
