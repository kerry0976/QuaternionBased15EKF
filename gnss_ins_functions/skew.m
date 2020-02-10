                    function S =skew(v_in)
% % ==========================================================%     
% %                 function S = skew(v_in)                   %
% %                                                           %
% %   This function returns the skew symmetric matrix S made  %
% %   from the entries of the input vector v_in.              %
% %                                                           %
% %   Programmer:     Demoz Gebre-Egziabher                   %
% %   Created:        July 2, 1998                            %
% %   Last Modified:  March 26, 2009                          %
% %                                                           %
% %     Copywrite 2009 Demoz Gebre-Egziabher                  %
% %     License: BSD, see bsd.txt for details                 %
% %===========================================================%

S =[0 -v_in(3) v_in(2); v_in(3) 0 -v_in(1); -v_in(2) v_in(1) 0];

%===========================================================%     

