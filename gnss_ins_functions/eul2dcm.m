 function C=eul2dcm(eul)
%EUL2DCM Convert 321 euler angles into DCM.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% C = eul2dcm(eul) This functions determines the direction cosine matrix C
% that transforms a vector in a reference axis system to another reference
% axis system. Input euler angles must cohere to a 321 rotation sequence.
%
% SOURCES:
% n/a (see dcm from euler angles online)
%
% INPUT PARAMETERS:
% eul = 3x1 matrix containing euler angles in radians
%     = [yaw pitch roll]^T
%     = [psi theta phi]^T
%
% OUTPUT PARAMETERS:
% C = 3x3 direction cosine matrix corresponding to 321 euler angle sequence
%
% Kail Laughlin
% Updated 11/25/2019
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ps=eul(1); th=eul(2); ph=eul(3);

C1=[1 0 0; 0 cos(ph) sin(ph); 0 -sin(ph) cos(ph)];
C2=[cos(th) 0 -sin(th); 0 1 0; sin(th) 0 cos(th)];
C3=[cos(ps) sin(ps) 0; -sin(ps) cos(ps) 0; 0 0 1];

C=C1*C2*C3;
end
