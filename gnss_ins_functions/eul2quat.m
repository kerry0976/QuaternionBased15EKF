 function q = eul2quat(eul)
%EUL2QUAT Convert 321 euler angles into quaternion representation.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Q = eul2quat(eul) This functions determines the quaternion representation
% given a 321 sequence of euler angles.
%
% SOURCES:
% n/a (see euler angles from dcm online)
%
% INPUT PARAMETERS:
% eul = 3x1 matrix containing euler angles in radians
%     = [yaw pitch roll]^T
%     = [psi theta phi]^T
%
% OUTPUT PARAMETERS:
% q = 4x1 quaternion corresponding to 321 euler angle sequence
%   = [q0 q1 q2 q3]^T NOTE:  first value of matrix, q0, is scalar.
%
% Kail Laughlin
% Updated 11/25/2019
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

eul = eul/2;

q0 = cos(eul(1))*cos(eul(2))*cos(eul(3)) + ...
        sin(eul(1))*sin(eul(2))*sin(eul(3));  

q1 = cos(eul(1))*cos(eul(2))*sin(eul(3)) - ...
        sin(eul(1))*sin(eul(2))*cos(eul(3));

q2 = cos(eul(1))*sin(eul(2))*cos(eul(3)) + ...
        sin(eul(1))*cos(eul(2))*sin(eul(3));  

q3 = sin(eul(1))*cos(eul(2))*cos(eul(3)) - ...
        cos(eul(1))*sin(eul(2))*sin(eul(3));

q = [q0;q1;q2;q3];
 end
