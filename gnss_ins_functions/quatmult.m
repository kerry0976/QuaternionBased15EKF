 function [quat] = quatmult(q1,q2);
%QUATMULT  Function to multiply two quaternions.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [QUAT] = quatmult(q1,q2) The function multiplies two quaternions using
% quaternion multiplication. Input quaternions must be 4x1 column arrays
% with the form of [scalar; vector]. The quaternion product assumes the
% convention of DCM multiplication. For example:
%
%               DCM_sa = DCM_sb*DCM_ba  
%                   
% is equivalent to,
%
%               q_sa = quatmult(q_sb,q_ba)
%
% with,
%
% DCM_s_a = DCM transforming vector in frame a to vector in frame s
% DCM_s_b = DCM transforming vector in frame b to vector in frame s
% DCM_b_a = DCM transforming vector in frame a to vector in frame b
% q_s_a = Quaternion transforming vector in frame a to vector in frame s
% q_s_b = Quaternion transforming vector in frame b to vector in frame s
% q_b_a = Quaternion transforming vector in frame a to vector in frame b
% 
% NOTE:  The quaternion product that is output is NOT normalized, and may
% require normalization to maintain the quaternion unit constraint.
%
% SOURCES:
% https://www.sciencedirect.com/topics/computer-science/quaternion-multiplication
%
% INPUT PARAMETERS:
% q1 = 4x1 Input quaternion #1 (unitless)
% q2 = 4x1 Input quaternion #2 (unitless)s
%
% OUTPUT PARAMETERS:
% quat = Output quaternion product (unitless)
%
% VARIABLES:
% qs1 = Scalar component of input quaternion #1 (unitless)
% qv1 = Vector component of input quaternion #1 (unitless)
% qs2 = Scalar component of input quaternion #2 (unitless)
% qv2 = Vector component of input quaternion #2 (unitless)
% quats = Quaternion product scalar (unitless)
% quatv = Quaternion product vector (unitless)
%
% Kail Laughlin
% Updated 11/19/2019
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Code
qs1 = q1(1);            % Scalar component of quaternion 1
qv1 = q1(2:4);          % Vector component of quaternion 1
qs2 = q2(1);            % Scalar component of quaternion 2
qv2 = q2(2:4);          % Vector component of quaternion 2

quats = qs2*qs1 - dot(qv2,qv1);               % Quaternion product scalar
quatv = qs2*qv1 + qs1*qv2 + cross(qv2,qv1);   % Quaternion product vector


try
    quat = [quats;quatv];   % Total quaternion product
catch
    disp('ERROR: One or more quaternion inputs are not in column vector format.')
end
 end
