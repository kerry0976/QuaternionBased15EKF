%=======================================================%
%               gnss_ins_data_loader.m                  %
%                   (MATLAB Version)                    %
%                                                       %
%   This m-file loads in data that will be played back  %
%   in the GNSS/INS EKF.  The data was collected from   %
%   flight test onboard a small hand launched aerial    %
%   vehicle and is contained in the file named          %
%   flight_test_data_set.mat.                           %
%                                                       %
%   Original Programmer:     Demoz Gebre-Egziabher      %
%   new Programmer:     Kerry Sun
%   Created:        March 26, 2009                      %
%   Last Modified:  Jan 22, 2020                       %
%   
%   Comment: remove "clear idx" 
%=======================================================%

if(ispc)                                    %   Path to gnss_ins toolbox
    addpath  'gnss_ins_functions\';
    addpath  'flight_data\';
    % flight data
    load('MjolnirFLT10.mat') % used in AIAA journal paper 1
    %rmpath(flightdata_path);
    
else
    addpath 'gnss_ins_functions/';
    addpath  'flight_data/';
    load('MjolnirFLT10.mat') % used in AIAA journal paper 1
end



%~~~~~~~~~~~~~~~ Real data Extraction  for <MjolnirFLT> ~~~~~~~~~~~~~~~~~~%
r2d = 180/pi;
d2r = pi/180; 
uT2G = 1000;               
%t_raw_begin = round(double(data.Fmu.Time_us(1))*10^-6,2);       
t = round(double(data.Fmu.Time_us(:))*10^-6,2); % - t_raw_begin;


%   Find subset of data that falls between t_start and t_end

idx = find(t >= t_start*60);
k_s = idx(1);
idx = find(t >= t_end*60);
k_e = idx(1);
idx = [k_s:k_e];
timesection = idx; 
drl = length(idx);


gyro_bias_true = data.NavFilter.GyroBias_rads(:,idx)';
accel_bias_true = data.NavFilter.AccelBias_mss(:,idx)';


p = data.Mpu9250.Gyro_rads(1,:)';
q = data.Mpu9250.Gyro_rads(2,:)';
r = data.Mpu9250.Gyro_rads(3,:)';
ax = data.Mpu9250.Accel_mss(1,:)';
ay = data.Mpu9250.Accel_mss(2,:)';
az = data.Mpu9250.Accel_mss(3,:)';
imu = [t p q r ax ay az]; 
hx = data.Mpu9250.Mag_uT(1,:)';
hy = data.Mpu9250.Mag_uT(2,:)';
hz = data.Mpu9250.Mag_uT(3,:)';
gps_pos_lla = [data.Gps_0.LLA(1,:)' data.Gps_0.LLA(2,:)'...
              data.Gps_0.LLA(3,:)'];% rad, rad, m
gps_vel_ned  = data.Gps_0.NEDVelocity_ms(:,:)'; % m/s 
% gps_vel_ned = gps_vel_ned;
% gps_pos_lla = [data.NavFilter.LLA(1,:)' data.NavFilter.LLA(2,:)'...
%                data.NavFilter.LLA(3,:)'];
% gps_vel_ned  = data.NavFilter.NEDVelocity_ms(:,:)'; % m/s 

roll = data.NavFilter.Euler_rad(1,:)'*r2d;
pitch = data.NavFilter.Euler_rad(2,:)'*r2d;
yaw = data.NavFilter.Euler_rad(3,:)'*r2d;
%airspeed = data.Airdata.vIasSmooth_m(1,:)'; %m/s for FLT03
airspeed = data.Airdata.vIasFilt_mps(1,:)'; %m/s



%~~~~~~~~~~~~~~~~~~~~~~~~ 5-hole air data calc  ~~~~~~~~~~~~~~~~~~%
if (five_hole_on == 1)
    %Method one using calibrated wind tunnel coefficient
    psi2pa = 6894.76;
    pa2psi = 1/psi2pa;
    K2a_avg = 0.079;
    K2b_avg = 0.079;
    %  x5 = [ -0.0095    0.7490   -0.3018   -5.9453   -0.1355]; % from  AIAA journal 1 
    x5 = [-0.1748 4.3553 0.2982 -2.4854 -0.2673 -1.2980 -0.1380 -3.9038 -2.4137 -0.7168]; % from FLT 06, 01/22/2020, Journal results 
    presAlpha_Pa = data.presAlpha_Pa.Pressure_Pa';  % pa1-pa2
    presBeta_Pa = data.presBeta_Pa.Pressure_Pa';    % pb2-pb1
    presStatic_Pa = data.presStatic_Pa.Pressure_Pa';% p_s
    presTip_Pa = data.presTip_Pa.Pressure_Pa';      % p3 - p_s
    
    %posRud_deg = data.posRud_rad.CalValue(timesection)*r2d';
    qcl2 = presTip_Pa;
    
    %pos_5hole = [0 0 0]*0.0254; % position of 5 hole probe tip to c.g.
    pos_5hole = [12 20 0.5]*0.0254; % position of 5 hole probe tip to c.g.
    pa1_pa2_d_qcl =  presAlpha_Pa./qcl2;
    pb1_pb2_d_qcl =  presBeta_Pa./qcl2;
%     alpha_5hole = pa1_pa2_d_qcl/K2a_avg*(1+x2(1)) + x2(2); % from aiaa journal 1
%     beta_5hole =  pb1_pb2_d_qcl/K2b_avg*(1+x3(3)) + x3(4); 
    alpha = pa1_pa2_d_qcl/K2a_avg*(1+x5(3)) + x5(4); % from aiaa journal 2
    beta =  pb1_pb2_d_qcl/K2b_avg*(1+x5(5)) + x5(6); 
end

%{
%~~~~~~~~~~~~~~~~~~~~~~~~ Real data Extraction  for 25e ~~~~~~~~~~~~~~~~~~%
r2d = 180/pi;
d2r = pi/180; 
imu = [flight_data.time flight_data.p flight_data.q flight_data.r...
       flight_data.ax flight_data.ay flight_data.az ]; 
gps_pos_lla = [flight_data.navlat flight_data.navlon flight_data.navalt];% rad, rad, m
gps_vel_ned  = [flight_data.navvn flight_data.navve flight_data.navvd]; % m/s 
t = flight_data.time; 
roll = flight_data.phi*r2d;
pitch = flight_data.theta*r2d;
yaw = flight_data.psi*r2d; 


%% Ultrastick 25e
x = 1.0033; %m; 39.5 inch
y = 0.5207; %m; 20.5 inch
z = 0.0762; %m; 3 inch

Lalpha_m = flight_data.l_alpha*r2d;
%alpha_m2  = flight_data.aoa(indxselect)*r2d;
Lbeta_m = flight_data.l_beta*r2d;
%beta_m2 = flight_data.aos(indxselect)*r2d;
p = flight_data.p;
q = flight_data.q;
r = flight_data.r;
airspeed = flight_data.ias; %m/s
Lalpha_cg = Lalpha_m +q*(-x)./airspeed - p*y./airspeed;
%Ralpha_cg = Ralpha_m +q*x./airspeed - p*y./airspeed;
Lbeta_cg = Lbeta_m - r*(-x)./airspeed + p*z./airspeed;
%Rbeta_cg = Rbeta_m - r*x./airspeed + p*z./airspeed;
%}



%~~~~~~~~~~~~~~~~~ reak flight data Extraction for 120e ~~~~~~~~~~~~~~~~~~%
% imu = [dtc.time dtc.imu.p dtc.imu.q dtc.imu.r...
%        dtc.imu.ax dtc.imu.ay dtc.imu.az ]; 
% gps_pos_lla = [gps_lat gps_lon gps_alt]*pi/180;
% gps_vel_ned  = [gps_Vn gps_Ve gps_Vd]; % m/s correct order, Xdot is V_East
% t = dtc.time; 
% roll = dtc.ekf.phi;
% pitch = dtc.ekf.theta;
% yaw = dtc.ekf.psi*pi/180; 
% 
% yaw (yaw  > pi) = pi;
% yaw(yaw < -pi) = -pi;
% indx_180 = find(yaw == pi);
% indx_all = [indx_180+1  indx_180+2 indx_180+3 indx_180+4 indx_180+5 indx_180+6];
% indx_all(:);
% yaw(indx_all) = pi; 
% 
% yaw = yaw*180/pi;

% Lalpha_m = dtc.air.bird.Lalpha;
% Ralpha_m = dtc.air.bird.Ralpha;
% Lbeta_m = dtc.air.bird.Lbeta;
% Rbeta_m = dtc.air.bird.Rbeta;
%% Ultrastick 
% x = 1.0033; %m; 39.5 inch
% y = 0.5207; %m; 20.5 inch
% z = 0.0762; %m; 3 inch
% p = dtc.imu.p*r2d;
% q = dtc.imu.q*r2d;
% r = dtc.imu.r*r2d;
% airspeed = dtc.air.pitot.airspeed;
% Lalpha_cg = Lalpha_m +q*(-x)./airspeed - p*y./airspeed;
% Ralpha_cg = Ralpha_m +q*x./airspeed - p*y./airspeed;
% Lbeta_cg = Lbeta_m - r*(-x)./airspeed + p*z./airspeed;
% Rbeta_cg = Rbeta_m - r*x./airspeed + p*z./airspeed;
%~~~~~~~~~~~~~~~~~~~~~~~~  end ~~~~~~~~~~~~~~~~~~~~~~~ ~~~~~~~~~~~~~~~~~~%

%   Establish initial value for inertial sensor biases



initial_gyro_bias = -mean(imu(idx:idx+1000,2:4));
initial_accel_bias = -(mean(imu(idx:idx+1000,5:7)) + [0 0 g]);

%   Extract IMU, GPS and attitude data that falls between t_start and t_end

p_temp = p(idx,:);
q_temp = q(idx,:);
r_temp = r(idx,:);
ax_temp = ax(idx,:);
ay_temp = ay(idx,:);
az_temp = az(idx,:);

t_temp = t(idx); % - ones(drl,1)*t(1);
imu_temp = imu(idx,:);
hx_temp = hx(idx,:)*uT2G;
hy_temp = hy(idx,:)*uT2G;
hz_temp = hz(idx,:)*uT2G;
gps_pos_lla_temp = gps_pos_lla(idx,:);
gps_vel_ned_temp = gps_vel_ned(idx,:);
roll_temp = roll(idx);
pitch_temp = pitch(idx);
yaw_temp = yaw(idx);
% wind_sim_temp = flight_data.SteadyWind(idx,:);
Va_temp = airspeed(idx);
gps_alt_temp = data.Airdata.altFilt_m(idx)';

if (five_hole_on == 1)
    alpha_tip_temp = alpha(idx);
    beta_tip_temp =  beta(idx);
    clear t imu gps_pos_lla gps_vel_ned roll pitch yaw k_s k_e ...
        p q r ax ay az airspeed alpha beta;
else
    clear t imu gps_pos_lla gps_vel_ned roll pitch yaw k_s k_e ...
        p q r ax ay az airspeed;
end

%   Clear extraneous data


%   Repackage data into original name variables

t = t_temp;
imu = imu_temp;
gps_pos_lla = gps_pos_lla_temp;
gps_vel_ned = gps_vel_ned_temp;
roll = roll_temp;
pitch = pitch_temp;
yaw = yaw_temp;
Va = Va_temp; 
gps_alt = gps_alt_temp; 
p = p_temp;
q = q_temp;
r = r_temp;
ax = ax_temp;
ay = ay_temp;
az = az_temp;
hx = hx_temp;
hy = hy_temp;
hz = hz_temp;

if (five_hole_on == 1)
    alpha_tip = alpha_tip_temp;
    beta_tip = beta_tip_temp;
    clear t_temp imu_temp gps_pos_lla_temp gps_vel_ned_temp roll_temp...
        pitch_temp yaw_temp Va_temp flight_data alpha_temp beta_temp...
        gps_alt_temp alpha_tip_temp beta_tip_temp...
        p_temp q_temp r_temp ax_temp ay_temp az_temp hx_temp hy_temp hz_temp;
else
    clear t_temp imu_temp gps_pos_lla_temp gps_vel_ned_temp roll_temp...
        pitch_temp yaw_temp Va_temp flight_data alpha_temp beta_temp...
        gps_alt_temp...
        p_temp q_temp r_temp ax_temp ay_temp az_temp hx_temp hy_temp hz_temp;
end


