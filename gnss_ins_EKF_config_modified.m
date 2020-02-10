%===========================================================%
%               gnss_ins_filter_config.m                    %
%                                                           %
%   This m-file contains all the conifiguraiton switches    %
%   for the GNSS/INS filter.                                %
%                                                           %
%   Programmer:     Kerry Sun                               %
%   Created:        05/29/2018                              %
%   Last Modified:  02/10/2020                              %
%                                                           %
%===========================================================%

%   Define start and stop time in MINUTES for playback. 
if (real_flight_data_input ~= 0)
% simulated Time
% FLT03
% t_start = 16.5;
% t_end = 28.5;

% FLT04
% t_start = 7;
% t_end = 21;
% 
% % FLT05
% t_start = 18;
% t_end = 32;

% for Mjolnir06
% t_start = 7;
% t_end = 14;

% for Mjolnir07
% t_start = 6.5;
% t_end = 23.5;

% for Mjolnir08
% t_start = 7;
% t_end = 15;

% for Mjolnir10 ( use this for flight data testing 07/10/2018 ) 
t_start = 16;
t_end = 25;

% for Mjolnir11
% t_start = 4;
% t_end = 15;

% for IBIS
% t_start = 3.5;
% t_end = 10;

else 
% simulated Time for simulation flight data 
% t_start = 5;
% t_end = 12.5;

t_start = 0;
t_end = 5;
end 
%   Configure the Extended Kalman Filter (EKF)

CLOSED_LOOP = 1;        %   If set to 1, a GNSS-aided 
                        %   inertial navigator is simulated.
                        %   If set to 0, the simulation will
                        %   be that of an unaided INS.  
                        
NO_CORIOLIS = 1;        %   If set to 1, coriolis acceleraitons
                        %   are ignored in the time upadate 
                        %   equations. Should be set to 0 when
                        %   using high grade inertial sensors and
                        %   GNSS updates come at a slow rate.
                        
SMALL_PROP_TIME = 1;    %   If set to 1, it means that the time
                        %   between GNSS updates is small and,
                        %   thus, Schuler dynamics can be
                        %   ignored without much consequence.
                        %   That is, the approximation given by
                        %   Equation (6.17) is used for the 
                        %   velocity propagation instead of 
                        %   Equation (6.13)

                                
gnss_update_rate = 1;   %  GNSS measurement update rate in Hz.

if (real_flight_data_input == 0)
    
    
    % GPS measurement noise standard deviation
    
    % gps_pos_sigma = 3;              %   m (North, East, Down)
    % gps_vel_sigma = 0.2;            %   m/s (North, East, Donw)
    
    % IMU output error covariance (Table 1, Chapter 6)
    
    % sigma_g_d = 0.3*d2r;                % Standard deviation of correlated gyro bias
    % tau_g = 300;                        % Correlation time or time constant of b_{gd}
    % sigma_g_w = 0.95*d2r;                % Standard deviation of gyro output noise
    %
    %
    % sigma_a_d = (0.5e-3)*g;             % Standard deviation of Accelerometer Markov Bias
    % tau_a = 300;                        % Correlation time or time constant of b_{ad}
    % sigma_a_w = 0.12*g;                 % Standard Deviation of Accelerometer Wide Band Noise
    
    % For simulink simulator
    g = 9.81;
    s2hr = 1/3600;              % Seconds to hours
    
    gps_pos_ne_sigma = 3;             %   m (North, East)
    gps_pos_d_sigma = 3;              %   m (Down)
    gps_vel_ne_sigma = 0.2;           %   m/s (North, East)
    gps_vel_d_sigma = 0.2;            %   m/s (Down)
    
%     gps_pos_ne_sigma = 3;             %   m (North, East)
%     gps_pos_d_sigma = 3;              %   m (Down)
%     gps_vel_ne_sigma = 0.5;           %   m/s (North, East)
%     gps_vel_d_sigma = 1;            %   m/s (Down)
%     

%     sigma_g_d = 0.3*d2r;
%     tau_g = 300;
%     sigma_g_w = 0.1*d2r;
%     sigma_a_d = (0.5e-2)*g;
%     tau_a = 300;
%     sigma_a_w = (1.0e-3);  

    % as 07/21/2018 
    %sigma_g_d = 0.3*d2r;    % [0.005 0.01 0.1 0.5 1 5 10 180 360 1080
    %2*1080]*d2r*s2hr; 10 is good, small 
    sigma_g_d = 1080*d2r*s2hr;
    tau_g = 300;
    sigma_g_w = 0.00175; % 0.1 deg/s 
    % sigma_a_d = (0.5e-2)*g; %  [5e-6 10e-6 20e-6  30e-6  50e-6  1e-3 2e-3 3e-3 4e-3 5e-3]*g
    sigma_a_d = (0.5e-2)*g;
    tau_a = 300;
    sigma_a_w = 0.05;   % m/s^2 0.05
    
    
%     gps_pos_ne_sigma = 10;             %   m (North, East)
%     gps_pos_d_sigma = 10;              %   m (Down)
%     gps_vel_ne_sigma = 1;           %   m/s (North, East)
%     gps_vel_d_sigma =  1;            %   m/s (Down)
%     sigma_g_d = 0.00025;
%     tau_g = 50;
%     sigma_g_w = 0.00175;
%     sigma_a_d = 0.01; 
%     tau_a = 100;
%     sigma_a_w = 0.05;
    
    % ~~~~~~~~~~~~~~~~~~~~~~~~  Wind parameter      ~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    %for simulation
    sigma_w_ne_d = 0.5;         % standard deviation of wind component for real flight data
    tau_w_ne  = 1;              % sec  correlation time  % 1
    
    sigma_w_d_d =  2; 1.5;

    tau_w_d = 10;
    
else
    % updated with MjolnirFLT03 03/03/2018
    % https://github.com/bolderflight/SOC/blob/newFuncs/soc-src/EKF_15state.cxx
    INIT.P_P_INIT = 10.0;
    INIT.P_V_INIT = 1.0;
    INIT.P_A_INIT = 0.34906;   % 20 deg
    INIT.P_HDG_INIT = 3.14159; % 180 deg
    INIT.P_AB_INIT = 0.9810;   % 0.5*g
    INIT.P_GB_INIT = 0.01745;  % 5 deg/s
    
    config.sig_w_ax = 0.05;     % Std dev of Accelerometer Wide Band Noise (m/s^2)
    config.sig_w_ay = 0.05;
    config.sig_w_az = 0.05;
    config.sig_w_gx = 0.00175;  % Std dev of gyro output noise (rad/s)  (0.1 deg/s)
    config.sig_w_gy = 0.00175;
    config.sig_w_gz = 0.00175;
    config.sig_a_d  = 0.01;     % Std dev of Accelerometer Markov Bias
    config.tau_a    = 100.0;    % Correlation time or time constant of b_{ad}
    config.sig_g_d  = 0.00025;  % Std dev of correlated gyro bias (rad)
    config.tau_g    = 50.0;     % Correlation time or time constant of b_{gd}
    config.sig_gps_p_ne = 3.0;  % GPS measurement noise std dev (m)
    config.sig_gps_p_d  = 6.0;  % GPS measurement noise std dev (m)
    config.sig_gps_v_ne = 0.5;  % GPS measurement noise std dev (m/s)
    config.sig_gps_v_d  = 1.0;  % GPS measurement noise std dev (m/s)
    config.sig_mag      = 0.3;  % Magnetometer measurement noise std dev (normalized -1 to 1)
    
    gps_pos_ne_sigma = config.sig_gps_p_ne;  %   m (North, East)
    gps_pos_d_sigma = config.sig_gps_p_d;    %   m (Down)
    gps_vel_ne_sigma = config.sig_gps_v_ne;  %   m/s (North, East)
    gps_vel_d_sigma = config.sig_gps_v_d;    %   m/s (Dowm)
    
    sigma_a_w = config.sig_w_ax;
    sigma_g_w = config.sig_w_gx;
    sigma_a_d = config.sig_a_d;
    tau_a = config.tau_a;
    sigma_g_d = config.sig_g_d;
    tau_g = config.tau_g;
    
    
    % IGNORE the following... 
    % ~~~~~~~~~~~~~~~~~~~~~~~~  Wind parameter as 07/10/2018      ~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    sigma_w_ne_d = 0.05; 0.05;0.15;         % standard deviation of wind component for real flight data
    tau_w_ne  = 1;              % sec  correlation time  % 1
    % sigma_w_d_d = 1.2;2.5;2;
    % tau_w_d = 10; 100;
    %    sigma_w_d_d =  0.1; % for a calm day
    sigma_w_d_d = 1.5;1;2;
    tau_w_d = 100;
    %sigma_w_d = 0.05;       % standard deviation of wind component
    %sigma_w_d_d = 0.5;
    
    %    sigma_w_d_d =  0.1; % for a calm day
    %     sigma_w_d_d = 1.5;1;2;
    %     tau_w_d = 100;
    
    
    %
    % ~~~~~~~~~~~~~~~~~~~~~~~~  Wind parameter as 01/30/20120      ~~~~~~~~~~~~~~~~~~~~~~~~~~~% 
    sigma_w_ne_d = 3;         % standard deviation of wind component for real flight data
    tau_w_ne  = 5;              % sec  correlation time  % 1
    
    sigma_w_d_d = 2; % 1.5;

    tau_w_d = 1000;
end
                    



 