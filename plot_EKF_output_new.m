%===========================================================%
%                       plot_EKF_output.m                   %
%                                                           %
%   This m-script plots the figures for Case Study 1 shown  %
%   shown.                                                  %
%   rogrammer: Kerry Sun                                    %
%   Created:        March 26, 2017                          %
%   Last Modified:  Feb  05, 2020                           %
%===========================================================%



close all;


%   Plot Figure 7.1:  Ground Track

start_lon = gps_pos_lla(1,2)*ones(drl,1);
start_lat = gps_pos_lla(1,1)*ones(drl,1);

lla_east = (gps_pos_lla(:,2)-start_lon);
lla_north = (gps_pos_lla(:,1)-start_lat);

start_lon_est = pos_ins(1,2)*ones(drl,1);
start_lat_est = pos_ins(1,1)*ones(drl,1);

pos_east = (pos_ins(:,2)-start_lon_est);
pos_north = (pos_ins(:,1)-start_lat_est);


%   Plot Figure 7.2:   Attitude Estimates
figure; % ('rend','painters','pos',[10 150 900 600])
set(gcf,'Name','Trajectory Estimation ');
gps_alt = gps_pos_lla(:,3);
zmin=min(gps_alt);
zmax=max(gps_alt);
map=colormap;
color_steps=size(map,1);

colorbar('Ticks',[0,0.2,0.4,0.6,0.8,1],...
         'TickLabels',{strcat(num2str(zmin+(zmax-zmin)*0/10,3),' m'),...
                       strcat(num2str(zmin+(zmax-zmin)*2/10,3),' m')...
                      ,strcat(num2str(zmin+(zmax-zmin)*4/10,3),' m'),...
                       strcat(num2str(zmin+(zmax-zmin)*6/10,3),' m'),...
                        strcat(num2str(zmin+(zmax-zmin)*8/10,3),' m'),...
                      strcat(num2str(zmin+(zmax-zmin)*10/10,3),' m')}); hold on;

hold on
for i=1:color_steps
    ind=find(gps_alt<zmin+i*(zmax-zmin)/color_steps & gps_alt>=zmin+(i-1)*(zmax-zmin)/color_steps);
    plot((R_0/cos(start_lat(1)))*lla_east(ind),R_0*lla_north(ind),'o','Color',map(i,:));
end
text((R_0/cos(start_lat(1)))*lla_east(1),R_0*lla_north(1),'Start');
text((R_0/cos(start_lat(1)))*lla_east(end)+1,R_0*lla_north(end),'Stop');
xlabel('East/West (m)');ylabel('North/South (m)'); grid on; axis equal; 
garyfyFigure


figure; %('rend','painters','pos',[10 150 900 600]);%(gcf+1)
set(gcf,'Name','NED position Estimation Comparison');
subplot(311)
h1 = plot(t,R_0*lla_north,'b-',t,R_0*pos_north,'r-');hold on;grid on;
xlabel('time (sec)');
ylabel('North/South (m)');
set(h1,'LineWidth',1);

subplot(312)
h1 = plot(t,(R_0/cos(start_lat(1)))*lla_east,'b-',t,(R_0/cos(start_lat(1)))*pos_east,'r-');hold on;grid on;
xlabel('time (sec)');
ylabel('East/West (m)');
set(h1,'LineWidth',1);

subplot(313)
h1 = plot(t,gps_pos_lla(:,3),'b-',t, pos_ins(:,3),'r-');hold on;grid on;
xlabel('time (sec)');
ylabel('altitude(m)');
set(h1,'LineWidth',1);

figure; %('rend','painters','pos',[10 150 900 600]);%(gcf+1)
set(gcf,'Name','NED position Error');
subplot(311)
h1 = plot(t,R_0*lla_north-R_0*pos_north,'r-');hold on;grid on;
xlabel('time (sec)');
ylabel('North/South (m)');
set(h1,'LineWidth',1);

subplot(312)
h1 = plot(t,(R_0/cos(start_lat(1)))*lla_east-(R_0/cos(start_lat(1)))*pos_east,'r-');hold on;grid on;
xlabel('time (sec)');
ylabel('East/West (m)');
set(h1,'LineWidth',1);

subplot(313)
h1 = plot(t,gps_pos_lla(:,3) - pos_ins(:,3),'r-');hold on;grid on;
xlabel('time (sec)');
ylabel('altitude(m)');
set(h1,'LineWidth',1);


figure; %('rend','painters','pos',[10 150 900 600]);%(gcf+1)
set(gcf,'Name','NED Velocity Estimation Comparison');
ax1 = subplot(311);
plot(t,gps_vel_ned(:,1),'k-',t,vel_ins(:,1),'r--','linewidth',2);
%legend('GPS measured','Estimate');

%xlabel('time (min)');
ylabel('V_N (m/s)');
grid on;
    
ax2 = subplot(312);
plot(t,gps_vel_ned(:,2),'k-',t,vel_ins(:,2),'r--','linewidth',2);
%legend('GPS measured','Estimate');

%xlabel('time (min)');
ylabel('V_E (m/s)');
grid on;

ax3 = subplot(313);
plot(t,gps_vel_ned(:,3),'k-',t,vel_ins(:,3),'r--','linewidth',2);
legend('True','Estimate');
ylim([-20 20]); 
xlabel('time (sec)');
ylabel('V_D (m/s)');
grid on;
linkaxes([ax1,ax2,ax3],'x')

figure; %('rend','painters','pos',[10 150 900 600]);%(gcf+1)
set(gcf,'Name','NED Velocity Error');
ax1 = subplot(311);
plot(t,gps_vel_ned(:,1)-vel_ins(:,1),'r--','linewidth',2);
%legend('GPS measured','Estimate');

%xlabel('time (min)');
ylabel('V_N (m/s)');
grid on;
    
ax2 = subplot(312);
plot(t,gps_vel_ned(:,2)-vel_ins(:,2),'r--','linewidth',2);
%legend('GPS measured','Estimate');

%xlabel('time (min)');
ylabel('V_E (m/s)');
grid on;

ax3 = subplot(313);
plot(t,gps_vel_ned(:,3)-vel_ins(:,3),'r--','linewidth',2);
legend('Error');
ylim([-20 20]); 
xlabel('time (sec)');
ylabel('V_D (m/s)');
grid on;
linkaxes([ax1,ax2,ax3],'x')

yaw_est_d = wrapTo180(eul_ins(:,3)*r2d);

figure;%('rend','painters','pos',[10 150 900 600]);
set(gcf,'Name','atitude Estimation Comparison');
subplot(311)
plot(t,roll,'k-',t,eul_ins(:,1)*r2d,'r--','linewidth',2);
%xlabel('time (min)');
ylabel('\phi (deg)');
grid on;
subplot(312)
plot(t,pitch,'k-',t,eul_ins(:,2)*r2d,'r--','linewidth',2);
%xlabel('time (min)');
ylabel('\theta (deg)');
grid on;
subplot(313)
plot(t,wrapTo180(yaw),'k-',t,wrapTo180(eul_ins(:,3)*r2d),'r--','linewidth',2);
legend('True','Estimate');
% title('Euler angle');
xlabel('time (sec)');
ylabel('\psi (deg)');
grid on;








figure; % ('rend','painters','pos',[10 150 900 600]);
set(gcf,'Name','atittude Estimation 3-\simga bound');
%temp = wrapTo180(eul_ins(:,3)*r2d)-wrapTo180(yaw);
temp = wrapTo180(eul_ins(:,3)*r2d-yaw);
%temp(find(abs(temp) > 20)) = NaN; 
yaw_err = temp; clear temp; 
subplot(311)
hold on;
plot(t,eul_ins(:,1)*r2d-roll,'r--','LineWidth',2);
h3 = plot(t,3*sqrt(Ppsi(:,1))*r2d,'k-','LineWidth',2);
h4 = plot(t,-3*sqrt(Ppsi(:,1))*r2d,'k-','LineWidth',2);
h3.Color(4) = 0.3;
h4.Color(4) = 0.3;
legend('\Delta \phi (deg)','3-\sigma bound'); grid on;  
ylim([-10 10]);
subplot(312)
hold on;
plot(t,eul_ins(:,2)*r2d-pitch,'r--','LineWidth',2);
h3 = plot(t,3*sqrt(Ppsi(:,2))*r2d,'k-','LineWidth',2);
h4 = plot(t,-3*sqrt(Ppsi(:,2))*r2d,'k-','LineWidth',2);
h3.Color(4) = 0.3;
h4.Color(4) = 0.3;
legend('\Delta \theta (deg)','3-\sigma bound'); grid on;  
ylim([-10 10]);
subplot(313)
hold on;
plot(t,yaw_err,'r--','LineWidth',2);
h3 = plot(t,3*sqrt(Ppsi(:,3))*r2d,'k-','LineWidth',2);
h4 = plot(t,-3*sqrt(Ppsi(:,3))*r2d,'k-','LineWidth',2);
h3.Color(4) = 0.3;
h4.Color(4) = 0.3;
legend('\Delta \psi (deg)','3-\sigma bound'); grid on; 
ylim([-10 10]);xlabel('time (sec)');
%suptitle('Attitude Estimate error and its 3\sigma Bounds (deg)')

%   %  Plot Figure 7.3:   Sensor Bias Estimates

% Nav_accelBias = data.NavFilter.AccelBias_mss(:,timesection)';
% Nav_gyroBias  = data.NavFilter.GyroBias_rads(:,timesection)';
figure('rend','painters');
set(gcf,'Name','Accel and gyro bias Estimation');
subplot(321)
h1 = plot(t,accelBias(:,1)*(1000/g),'r',t,accel_bias_true(:,1)*(1000/g),'b');hold on;grid on;%estimate
title('Accelerometer Bias Estimates');
ylabel('b_{a_1}(milli-g)');
%axis([t(1)/60 14 -300 300]);
set(h1,'LineWidth',2);

subplot(323)
h2 = plot(t,accelBias(:,2)*(1000/g),'r',t,accel_bias_true(:,2)*(1000/g),'b');hold on;grid on;
ylabel('b_{a_2} (milli-g)');
%axis([t(1)/60 14 -300 300]);
set(h2,'LineWidth',2);

subplot(325)
h3 = plot(t,accelBias(:,3)*(1000/g),'r',t,accel_bias_true(:,3)*(1000/g),'b');hold on;grid on;
ylabel('b_{a_3} Bias(milli-g)');
xlabel('Time (sec)');
%axis([t(1)/60 14 -300 300]);
set(h3,'LineWidth',2);

subplot(322)
h4 = plot(t,gyroBias(:,1)*r2d,'r',t,gyro_bias_true(:,1)*r2d,'b');hold on;grid on;
title('Gyro Bias Estimates');
ylabel('b_{g_1} (deg/s)');
set(h4,'LineWidth',2);
%axis([t(1)/60 14 -10 10]);
subplot(324)
h5 = plot(t,gyroBias(:,2)*r2d,'r',t,gyro_bias_true(:,2)*r2d,'b');hold on;grid on;
ylabel('b_{g_2} (deg/s)');
set(h5,'LineWidth',2);
%axis([t(1)/60 14 -10 10]);

subplot(326)
h6 = plot(t,gyroBias(:,3)*r2d,'r',t,gyro_bias_true(:,3)*r2d,'b');hold on;grid on;
ylabel('b_{g_3} (deg/s)');
xlabel('Time (sec)');
set(h6,'LineWidth',2);

