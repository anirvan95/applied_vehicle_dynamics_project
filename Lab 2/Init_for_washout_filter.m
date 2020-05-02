%----------------------------------------------------------------
% Template created for the course SD2231 by Mikael Nybacka 2019
% Following file is the start file for the Simulink implementation 
% of the integration, model-based, and washout filter.
%----------------------------------------------------------------

clear all
close all;
% clc;
addpath('scripts')
addpath('logged_data')
disp(' ');

% Set global variables so that they can be accessed from other matlab
% functions and files
global vbox_file_name

%----------------------------
% LOAD DATA FROM VBOX SYSTEM
%----------------------------
%Test for SD2231 20180912

%Cloudy 18 degrees celsius
%2.8 bar in all four tyres
%Two persons in the front

for man = 1:4

manouver=man

%vbox_file_name='S90__035.VBO';   %Standstill
switch manouver
    case 1
        vbox_file_name='S90__036.VBO';   %Circular driving to the left, radius=8m
    case 2
        vbox_file_name='S90__038.VBO';  %Slalom, v=30km/h
    case 3
        vbox_file_name='S90__040.VBO';  %Step steer to the left, v=100km/h
    case 4
        vbox_file_name='S90__041.VBO';  %Frequency sweep, v=50km/h
end


vboload;
%  Channel 1  = satellites
%  Channel 2  = time
%  Channel 3  = latitude
%  Channel 4  = longitude
%  Channel 5  = velocity kmh
%  Channel 6  = heading
%  Channel 7  = height
%  Channel 8  = vertical velocity kmh
%  Channel 9  = long accel g
%  Channel 10 = lat accel g
%  Channel 11 = glonass_sats
%  Channel 12 = gps_sats
%  Channel 13 = imu kalman filter status
%  Channel 14 = solution type
%  Channel 15 = velocity quality
%  Channel 16 = event 1 time
%  Channel 17 = latitude_raw
%  Channel 18 = longitude_raw
%  Channel 19 = speed_raw
%  Channel 20 = heading_raw
%  Channel 21 = height_raw
%  Channel 22 = vertical_velocity_raw
%  Channel 23 = rms_hpos
%  Channel 24 = rms_vpos
%  Channel 25 = rms_hvel
%  Channel 26 = rms_vvel
%  Channel 27 = poscov_xx
%  Channel 28 = poscov_yy
%  Channel 29 = poscov_zz
%  Channel 30 = velcov_xx
%  Channel 31 = velcov_yy
%  Channel 32 = velcov_zz
%  Channel 33 = t1
%  Channel 34 = accrpedlln 
%  Channel 35 = engn
%  Channel 36 = ptgearact
%  Channel 37 = pttqatw_fl 
%  Channel 38 = pttqatw_fr 
%  Channel 39 = swa
%  Channel 40 = brkpedlpsd 
%  Channel 41 = vehspdlgt
%  Channel 42 = flwhlspd
%  Channel 43 = frwhlspd
%  Channel 44 = rlwhlspd
%  Channel 45 = rrwhlspd
%  Channel 46 = algt1
%  Channel 47 = alat1
%  Channel 48 = rollrate1
%  Channel 49 = yawrate1
%  Channel 50 = true_head
%  Channel 51 = slip_angle 
%  Channel 52 = lat._vel.
%  Channel 53 = roll_angle 
%  Channel 54 = lng._vel.
%  Channel 55 = slip_cog
%  Channel 56 = yawrate
%  Channel 57 = x_accel
%  Channel 58 = y_accel
%  Channel 59 = temp
%  Channel 60 = pitchrate
%  Channel 61 = rollrate
%  Channel 62 = z_accel
%  Channel 63 = roll_imu
%  Channel 64 = pitch_ang. 
%  Channel 65 = yaw_rate
%  Channel 66 = slip_fl
%  Channel 67 = slip_fr
%  Channel 68 = slip_rl
%  Channel 69 = slip_rr
%  Channel 70 = true_head2 
%  Channel 71 = head_imu
%  Channel 72 = pitch_imu
%  Channel 73 = pos.qual.
%  Channel 74 = lng_jerk
%  Channel 75 = lat_jerk
%  Channel 76 = head_imu2

%-----------------------------------
% SET VEHICLE DATA FOR THE VOLVO S90
%-----------------------------------


Rt=0.35;            % Tyre radius (m)
L=2.941;            % Wheel base (m)
lf=1.65;            % Distance from CoG to front axis (m)
lr=L-lf;            % Distance from CoG to rear axis (m)
mass=2010.5;        % Mass (kg)
Iz=3089;            % Yaw inertia (kg-m2)
tw=1.617;           % Track width (m)
h_cog = 0.570;      % Height of CoG above ground
Ratio=16.3;         % Steering gear ratio

% Cf = 100000;
% Cr = 100000;

Cf=160000;          % Lateral stiffness front axle (N)
Cr = 225000;          % Lateral stiffness rear axle (N)
Lx_relax=0.05;      % Longitudinal relaxation lenth of tyre (m)
Ly_relax=0.15;      % Lateral relaxation lenth of tyre (m)
Roll_res=0.01;      % Rolling resistance of tyre
rollGrad=deg2rad(4.5);       % rollangle deg per latacc
rx=0.29;            % distance from CoG to IMU x-axle
ry=0;               % distance from CoG to IMU y-axle
rz=0;               % distance from CoG to IMU z-axle

Ts = 0.01;

%--------------------------------------
% SET ENVIRONEMNTAL PARAMETERS FOR TEST
%--------------------------------------
Mu=0.85;            % Coefficient of friction
g=9.81;             % Gravity constant

%--------------------------------------------
% SET VARIABLES DATA FROM DATA READ FROM FILE
%--------------------------------------------

Time            = vbo.channels(1, 2).data-vbo.channels(1, 2).data(1,1);
yawRate_VBOX    = vbo.channels(1, 56).data.*(-pi/180); %VBOX z-axis is pointing downwards, hence (-)
vx_VBOX         = vbo.channels(1, 54).data./3.6;
vy_VBOX         = vbo.channels(1, 52).data./3.6;
SteerAngle      = vbo.channels(1, 39).data./Ratio;
ax_VBOX         = vbo.channels(1, 57).data.*g;
ay_VBOX         = vbo.channels(1, 58).data.*g;
Beta_VBOX       = (vy_VBOX + rx*yawRate_VBOX)./vx_VBOX;

simTime = Time(end-1);
% subplot(2,2,man)
% plot(yawRate_VBOX)

%% Steady state check 
vy_VBOX = vy_VBOX + rx*yawRate_VBOX;
vy_dot = diff(vy_VBOX)./diff(Time);
yawRate_dot_VBOX = diff(yawRate_VBOX)./diff(Time);

% yyaxis('right')
% plot(yawRate_dot_VBOX)
ay_COG = ay_VBOX(1:end-1) + rx.*yawRate_dot_VBOX - ry.*yawRate_dot_VBOX;
yawRate_dot_VBOX = smooth(yawRate_dot_VBOX,0.01,'rlowess');

% plot(vy_VBOX(1:end-1))
% hold on

    vy_model_denom = ((lf+lr)*(lf+lr)*Cf*Cr + mass*vx_VBOX.*vx_VBOX*(lr*Cr - lf*Cf));
    vy_model_denom(vy_model_denom <= eps) = eps;
    vy_model_num = vx_VBOX.*((lr*(lf+lr)*Cf*Cr - lf*Cf*mass*vx_VBOX.*vx_VBOX));
    vx = vx_VBOX;
    vx(vx_VBOX < 0.001) = 0.001;
    vy_model = vy_model_num.*SteerAngle./vy_model_denom;
    beta_model = vy_model./vx;
    
    Beta_VBOX_smooth=smooth(Beta_VBOX(1:end-1),0.01,'rlowess'); 
    [e_beta_mean,e_beta_max,time_at_max,error] = errorCalc(beta_model(1:end-1),Beta_VBOX_smooth);
    disp(' ');
    fprintf('Model based calculation\n')
    fprintf('The MSE of Beta estimation is: %d \n',e_beta_mean);
    fprintf('The Max error of Beta estimation is: %d \n',e_beta_max);

%% Integration based Slip 
standstill_time = abs(vy_VBOX(1:end-1))<eps;
mean_ay_COG = mean(ay_COG(standstill_time));
if isnan(mean_ay_COG)
    cfilt_ay_COG = lowpass(ay_COG, 49, 100);
else
    cfilt_ay_COG = lowpass(ay_COG, 49, 100) - mean_ay_COG;
end

% window_size = 100;
% window_mean = [];
% label = zeros(length(cfilt_ay_COG),1);
% for i=window_size:(length(cfilt_ay_COG)-window_size)
%     window_data = abs(cfilt_ay_COG(i-window_size/2:i+window_size/2));
%     window_mean = [window_mean;mean(window_data)];
%     label(i) = mean(window_data)>0.6;
% end
%cfilt_ay_COG = ay_COG;

label = abs(cfilt_ay_COG)>=0.15;

value = cfilt_ay_COG.*(1-rollGrad) - yawRate_VBOX(1:end-1).*vx(1:end-1);
sum = 0;

vy_inter = zeros(length(Time)-1,1);
for iter = 1:length(Time)-1
    sum = sum + value(iter)*Ts;
    vy_inter(iter) = sum;
end
hp_vy_inter = highpass(vy_inter, 0.04, 100);
beta_inter = (hp_vy_inter./vx(1:end-1));
% beta_inter(label<1)=0;
% figure,
% plot(Beta_VBOX,'b')
% hold on
% plot(beta_inter,'r')
[e_beta_mean,e_beta_max,time_at_max,error] = errorCalc(beta_inter,Beta_VBOX_smooth);
disp(' ');
fprintf('Integration based calculation\n')
fprintf('The MSE of Beta estimation is: %d \n',e_beta_mean);
fprintf('The Max error of Beta estimation is: %d \n',e_beta_max);

% 
% 
%% Washout
T = 0.5;
s = tf('s');
cont_sys_lp = 1/(1+s*T(m));
cont_sys_hp = s*T/(1+s*T(m));
dis_sys_lp = c2d(cont_sys_lp, 0.01);
dis_sys_hp = c2d(cont_sys_hp, 0.01);
a_lp = dis_sys_lp.Numerator{1};
b_lp = dis_sys_lp.Denominator{1};
a_hp = dis_sys_hp.Numerator{1};
b_hp = dis_sys_hp.Denominator{1};
% prefiltervy_washout = (vy_model(1:end-1) + T*(cfilt_ay_COG.*(1-rollGrad) - yawRate_VBOX(1:end-1).*vx(1:end-1)));
% filtVy_washout = filter(a_lp, b_lp, prefiltervy_washout);
filtVy_washout = filter(a_lp, b_lp, vy_model) + filter(a_hp, b_hp, hp_vy_inter);
beta_washout = filtVy_washout./vx(1:end-1);
% beta_washout(label<1)=0;
subplot(2,2,man)
plot(Beta_VBOX_smooth,'b')
hold on
plot(beta_washout,'r')
legend(["Beta\_VBOX","Beta\_washout"],'Location','best')
drawnow

[e_beta_mean,e_beta_max,time_at_max,error] = errorCalc(beta_washout,Beta_VBOX_smooth);
disp(' ');
fprintf('Wash out filter based calculation\n')
fprintf('The MSE of Beta estimation is: %d \n',e_beta_mean);
fprintf('The Max error of Beta estimation is: %d \n',e_beta_max);


end