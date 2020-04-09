%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  KTH SD2231 - Applied Vehicle Dynamics Control
%
%  Lab:     1 - Slip Control for vehicles
%  Date:    Spring term 2020
%  Teacher: Mikael Nybacka and Wenliang Zhang
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc
close all

global Veh
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Choose input parameters
mu_select = 1;              % Here you can change to different friction levels
                            % but for this lab you should stick to mu_select = 1
                            % representing dry road. 
                            % If you have time you can play around with
                            % other mu levels but the tyre model in
                            % CarMaker will not be realistic for snow or
                            % wet road.
dt = 0.001;
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Vehicle parameters DO NOT CHANGE
% ______________________________________________________________________________
% 
% BASIC PARAMETERS         UNIT          TOTAL
% ______________________________________________________________________________
% 
% Total mass               kg           2107.996
% Total weight             N           20671.009
% Total roll inertia       kgm^2         952.228
% Total pitch inertia      kgm^2        3508.505
% Total yaw inertia        kgm^2        3954.288
% Total CoG height (FrD)   m               0.545
% 
% Sprung mass              kg           1841.860
% Sprung weight            N           18061.279
% Sprung roll inertia      kgm^2         741.760
% Sprung pitch inertia     kgm^2        2903.000
% Sprung yaw inertia       kgm^2        3170.600
% Sprung CoG height (FrD)  m               0.578
% 
% Unsprung mass (total)    kg            266.136
% 
% Wheelbase                m               2.971
% 
% ______________________________________________________________________________
% 
% BASIC PARAMETERS         UNIT      AVERAGE/TOTAL       LEFT         RIGHT
% ______________________________________________________________________________
% 
% Front axle:
% 
% Track width              m               1.662
% Toe Angle                deg             0.200          0.200         0.200
% Camber Angle             deg            -0.000         -0.000        -0.000
% 
% Caster Angle             deg             1.298          1.298         1.298
% Caster Offset            mm              6.077          6.077         6.077
% Kingpin Angle            deg            13.257         13.257        13.257
% Kingpin Offset           mm              2.809          2.809         2.809
% 
% Normal force             N           10436.920       5218.459      5218.461
% Loaded tire radius       m               0.296          0.296         0.296
% Tire rate                N/mm          236.700        236.700       236.700
% ______________________________________________________________________________
% 
% Rear axle:
% 
% Track width              m               1.715
% Toe Angle                deg             0.115          0.115         0.115
% Camber Angle             deg            -1.490         -1.490        -1.490
% 
% Normal force             N           10234.057       5117.028      5117.029
% Loaded tire radius       m               0.297          0.297         0.297
% Tire rate                N/mm          236.700        236.700       236.700
% ______________________________________________________________________________

% Initial values for road vehicle
         g = 9.81;
         Veh.ms  = 1841.860;            % sprung mass of the car in kg 
         Veh.mw = 266.136;              % unsprung mass of the car in kg
         Veh.Jw = 2.592;                % inertia of wheel in kgm^2
         Veh.L = 2.971;                 % Wheelbase
         Veh.lambda = 0.4958;           % Ratio of CoG
         Veh.Lf = Veh.L*Veh.lambda;     % Distance from front axis to CoG
         Veh.Lr = Veh.L*(1-Veh.lambda); % Distance from front axis to CoG
         Veh.h = 0.578;                 % CoG height from ground
         Veh.r = 0.297;                 % rolling radius for tire in m
         
        Veh.mu = [1 0.7 0.3];           % friction coefficients [dry, rain and snow]
        K_em = [230 160 80];
        K_brake = [600 500 400];
        tire_leg={'\mu = 1','\mu = 0.7','\mu = 0.3'};
        switch mu_select
            case {1}
                K_em = K_em(1)
                K_brake = K_brake(1)
                Veh.mu = Veh.mu(1)
            case {2}
                K_em = K_em(2)
                K_brake = K_brake(2)
                Veh.mu = Veh.mu(2)
            case {3}
                K_em = K_em(3)
                K_brake = K_brake(3)
                Veh.mu = Veh.mu(3)
        end

%     load('init.mat')        
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Here you can ad your control parameters for the electric motor

%%Basic PID parameters for Task 4, user defined PID connected in the current file
Kp_em = 7.00;
Ki_em = 12.5;
Kd_em = 0.0;
Kp_brake = 6.75;
Kp_brake_r = 7.25;
Ki_brake = 30.5;
Ki_brake_r = 35.0;
Kd_brake = 0.00;
Kd_brake_r = 0.00;


%%Advanced PID parameters Task 5, connect inbuilt PID block to obtain the
%%results presented in the report
% Kp_em = 6.85;
% Ki_em = 13.5;
% Kd_em = 0.15;
% Kp_brake = 6.75;
% Kp_brake_r = 10.25;
% Ki_brake = 30.5;
% Ki_brake_r = 45.0;
% Kd_brake = 0.00;
% Kd_brake_r = 0.01;
% 
% 
TCS_slip_ref = 0.18;
ABS_slip_refF = 0.175;
ABS_slip_refR = 0.185;



% Run your model
sim('SD2231_Lab1.slx')



%% Plot 2.b and 2.c [Modified Slip Observer]
load('InitialData.mat')
Inittime = dt.*(1:size(InitialData.Amu_r,1));
figure,plot(squeeze(Inittime),InitialData.slip(:,2),'b','Linewidth',2);
title("Rear tractive slip",'FontSize',12,'FontWeight','bold')
xlabel('Time [sec]','FontSize',10,'FontWeight','bold')
ylabel('RL Slip','FontSize',10,'FontWeight','bold')
grid on

figure,plot(squeeze(Inittime),InitialData.slip(:,4),'r','Linewidth',2);
title("Front tractive slip",'FontSize',12,'FontWeight','bold')
xlabel('Time [sec]','FontSize',10,'FontWeight','bold')
ylabel('FL Slip','FontSize',10,'FontWeight','bold')
grid on

figure,plot(squeeze(Inittime),InitialData.slip(:,1),'b','Linewidth',2);
title("Rear braking slip",'FontSize',12,'FontWeight','bold')
xlabel('Time [sec]','FontSize',10,'FontWeight','bold')
ylabel('RL Slip','FontSize',10,'FontWeight','bold')
grid on

figure,plot(squeeze(Inittime),InitialData.slip(:,3),'r','Linewidth',2);
title("Front braking slip",'FontSize',12,'FontWeight','bold')
xlabel('Time [sec]','FontSize',10,'FontWeight','bold')
ylabel('FL Slip','FontSize',10,'FontWeight','bold')
grid on

%%Used friction over longitudinal slip
figure,plot(InitialData.slip(1:4000,2),InitialData.Amu_r(1:4000),'k','Linewidth',2);
title("Rear Friction vs Tractive Slip",'FontSize',12,'FontWeight','bold')
xlabel('RL Slip','FontSize',10,'FontWeight','bold')
ylabel('\mu_r','FontSize',10,'FontWeight','bold')
grid on

figure,plot(InitialData.slip(5000:7600,1),InitialData.Bmu_r(5000:7600),'b','Linewidth',2);
title("Rear Friction vs Braking Slip",'FontSize',12,'FontWeight','bold')
xlabel('RL Slip','FontSize',10,'FontWeight','bold')
ylabel('\mu_r','FontSize',10,'FontWeight','bold')
grid on

figure,plot(InitialData.slip(5000:7600,3),InitialData.Bmu_f(5000:7600),'r','Linewidth',2);
title("Front Friction vs Braking Slip",'FontSize',12,'FontWeight','bold')
xlabel('FL Slip','FontSize',10,'FontWeight','bold')
ylabel('\mu_f','FontSize',10,'FontWeight','bold')
grid on


%% Task 3b and 3c plots
close all

figure, 
time = dt.*(1:size(ax));
%%TCS
plot(time, slip(:,2),'b','Linewidth',2);
hold on 
plot(time, TCS_slip_ref*ones(size(ax)), 'r','Linewidth',2);
title("Tractive Slip vs Time",'FontSize',12,'FontWeight','bold')
xlabel('Time','FontSize',10,'FontWeight','bold')
ylabel('RL Slip','FontSize',10,'FontWeight','bold')
legend("Controlled", "Reference")
grid on

%%ABS
figure,
subplot(1,2,1), plot(time, slip(:,1),'b','Linewidth',2);
hold on 
subplot(1,2,1), plot(time, ABS_slip_refR*ones(size(ax)), 'r','Linewidth',2);
title("Rear braking Slip vs Time",'FontSize',12,'FontWeight','bold')
xlabel('Time','FontSize',10,'FontWeight','bold')
ylabel('RL Slip','FontSize',10,'FontWeight','bold')
legend("Controlled", "Reference")
grid on

subplot(1,2,2), plot(time, slip(:,3),'b','Linewidth',2);
hold on 
subplot(1,2,2), plot(time, ABS_slip_refF*ones(size(ax)), 'r','Linewidth',2);
title("Front braking Slip vs Time",'FontSize',12,'FontWeight','bold')
xlabel('Time','FontSize',10,'FontWeight','bold')
ylabel('FL Slip','FontSize',10,'FontWeight','bold')
legend("Controlled", "Reference")
grid on




%% Include plots of Throttle, Torque output etc.
figure, 
subplot(1,3,1), plot(InitialData.time, InitialData.ax,'b','Linewidth',2);
hold on 
subplot(1,3,1), plot(time, ax, 'r','Linewidth',2);
title("Acceleration Comparision",'FontSize',12,'FontWeight','bold')
xlabel('Time [sec]','FontSize',10,'FontWeight','bold')
ylabel('Longitudinal acc [m/sec^2]','FontSize',10,'FontWeight','bold')
legend("Without controller", "With controller")
grid on

subplot(1,3,2), plot(time, throttle,'g','Linewidth',2);
hold on 
subplot(1,3,2), plot(time, TCS_out.Data, '-.k','Linewidth',2);
ylim([-0.1 1.2])
title("TCS Performance vs Time",'FontSize',12,'FontWeight','bold')
xlabel('Time [sec]','FontSize',10,'FontWeight','bold')
ylabel('Input Throttle and Output Torque','FontSize',10,'FontWeight','bold')
legend("Driver hrottle", "Torque Output")
grid on

subplot(1,3,3), plot(time, Brake,'k','Linewidth',2);
hold on 
subplot(1,3,3), plot(time, ABS_OUT_F.Data, '-.r','Linewidth',2);
subplot(1,3,3), plot(time, ABS_OUT_R.Data, '-.b','Linewidth',2);
ylim([-0.1 1.2])
title("ABS Performance vs Time",'FontSize',12,'FontWeight','bold')
xlabel('Time [sec]','FontSize',10,'FontWeight','bold')
ylabel('Input Brake and Output Torque','FontSize',10,'FontWeight','bold')
legend("Driver Brake", "Torque Output Front","Torque Output Rear")
grid on
