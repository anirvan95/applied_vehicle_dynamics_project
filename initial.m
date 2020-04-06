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

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Here you can ad your control parameters for the electric motor

Kp_em = 6.5;
Ki_em = 1.2;
Kd_em = 0.0;

% Here you can ad your control parameters for the brake 

Kp_brake = 0;
Ki_brake = 0;
Kd_brake = 0;

% Run your model
sim('SD2231_Lab1.slx')

% Calculate the X-Y data, plot and estimate the optimum slip value
% Plot longitudinal slip on X-axis and used friction mu on y-axis

%%
close all
acclabel = (ax>0);
brklabel = (ax<0);
AFxr = acclabel.*FxRL;
ANr = acclabel.*(FzRL+FzRR);
AFxf = acclabel.*FxFL;
ANf = acclabel.*(FzFL+FzFR);

BFxr = brklabel.*FxRL;
BNr = brklabel.*(FzRL+FzRR);
BFxf = brklabel.*FxFL;
BNf = brklabel.*(FzFL+FzFR);

% mu_r = Fx./(Veh.lambda*Veh.ms*g - Veh.h/Veh.L*Fx);
%mu_tr = Fxr./(Veh.lambda*Veh.ms*g - Veh.h/Veh.L*(Fxr+Fxf));
Amu_r = AFxr./ANr;
Amu_f = AFxf./ANf;
Bmu_f = -BFxf./BNf;
Bmu_r = -BFxr./BNr;

%% Plot 2.c [Modified_slip_observer]
time = dt.*(1:size(Amu_r,1));
figure,plot(squeeze(time),slip(:,2),'b','Linewidth',2);
title("Rear tractive slip",'FontSize',12,'FontWeight','bold')
xlabel('Time [sec]','FontSize',10,'FontWeight','bold')
ylabel('RL Slip','FontSize',10,'FontWeight','bold')
grid on

time = dt.*(1:size(Bmu_r,1));
figure,plot(squeeze(time),slip(:,4),'r','Linewidth',2);
title("Front tractive slip",'FontSize',12,'FontWeight','bold')
xlabel('Time [sec]','FontSize',10,'FontWeight','bold')
ylabel('FL Slip','FontSize',10,'FontWeight','bold')
grid on


time = dt.*(1:size(Bmu_r,1));
figure,plot(squeeze(time),slip(:,1),'b','Linewidth',2);
title("Rear braking slip",'FontSize',12,'FontWeight','bold')
xlabel('Time [sec]','FontSize',10,'FontWeight','bold')
ylabel('RL Slip','FontSize',10,'FontWeight','bold')
grid on

time = dt.*(1:size(Bmu_r,1));
figure,plot(squeeze(time),slip(:,3),'r','Linewidth',2);
title("Front braking slip",'FontSize',12,'FontWeight','bold')
xlabel('Time [sec]','FontSize',10,'FontWeight','bold')
ylabel('FL Slip','FontSize',10,'FontWeight','bold')
grid on

%%Used friction over longitudinal slip
time = dt.*(1:size(Bmu_r,1));
figure,plot(slip(1:4000,2),Amu_r(1:4000),'k','Linewidth',2);
title("Rear Friction vs Tractive Slip",'FontSize',12,'FontWeight','bold')
xlabel('RL Slip','FontSize',10,'FontWeight','bold')
ylabel('\mu_r','FontSize',10,'FontWeight','bold')
grid on

figure,plot(slip(5000:7600,1),Bmu_r(5000:7600),'b','Linewidth',2);
title("Rear Friction vs Braking Slip",'FontSize',12,'FontWeight','bold')
xlabel('RL Slip','FontSize',10,'FontWeight','bold')
ylabel('\mu_r','FontSize',10,'FontWeight','bold')
grid on

figure,plot(slip(5000:7600,3),Bmu_f(5000:7600),'r','Linewidth',2);
title("Front Friction vs Braking Slip",'FontSize',12,'FontWeight','bold')
xlabel('FL Slip','FontSize',10,'FontWeight','bold')
ylabel('\mu_f','FontSize',10,'FontWeight','bold')
grid on

InitialData.slip = slip;
InitialData.throttle = throttle;
InitialData.ax = ax;
InitialData.Amu_f = Amu_f;
InitialData.Amu_r = Amu_r;
InitialData.Bmu_f = Bmu_f;
InitialData.Bmu_r = Amu_r;

save 'InitialData.mat' 'InitialData'

