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

    load('init.mat')        
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Here you can ad your control parameters for the electric motor

Kp_em = 10.75;
Ki_em = 20.8;
Kd_em = 0;

% Here you can ad your control parameters for the brake 

Kp_brake = 4.0;
Kp_brake_r = 4.0;
Ki_brake = 25;
Ki_brake_r = 22.5;
Kd_brake = 0;
Kd_brake_r = 0;

% Run your model
sim('SD2231_Lab1.slx')

% Calculate the X-Y data, plot and estimate the optimum slip value
% Plot longitudinal slip on X-axis and used friction mu on y-axis


%%
close all
time = dt.*(1:size(ax));
subplot(1,3,1), plot(slip(:,2),'r')
hold on
subplot(1,3,1), plot(init.slip,'b');
subplot(1,3,2), plot(vel*18/5,'r', 'Linewidth', 1);
hold on
subplot(1,3,2), plot(init.vel*18/5,'b', 'Linewidth', 1);
subplot(1,3,3), plot(throttle,'r', 'Linewidth', 1);
hold on
subplot(1,3,3), plot(Brake,'r--')
subplot(1,3,3), plot(init.throttle,'b', 'Linewidth', 1);

pid_out = squeeze(PID_output.data);
figure,
plot(pid_out), title("PID Output")
figure,
plot(RL_radius.*rot_RL)
hold on
plot(vel)
yyaxis right, plot(rot_RL)
legend("RL\_omega", "vel", "rot")

%%
close all
time = dt.*(1:size(ax));
subplot(1,3,1), plot(slip(:,1),'r')
hold on
subplot(1,3,1), plot(init.slip_bR,'b'), title("slip\_bR")

subplot(1,3,2), plot(slip(:,3),'r')
hold on
subplot(1,3,2), plot(init.slip_bF,'b'), title("slip\_bF")

subplot(1,3,3), plot(vel*18/5,'r', 'Linewidth', 1);
hold on
subplot(1,3,3), plot(init.vel*18/5,'b', 'Linewidth', 1), title("speed")

figure
subplot(1,3,1), plot(Brake,'r')
hold on
subplot(1,3,1), plot(init.brake,'b', 'Linewidth', 1), title("Brake")

pid_out_br = squeeze(PID_OUT_BR.data);
pid_out_bf = squeeze(PID_OUT_BF.data);
subplot(1,3,2), plot(pid_out_bf), title("PID Output front brake")
subplot(1,3,3), plot(pid_out_br), title("PID output rear brake")

figure,
plot(RL_radius.*rot_RL)
hold on
plot(vel)
yyaxis right
plot(rot_RL)
legend("RL\_omega", "vel", "rot")

%%
close all
label = (ax>0);
label_br = (ax<0);
% Fx = label.*Veh.ms.*ax.*0.5;
% mu_r = Fx./(Veh.lambda*Veh.ms*g - Veh.h/Veh.L*Fx);

TFxr = label.*FxRL;
TFxf = label.*FxFL;
% mu_r = Fx./(Veh.lambda*Veh.ms*g - Veh.h/Veh.L*Fx);
mu_tr = TFxr./(Veh.lambda*Veh.ms*g - Veh.h/Veh.L*(TFxr+TFxf));

BFxr = abs(label_br.*FxRL);
BFxf = abs(label_br.*FxFL);
mu_br = BFxr./(Veh.lambda*Veh.ms*g - Veh.h/Veh.L*(BFxr+BFxf));
mu_bf = BFxf./((1-Veh.lambda)*Veh.ms*g + Veh.h/Veh.L*(BFxr+BFxf));

% plot(mu_bf)
% hold on
% plot(mu_br)
% legend("front","rear")

figure, plot(slip(1:4000, 2), mu_tr(1:4000))
figure, plot(slip(5000:7045,1),mu_br(5000:7045)), title("Rear tyre")
figure, plot(slip(5000:7045,3),mu_bf(5000:7045)), title("Front tyre")
