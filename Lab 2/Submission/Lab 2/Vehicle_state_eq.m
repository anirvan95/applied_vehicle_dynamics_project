function x_n = Vehicle_state_eq(x,param)
% ADDME Dynamic model function
%    x = the states
%    param = parameters that you might need, such as vehicle parameters.
% param(1) = Steering Angle


global lf lr mass Iz Cf Cr

alpha_12 = atan2((x(2) + x(3)*lf),x(1)) - param.input;
alpha_34 = atan2((x(2) - x(3)*lr),x(1));
F_12 = -Cf*alpha_12;
F_34 = -Cr*alpha_34;
f_x = [x(3)*x(2) - F_12*sin(param.input)/mass; -x(3)*x(1) + (F_34+F_12*cos(param.input))/mass;(lf*F_12*cos(param.input) - lr*F_34)/Iz];


% Integrate using Runge Kutta (in the script folder) or simple euler forward
dt = param.dt;
f = @(x)[f_x(1,:);f_x(2,:);f_x(3,:)];
x_n = rk4(f,dt,x(1:3,:));