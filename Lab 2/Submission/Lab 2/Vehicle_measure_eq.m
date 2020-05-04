function y_n = Vehicle_measure_eq(x,param)
% ADDME Measurement function
%    x = the states
%    param = parameters that you might need, such as vehicle parameters.

global lf lr mass Cf Cr

alpha_12 = atan2((x(2) + x(3)*lf),x(1)) - param.input;
alpha_34 = atan2((x(2) - x(3)*lr),x(1));
F_12 = -Cf*alpha_12;
F_34 = -Cr*alpha_34;

y_n = [x(1);(F_34 + F_12*cos(param.input))/mass;x(3)];
