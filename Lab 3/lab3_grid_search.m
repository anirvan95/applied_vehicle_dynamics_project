%%Gaar phar grid search 
clc
clear all
close all

mp = 0.16;
cp = 0.4;
kp = 6.32;

s = tf('s');
tf_damped = (cp*s + kp)/(mp*s*s + cp*s + kp);

h1 = 0.01;
t1 = 0:h1:2*pi;
omega = 1; 
u_sin = 0.05*sin(omega*t1);

impulse_amp = 0.05;
h2 = 0.01;
t2 = 0:h2:40;
u_impulse = zeros(length(t2),1);
u_impulse(10) = impulse_amp;

[y_damped_sin,~,~] = lsim(tf_damped, u_sin, t1);
[y_damped_impulse,~,~] = lsim(tf_damped, u_impulse, t2);
steadyState = movmean(abs(y_damped_impulse),25) < 1e-5;
indices = find(steadyState == 1);
settlingTime_damped = indices(1)*h2;
[pks_damped,loc_damped] = findpeaks(abs(y_damped_impulse));
dp_vect = -0.5:0.01:0.015;
dd_vect = -0.5:0.1:2.0;

for i=1:length(dp_vect)
    i
    for j=1:length(dd_vect)
        j
        tf_active = kp/(mp*s*s + dd_vect(j)*s + kp+dp_vect(i));
        [y_act_sin,~,~] = lsim(tf_active, u_sin, t1);
        sin_amp_improv(i,j) = max(y_damped_sin) - max(y_act_sin);
        
        [y_act_impulse,~,~] = lsim(tf_active, u_impulse, t2);
        [pks,loc] = findpeaks(abs(y_act_impulse));
        
        if length(pks)>=3 && ((pks(1)/pks(2)) < 30) %ensuring osciallations
            imp_amp_improv(i,j) = pks_damped(1) - pks(1) + pks_damped(2) - pks(2)+ pks_damped(3) - pks(3);
        else
            imp_amp_improv(i,j) = 0;
        end
        steadyState = movmean(abs(y_act_impulse),25) < 1e-5;
        indices = find(steadyState == 1);
        if ~(isempty(indices))
            settlingTime_improv(i,j) = settlingTime_damped - indices(1)*h2;
        else
            settlingTime_improv(i,j) = 0;
        end
        improv_total(i,j) = sin_amp_improv(i,j)+ imp_amp_improv(i,j) + settlingTime_improv(i,j);
    end
end

max_improv = max(improv_total(:));
for i=1:length(dp_vect)
    for j=1:length(dd_vect)
        if(improv_total(i,j) == max_improv)
            disp("optimal dp value")
            disp(dp_vect(i))
            disp("optimal dd value")
            disp(dd_vect(j))
            break
        end
    end
end