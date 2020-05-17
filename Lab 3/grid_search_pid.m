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
omega = 6.28; 
u_sin = 0.05*sin(omega*t1);

impulse_amp = 0.05;
h2 = 0.01;
t2 = 0:h2:10;
u_impulse = zeros(length(t2),1);
u_impulse(10) = impulse_amp;

[y_damped_sin,~,~] = lsim(tf_damped, u_sin, t1);
[y_damped_impulse,~,~] = lsim(tf_damped, u_impulse, t2);
steadyState = movmean(abs(y_damped_impulse),25) < 1e-5;
indices = find(steadyState == 1);
settlingTime_damped = indices(1)*h2;
[pks_damped,loc_damped] = findpeaks(abs(y_damped_impulse));

hp_vect = -0.5:0.01:0.01;
hd_vect = -0.5:0.1:3.0;
hi_vect = 0:0.1:2;

for i=1:length(hp_vect)
    i
    for j=1:length(hd_vect)
        j
        for k=1:length(hi_vect)
            k
            tf_active = (kp*s)/(mp*s*s*s + hd_vect(j)*s*s + (kp+hp_vect(i))*s + hi_vect(k));
            [y_act_sin,~,~] = lsim(tf_active, u_sin, t1);
            sin_amp_improv(i,j,k) = max(y_damped_sin) - max(y_act_sin);

            [y_act_impulse,~,~] = lsim(tf_active, u_impulse, t2);
            [pks,loc] = findpeaks(abs(y_act_impulse));

            if length(pks)>=3 && ((pks(1)/pks(2)) < 30) %ensuring osciallations
                imp_amp_improv(i,j,k) = pks_damped(1) - pks(1) + pks_damped(2) - pks(2)+ pks_damped(3) - pks(3);
            else
                imp_amp_improv(i,j,k) = 0;
            end
            steadyState = movmean(abs(y_act_impulse),25) < 1e-5;
            indices = find(steadyState == 1);
            if ~(isempty(indices))
                settlingTime_improv(i,j,k) = settlingTime_damped - indices(1)*h2;
            else
                settlingTime_improv(i,j,k) = 0;
            end
            improv_total(i,j,k) = sin_amp_improv(i,j,k)+ imp_amp_improv(i,j,k) + settlingTime_improv(i,j,k);
        end
    end
end

max_improv = max(improv_total(:));
for i=1:length(hp_vect)
    for j=1:length(hd_vect)
        for k=1:length(hi_vect)
        if(improv_total(i,j,k) == max_improv)
            disp("optimal hi value")
            disp(hi_vect(k))
            disp("optimal hd value")
            disp(hd_vect(j))
            disp("optimal hp value")
            disp(hp_vect(i))
            break
        end
        end
    end
end