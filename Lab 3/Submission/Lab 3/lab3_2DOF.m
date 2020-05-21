clc
clear all
close all

s = tf('s');

%%1DOF 
mp1 = 0.16;
cp1 = 0.4;
kp1 = 6.32;

tf_1dof = (cp1*s + kp1)/(mp1*s*s + cp1*s + kp1);

%%2DOF
mp = 0.16;
ms = 0.16;
cp = 0.8;
cs = 0.05;
kp = 6.32;
ks = 0.0632;

tf_2dof = (cp*cs*s^2 + (cp*ks + cs*kp)*s + kp*ks)/(mp*ms*s^4 + (ms*cp + ms*cs + mp*cs)*s^3 + (ms*kp + ms*ks + mp*ks + cp*cs)*s^2 + (cp*ks + cs*kp)*s + kp*ks);

%%Task 6.2
bode(tf_1dof,'r');
hold on;
bode(tf_2dof,'b');
legend('1 DOF', '2 DOF')
grid on

%%Defining input signals
h1 = 0.01;
omega = 0.626;
t1 = 0:h1:2*pi/omega;
u_sin = 0.05*sin(omega*t1);
[y_sin,~,~] = lsim(tf_2dof, u_sin, t1); 

impulse_amp = 0.05;
h2 = 0.01;
t2 = 0:h2:10;
u_impulse = zeros(length(t2),1);
u_impulse(100) = impulse_amp;
[y_impulse,~,~] = lsim(tf_2dof, u_impulse, t2);

step_amp = 0.05;
h3 = 0.01;
t3 = 0:h3:40;
u_step = zeros(length(t3),1);
u_step(t3<(1/step_amp)) = impulse_amp;
[y_step,~,~] = lsim(tf_2dof, u_step, t3);

w = 0:0.01:25;
H = freqresp(tf_2dof,w);
Sw = (4.028*(10^-7))./(2.88*(10^-4) + 0.68*(w.^2) + (w.^4));
H = squeeze(H)';
psd = abs((H.^2)).*(Sw);

[b, a] = tfdata(tf_2dof, 'v');
[A_pass, B_pass, C_pass, D_pass] = tf2ss(b,a);

%%Skyhook controller
T_vect = [0.1, 1];
T = T_vect(1);
y_sin_skyhook = genResponse(u_sin,h1,T_vect(1));
y_sin_skyhook_2 = genResponse(u_sin,h1,T_vect(2));
y_impulse_skyhook = genResponse(u_impulse,h2,T_vect(1));
y_impulse_skyhook_2 = genResponse(u_impulse,h2,T_vect(2));
y_step_skyhook = genResponse(u_step,h3,T_vect(1));
y_step_skyhook_2 = genResponse(u_step,h3,T_vect(2));

%%Task 7.1
figure,
plot(t1,u_sin,'-k','Linewidth',1)
hold on
plot(t1,y_sin,'g','Linewidth',1.0)
plot(t1,y_sin_skyhook(1,:),'b','Linewidth',1.0)
plot(t1,y_sin_skyhook_2(1,:),'r','Linewidth',1.0)
grid on
xlabel('Time (sec)')
ylabel('Amplitude m_s (m)')
title('Sinusoidal Response of 2 DOF system')
legend('Input','Output passive', 'Output Skyhook T=0.1','Output Skyhook T=1')

figure,
plot(t2,u_impulse/20,'-.k','Linewidth',1)
hold on
plot(t2,y_impulse,'g','Linewidth',1.0)
plot(t2,y_impulse_skyhook(1,:),'b','Linewidth',1.0)
plot(t2,y_impulse_skyhook_2(1,:),'r','Linewidth',1.0)
grid on
xlabel('Time (sec)')
ylabel('Amplitude m_s (m)')
title('Impulse Response of  2 DOF system')
legend('Input (scaled by 0.05)','Output passive', 'Output Skyhook T=0.1','Output Skyhook T=1')

figure,
plot(t3,u_step,'-.k','Linewidth',1)
hold on
plot(t3,y_step,'g','Linewidth',1.0)
plot(t3,y_step_skyhook(1,:),'b','Linewidth',1.0)
plot(t3,y_step_skyhook_2(1,:),'r','Linewidth',1.0)
grid on
xlabel('Time (sec)')
ylabel('Amplitude m_s (m)')
title('Step Response of  2 DOF system')
legend('Input','Output passive', 'Output Skyhook T=0.1','Output Skyhook T=1')
