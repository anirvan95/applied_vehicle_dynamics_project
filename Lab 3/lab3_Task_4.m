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

bode(tf_1dof,'r');
hold on;
bode(tf_2dof,'b');
legend('1 DOF', '2 DOF')
grid on

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
Ask = [0, 1, 0, 0;-ks/ms, -T/ms, ks/ms, 0;0, 0, 0, 1;ks/mp, T/mp, -(ks + kp)/mp, -cp/mp];
Bsk = [0, 0; 0, 0; 0, 0; kp/mp, cp/mp];
Csk = [1, 0, 0, 0];
Dsk = [0,0];
% %%Sine
% sys = ss(Ask,Bsk,Csk,Dsk);
% dsys_impulse = c2d(sys, h1);
% y_sin_skyhook = [0;0;0;0];
% for iter = 1:(length(t1)-1)
%     zw = u_sin(iter);
%     zw_dot = (u_sin(iter+1) - u_sin(iter))/h1;
%     y_sin_skyhook(:,iter+1) = dsys_impulse.A*y_sin_skyhook(:,iter) + dsys_impulse.B*[zw;zw_dot];
% end
% 
% %%Impulse
% dsys_impulse = c2d(sys, h2);
% y_impulse_skyhook = [0;0;0;0];
% for iter = 1:(length(t2)-1)
%     zw = u_impulse(iter);
%     zw_dot = (u_impulse(iter+1) - u_impulse(iter))/h2;
%     y_impulse_skyhook(:,iter+1) = dsys_impulse.A*y_impulse_skyhook(:,iter) + dsys_impulse.B*[zw;zw_dot];
% end



% figure,
% plot(t1,u_sin,'-k','Linewidth',1)
% hold on
% plot(t1,y_sin,'g','Linewidth',1.0)
% plot(t1,y_sin_skyhook(1,:),'b','Linewidth',1.0)
% plot(t1,y_sin_skyhook_2(1,:),'r','Linewidth',1.0)
% grid on
% xlabel('Time (sec)')
% ylabel('Amplitude m_s (m)')
% title('Sinusoidal Response of 1 DOF system')
% legend('Input','Output passive', 'Output Skyhook T=0.1','Output Skyhook T=1')
% 
% figure,
% plot(t2,u_impulse/20,'-.k','Linewidth',1)
% hold on
% plot(t2,y_impulse,'g','Linewidth',1.0)
% plot(t2,y_impulse_skyhook(1,:),'b','Linewidth',1.0)
% plot(t2,y_impulse_skyhook_2(1,:),'r','Linewidth',1.0)
% grid on
% xlabel('Time (sec)')
% ylabel('Amplitude m_s (m)')
% title('Impulse Response of  1 DOF system')
% legend('Input (scaled by 0.05)','Output passive', 'Output Skyhook T=0.1','Output Skyhook T=1')

