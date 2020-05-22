clc
clear all
close all

mp = 0.16;
cp = 0.4;
kp = 6.32;

%% Task 1
%Task 1.2
wn = sqrt(kp/mp);
ita = cp/(2*sqrt(kp*mp));

%Task 1.3
s = tf('s');
tf_damped = (cp*s + kp)/(mp*s*s + cp*s + kp);
tf_undamped = kp/(mp*s*s + kp);

figure,
bode(tf_damped,'r');
hold on;
bode(tf_undamped,'b');
legend('Damped', 'Undamped')

%Task 1.4
%%Defining input excitation signals
%Sin 1 Hz
h1 = 0.01;
t1 = 0:h1:2*pi;
omega = 6.28; 
u_sin = 0.05*sin(omega*t1);
%Impulse
impulse_amp = 0.05;
h2 = 0.01;
t2 = 0:h2:10;
u_impulse = zeros(length(t2),1);
u_impulse(100) = impulse_amp;
%Step
step_amp = 0.05;
h3 = 0.01;
t3 = 0:h3:40;
u_step = zeros(length(t3),1);
u_step(t3<(1/step_amp)) = impulse_amp;

[y_damped_sin,~,~] = lsim(tf_damped, u_sin, t1); 
[y_damped_impulse,~,~] = lsim(tf_damped, u_impulse, t2,'b');
[y_damped_step,~,~] = lsim(tf_damped, u_step, t3);

% figure,
% plot(t1,u_sin,'b','Linewidth',1)
% hold on
% plot(t1,y_damped_sin,'r','Linewidth',1.2)
% grid on
% xlabel('time')
% ylabel('amplitude')
% title('Sinusoidal Response of damped system')
% legend('input','output')

% figure,
% plot(t2,u_impulse/10,'b','Linewidth',1)
% hold on
% plot(t2,y_damped_impulse,'r','Linewidth',1.2)
% grid on
% xlabel('Time(sec)','Fontsize',14)
% ylabel('Amplitude(m)','Fontsize',14)
% title('Impulse Response of damped system')
% legend('Input (scaled by 0.1)','Output','Fontsize',12)

%%Analysis 
ita_ud = 0.1;
ita_cd = 1;
ita_od = 2;
tf_ud = (2*ita_ud*wn*s + wn*wn)/(s*s + 2*ita_ud*wn*s + wn*wn);
tf_cd = (2*ita_cd*wn*s + wn*wn)/(s*s + 2*ita_cd*wn*s + wn*wn);
tf_od = (2*ita_od*wn*s + wn*wn)/(s*s + 2*ita_od*wn*s + wn*wn);

[y_ud,~,~] = lsim(tf_ud, u_impulse, t2,'r');
[y_cd,~,~] = lsim(tf_cd, u_impulse, t2,'g');
[y_od,~,~] = lsim(tf_od, u_impulse, t2,'b');

figure,
plot(t2,u_impulse,'k','Linewidth',1)
hold on
plot(t2,y_damped_impulse,'r','Linewidth',1)
plot(t2,y_ud,'g','Linewidth',1)
plot(t2,y_cd,'m','Linewidth',1)
plot(t2,y_od,'b','Linewidth',1)
grid on
xlabel('Time(sec)','Fontsize',14)
ylabel('Amplitude(m)','Fontsize',14)
title('Impulse Response of damped system for different ratio')
legend('Input','Output damped \eta=0.4','Output \eta=0.1','Output \eta=1','Output \eta=2','Fontsize',12)

[y_ud,~,~] = lsim(tf_ud, u_step, t3,'r');
[y_cd,~,~] = lsim(tf_cd, u_step, t3,'g');
[y_od,~,~] = lsim(tf_od, u_step, t3,'b');

figure,
plot(t3,u_step,'k','Linewidth',1)
hold on
plot(t3,y_damped_step,'r','Linewidth',1)
plot(t3,y_ud,'g','Linewidth',1)
plot(t3,y_cd,'m','Linewidth',1)
plot(t3,y_od,'b','Linewidth',1)
grid on
xlabel('Time(sec)','Fontsize',14)
ylabel('Amplitude(m)','Fontsize',14)
title('Step Response of damped system for different ratio')
legend('Input','Output damped \eta=0.4','Output \eta=0.1','Output \eta=1','Output \eta=2','Fontsize',12)

%PSD response of track disturbance
w = 0:0.01:25;
H = freqresp(tf_damped,w);
Sw = (4.028*(10^-7))./(2.88*(10^-4) + 0.68*(w.^2) + (w.^4));
H = squeeze(H)';
psd = abs((H.^2)).*(Sw);
% figure,
% semilogy(w,Sw,'r');
% hold on
% semilogy(w,psd,'b');
% grid on
% xlabel('Frequency (rad/sec)','Fontsize',14)
% ylabel('Power Spectral Density','Fontsize',14)
% title('PSD Response of damped system')
% legend('Input','Output')

%% Task 2 PD Control
%Task 2.2
dp = 0;
dd = 0.4;
tf_pd_untuned = kp/(mp*s*s + dd*s + kp+dp);
[y_sin_pdut,~,~] = lsim(tf_pd_untuned, u_sin, t1);
[y_impulse_pdut,~,~] = lsim(tf_pd_untuned, u_impulse, t2);
[y_step_pdut,~,~] = lsim(tf_pd_untuned, u_step, t3);
H_pd_ut = freqresp(tf_pd_untuned,w);
H_pd_ut = squeeze(H_pd_ut)';
psd_pd_ut = abs((H_pd_ut.^2)).*(Sw);

%%Task 2.3
dp = 0.01;
dd = 1.2;
tf_pd_tuned = kp/(mp*s*s + dd*s + kp+dp);
[y_sin_pd,~,~] = lsim(tf_pd_tuned, u_sin, t1);
[y_impulse_pd,~,~] = lsim(tf_pd_tuned, u_impulse, t2);
[y_step_pd,~,~] = lsim(tf_pd_tuned, u_step, t3);
H_pd = freqresp(tf_pd_tuned,w);
H_pd = squeeze(H_pd_ut)';
psd_pd = abs((H_pd_ut.^2)).*(Sw);

figure,
plot(t1,u_sin,'-k','Linewidth',1)
hold on
plot(t1,y_damped_sin,'g','Linewidth',1.0)
plot(t1,y_sin_pdut,'r','Linewidth',1.0)
plot(t1,y_sin_pd,'b','Linewidth',1.0)
grid on
xlabel('time')
ylabel('amplitude')
title('Sinusoidal Response of damped system')
legend('Input','Output damped','Output active PD untuned', 'Output active PD tuned')

figure,
plot(t2,u_impulse/10,'-.k','Linewidth',1)
hold on
plot(t2,y_damped_impulse,'g','Linewidth',1.0)
plot(t2,y_impulse_pdut,'r','Linewidth',1.0)
plot(t2,y_impulse_pd,'b','Linewidth',1.0)
grid on
xlabel('time')
ylabel('amplitude')
title('Impulse Response of damped system')
legend('Input (scaled by 0.1)','Output damped','Output active PD untuned', 'Output active PD tuned')
figure,
plot(t3,u_step,'-.k','Linewidth',1)
hold on
plot(t3,y_damped_step,'g','Linewidth',1.0)
plot(t3,y_step_pdut,'r','Linewidth',1.0)
plot(t3,y_step_pd,'b','Linewidth',1.0)
grid on
xlabel('time')
ylabel('amplitude')
title('Step Response of damped system')
legend('Input','Output damped','Output active PD untuned', 'Output active PD tuned')


figure,
semilogy(w,Sw,'-.k');
hold on
semilogy(w,psd,'g','Linewidth',2);
semilogy(w,psd_pd_ut,'r','Linewidth',2);
semilogy(w,psd_pd,'b','Linewidth',1.5);
grid on
xlabel('Frequency (rad/sec)','Fontsize',14)
ylabel('Power Spectral Density','Fontsize',14)
title('PSD Response of damped system')
legend('Input','Output damped','Output active PD untuned', 'Output active PD tuned')


% Extra task 1
% dd = 2*sqrt(kp*mp);
% dp = 0;
% tf_active = kp/(mp*s*s + dd_vect(j)*s + kp+dp_vect(i));

%% Task 3 - PID Control
%Task 3.1
hp = 0.01;
hd = 1.4;
hi = 0.1;
tf_pid = (kp*s)/(mp*s*s*s + hd*s*s + (kp+hp)*s + hi);
[y_sin_pid,~,~] = lsim(tf_pid, u_sin, t1);
[y_impulse_pid,~,~] = lsim(tf_pid, u_impulse, t2);
[y_step_pid,~,~] = lsim(tf_pid, u_step, t3);

H_pid = freqresp(tf_pid,w);
H_pid = squeeze(H_pid)';
psd_pid = abs((H_pid.^2)).*(Sw);


%% Task 4 - Skyhook control
%Task 4
T = 1.05;
tf_skyhook = kp/(mp*s*s + T*s + kp);
[y_sin_skyhook,~,~] = lsim(tf_skyhook, u_sin, t1);
[y_impulse_skyhook,~,~] = lsim(tf_skyhook, u_impulse, t2);
[y_step_skyhook,~,~] = lsim(tf_skyhook, u_step, t3);
H_pd_skyhook = freqresp(tf_skyhook,w);
H_pd_skyhook = squeeze(H_pd_skyhook)';
psd_skyhook = abs((H_pd_skyhook.^2)).*(Sw);

%Task 5.1
figure,
subplot(2,2,1)
plot(t1,u_sin,'-.k','Linewidth',1)
hold on
plot(t1,y_damped_sin,'g','Linewidth',1.0)
plot(t1,y_sin_pd,'c','Linewidth',1.0)
plot(t1,y_sin_pid,'r','Linewidth',1.0)
plot(t1,y_sin_skyhook,'b','Linewidth',1.0)
grid on
xlabel('time')
ylabel('amplitude')
title('Sinusoidal Response of 1 DOF system')
legend('Input','Output damped','Output PD', 'Output active PID', 'Output active Skyhook')

subplot(2,2,2)
plot(t2,u_impulse/10,'-.k','Linewidth',1)
hold on
plot(t2,y_damped_impulse,'g','Linewidth',1.0)
plot(t2,y_impulse_pd,'c','Linewidth',1.0)
plot(t2,y_impulse_pid,'r','Linewidth',1.0)
plot(t2,y_impulse_skyhook,'b','Linewidth',1.0)
grid on
xlabel('time')
ylabel('amplitude')
title('Impulse Response of  1 DOF system')
legend('Input (scaled by 0.1)','Output damped','Output PD','Output active PID', 'Output active skyhook')

subplot(2,2,3)
plot(t3,u_step,'-.k','Linewidth',1)
hold on
plot(t3,y_damped_step,'g','Linewidth',1.0)
plot(t3,y_step_pd,'c','Linewidth',1.0)
plot(t3,y_step_pid,'r','Linewidth',1.0)
plot(t3,y_step_skyhook,'b','Linewidth',1.0)
grid on
xlabel('time')
ylabel('amplitude')
title('Step Response of  1 DOF system')
legend('Input','Output damped','Output PD','Output active PID', 'Output active skyhook')


subplot(2,2,4),
semilogy(w,Sw,'-.k');
hold on
semilogy(w,psd,'g','Linewidth',2);
semilogy(w,psd_pd,'c','Linewidth',2);
semilogy(w,psd_pid,'r','Linewidth',2);
semilogy(w,psd_skyhook,'b','Linewidth',1.5);
grid on
xlabel('Frequency (rad/sec)','Fontsize',14)
ylabel('Power Spectral Density','Fontsize',14)
title('PSD Response of 1 DOF system')
legend('Input','Output damped','Output PD','Output active PID', 'Output active skyhook')


%%Task 5.2
%%Bode plot
figure,
bodemag(tf_damped);
hold on
bodemag(tf_pd_tuned);
bodemag(tf_pid);
bodemag(tf_skyhook);
grid on
legend('Passive system damped','Active system PD','Active system  PID', 'Active system skyhook')





