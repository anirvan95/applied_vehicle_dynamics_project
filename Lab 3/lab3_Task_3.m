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

%Task 1.4
h1 = 0.01;
t1 = 0:h1:2*pi;
omega = 1; 
u_sin = 0.05*sin(omega*t1);
figure,
lsim(tf_damped, u_sin, t1,'r'); 

impulse_amp = 0.05;
h2 = 0.01;
t2 = 0:h2:40;
u_impulse = zeros(length(t2),1);
u_impulse(10) = impulse_amp;
figure,
lsim(tf_damped, u_impulse, t2,'b');

%%Analysis 
ita_ud = 0.1;
ita_cd = 1;
ita_od = 2;
tf_ud = (2*ita_ud*wn*s + wn*wn)/(s*s + 2*ita_ud*wn*s + wn*wn);
tf_cd = (2*ita_cd*wn*s + wn*wn)/(s*s + 2*ita_cd*wn*s + wn*wn);
tf_od = (2*ita_od*wn*s + wn*wn)/(s*s + 2*ita_od*wn*s + wn*wn);
figure,
lsim(tf_ud, u_impulse, t2,'r');
hold on
lsim(tf_cd, u_impulse, t2,'g');
lsim(tf_od, u_impulse, t2,'b');

% u_step = t2>=1;
% lsim(tf_ud, u_step, t2,'r');
% hold on
% lsim(tf_cd, u_step, t2,'g');
% lsim(tf_od, u_step, t2,'b');

w = 0:0.01:25;
H = freqresp(tf_damped,w);
Sw = (4.028*(10^-7))./(2.88*(10^-4) + 0.68*(w.^2) + (w.^4));
H = squeeze(H)';
psd = abs((H.^2)).*(Sw);
figure,
semilogy(w,psd,'k');

%% Task 2
%Task 2.2
dp = 0;
dd = 0.4;
tf_active = kp/(mp*s*s + dd*s + kp+dp);
figure,
subplot(1,3,1),
lsim(tf_damped, u_sin, t1, 'r')
hold on
lsim(tf_active, u_sin, t1, 'b')

subplot(1,3,2),
lsim(tf_damped, u_impulse, t2, 'r')
hold on
lsim(tf_active, u_impulse, t2, 'b')

H_active = freqresp(tf_active,w);
H_active = squeeze(H_active)';
psd_active = abs((H_active.^2)).*(Sw);

subplot(1,3,3),
semilogy(w,psd,'r');
hold on
semilogy(w,psd_active,'b');

% Task 2.3
dp = 0;
dd = 1.0;
tf_active = kp/(mp*s*s + dd*s + kp+dp);
tf_active_undamped = kp/(mp*s*s + kp+dp);
bode(tf_active_undamped)

subplot(1,3,1),
lsim(tf_damped, u_sin, t1, 'r')
hold on
lsim(tf_active, u_sin, t1, 'b')

subplot(1,3,2),
lsim(tf_damped, u_impulse, t2, 'r')
hold on
lsim(tf_active, u_impulse, t2, 'b');

H_active = freqresp(tf_active,w);
H_active = squeeze(H_active)';
psd_active = abs((H_active.^2)).*(Sw);

subplot(1,3,3),
semilogy(w,psd,'r');
hold on
semilogy(w,psd_active,'b');

% Extra task 1
% dd = 2*sqrt(kp*mp);
% dp = 0;
% tf_active = kp/(mp*s*s + dd_vect(j)*s + kp+dp_vect(i));

%% Task 3
hp = 0.01;
hd = 1;
hi = 0.1;
tf_pid = (kp*s)/(mp*s*s*s + hd*s*s + (kp+hp)*s + hi);
u_step = 0.05.*(t2>=1);
% lsim(tf_ud, u_step, t2,'r');

subplot(1,3,1),
lsim(tf_damped, u_sin, t1, 'r')
hold on
lsim(tf_pid, u_sin, t1, 'b')

subplot(1,3,2),
lsim(tf_damped, u_impulse, t2, 'r')
hold on
lsim(tf_pid, u_impulse, t2, 'b')

H_pid = freqresp(tf_pid,w);
H_pid = squeeze(H_pid)';
psd_pid = abs((H_pid.^2)).*(Sw);

subplot(1,3,3),
semilogy(w,psd,'r');
hold on
semilogy(w,psd_pid,'b');



