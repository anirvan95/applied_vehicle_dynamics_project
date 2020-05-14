clc
clear all
close all

s = tf('s');

%%1DOF 
mp1 = 0.16;
cp1 = 0.4;
kp1 = 6.32;

tf_1dof = (cp1*s + kp1)/(mp1*s*s + cp1*s + kp1);
bode(tf_1dof,'r');
hold on;

%%2DOF
mp = 0.16;
ms = 0.16;
cp = 0.8;
cs = 0.05;
kp = 6.32;
ks = 0.0632;

tf_2dof = (cp*cs*s^2 + (cp*ks + cs*kp)*s + kp*ks)/(mp*ms*s^4 + (ms*cp + ms*cs + mp*cs)*s^3 + (ms*kp + ms*ks + mp*ks + cp*cs)*s^2 + (cp*ks + cs*kp)*s + kp*ks);

bode(tf_2dof,'b');
hold on

h1 = 0.001;
t1 = 0:h1:2*pi;
omega = 1;
u_sin = 0.05*sin(omega*t1);
figure,
lsim(tf_2dof, u_sin, t1,'r');

impulse_amp = 0.05;
h2 = 0.001;
t2 = 0:h2:40;
u_impulse = zeros(length(t2),1);
u_impulse(10) = impulse_amp;
lsim(tf_2dof, u_impulse, t2);

w = 0:0.01:25;
H = freqresp(tf_2dof,w);
Sw = (4.028*(10^-7))./(2.88*(10^-4) + 0.68*(w.^2) + (w.^4));
H = squeeze(H)';
psd = abs((H.^2)).*(Sw);
subplot(1,3,3)
semilogy(w,psd,'k');

%%Skyhook controller 
T = 0.1;
A = [0, 1, 0, 0;-ks/ms, -T/ms, ks/ms, 0;0, 0, 0, 1;ks/mp, T/mp, -(ks + kp)/mp, -cp/mp];
B = [0, 0; 0, 0; 0, 0; kp/mp, cp/mp];
C = [1, 0, 0, 0];
D = 0;
%%Sine
sys = ss(A,B,C,D);
dsys = c2d(sys, h1);
X = [0;0;0;0];
for iter = 1:(length(t1)-1)
    zw = u_sin(iter);
    zw_dot = (u_sin(iter+1) - u_sin(iter))/h1;
    X(:,iter+1) = dsys.A*X(:,iter) + dsys.B*[zw;zw_dot];
end
figure,
plot(t1,u_sin,'r')
hold on
plot(t1,X(1,:),'b')

clear X
%%Impulse
dsys = c2d(sys, h2);
X = [0;0;0;0];
for iter = 1:(length(t2)-1)
    zw = u_impulse(iter);
    zw_dot = (u_impulse(iter+1) - u_impulse(iter))/h2;
    X(:,iter+1) = dsys.A*X(:,iter) + dsys.B*[zw;zw_dot];
end
figure,
plot(t2,u_impulse,'r')
hold on
plot(t2,X(1,:),'b')

