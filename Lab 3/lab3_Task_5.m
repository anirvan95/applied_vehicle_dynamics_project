clc
clear all
close all

m = 22000;
j = 700000;
c1 = 40000;
c2 = 40000;
k1 = 600000;
k2 = 600000;

l1 = 6;
l2 = 6;

h1 = 0.001;
t1 = 0:h1:2*pi;
omega = 1;
u_sin = 0.05*sin(omega*t1);


impulse_amp = 0.05;
h2 = 0.001;
t2 = 0:h2:40;
u_impulse = zeros(length(t2),1);
u_impulse(10) = impulse_amp;

A = [0, 1, 0, 0;
    -(k1+k2)/m,  -(c1+c2)/m, (k1*l1 - k2*l2)/m, (c1*l1 - c2*l2)/m;
    0, 0, 0, 1;
    (k1*l1 - k2*l2)/j, (c1*l1 - c2*l2)/j, -(k1*l1*l1 + k2*l2*l2)/j, -(c1*l1*l1 + c2*l2*l2)/j];

B = [0,0,0,0;
    k1/m, c1/m, k2/m, c2/m;
    0,0,0,0;
    -k1*l1/j, -c1*l1/j, k2*l2/j, c2*l2/j];

C = [1, 0, 0, 0;
    0, 0, 1, 0];

D = zeros(2,4);
sys = ss(A,B,C,D);
naturalFreq = eig(A);
[wn,zeta] = damp(sys);

dsys = c2d(sys, h1);
X = [0;0;0;0];
for iter = 1:(length(t1)-1)
    zw1 = u_sin(iter);
    zw1_dot = (u_sin(iter+1) - u_sin(iter))/h1;
    zw2 = 0;
    zw2_dot = 0;
    X(:,iter+1) = dsys.A*X(:,iter) + dsys.B*[zw1;zw1_dot;zw2;zw2_dot];
end
figure,
plot(t1,u_sin,'g')
hold on
plot(t1,X(1,:),'r')
plot(t1,X(3,:),'b')


%%Skyhook
%
%
%  Ash = [0, 1, 0, 0;
%         -(k1+k2)/m, cz/m, -(-k1*l1 + k2*l2)/m, 0;
%         0, 0, 0, 1;
%         -(-k1*l1 + k2*l2)/j, 0, -(k1*l1*l1 + k2*l2*l2)/j, -cx/j];
%  Bsh = [0,0;
%         k1/m, k2/m;
%         0,0;
%         -k1*l1/j, k2*l2/j];
%
%  Csh = [0, 1, 0, 0;
%         0, 0, 0, 1];
%
%  Dsh = zeros(2,2);

% syms cz zdot xdot Fa1 Fa2 cx l1 l2
% 
% eqns = [Fa1+Fa2 == -cz*zdot;Fa1*l1-Fa2*l2==-cx*xdot];
% sln = solve(eqns,[Fa1,Fa2])

%Feedback based
cz = 1000;
cx = 1000;
Ash = [0, 1, 0, 0;
    -(k1+k2)/m, 0, (k1*l1 - k2*l2)/m, 0;
    0, 0, 0, 1;
    (k1*l1 - k2*l2)/j, 0, -(k1*l1*l1 + k2*l2*l2)/j, 0];
Bsh = [0,0,0,0;
    k1/m, k2/m, -1/m, -1/m;
    0,0,0,0;
    -k1*l1/j, k2*l2/j, l1/j, -l2/j];

Csh = [0, 1, 0, 0;
    0, 0, 0, 1];

Dsh = zeros(2,4);

sys = ss(Ash,Bsh,Csh,Dsh);
dsys = c2d(sys, h1);
X = [0;0;0;0];
for iter = 1:(length(t1)-1)
    zw1 = u_sin(iter);
    zw2 = 0;
    Fa1 = -cx*X(4,iter)/(l1+l2) - cz*l2*X(2,iter)/(l1+l2);
    Fa2 = cx*X(4,iter)/(l1+l2) - cz*l1*X(2,iter)/(l1+l2);
    X(:,iter+1) = dsys.A*X(:,iter) + dsys.B*[zw1;zw2;Fa1;Fa2];
end
figure,
plot(t1,u_sin,'g')
hold on
plot(t1,X(1,:),'r')
plot(t1,X(3,:),'b')
