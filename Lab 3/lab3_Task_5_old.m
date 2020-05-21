% clc
% clear all
% close all

m = 22000;
j = 700000;
c1 = 40000;
c2 = 40000;
k1 = 600000;
k2 = 600000;
l1 = 6;
l2 = 6;

%%Defining Input signals

h1 = 0.01;
f_1 = 1;
f_2 = 8;
t1 = 0:h1:5;
u_sin_1 = 0.01*sin(2*pi*f_1*t1);
u_sin_2 = 0.01*sin(2*pi*f_2*t1);

step_amp = 0.03;
h2 = 0.01;
t2 = 0:h2:50;
u_step = zeros(length(t2),1);
u_step(t2<1/step_amp) = step_amp;

impulse_amp = 0.03;
h3 = 0.01;
t3 = 0:h2:8;
u_impulse = zeros(length(t3),1);
u_impulse(100) = impulse_amp;


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
sys_passive = ss(A,B,C,D);
naturalFreq = eig(A);
[wn,zeta] = damp(sys_passive);
% 
% %%Generate response for passive system
% X_passive_sin_1 = genRespPassive(sys_passive,u_sin_1,h1);
% X_passive_sin_2 = genRespPassive(sys_passive,u_sin_2,h1);
% X_passive_step = genRespPassive(sys_passive,u_step,h2);
% X_passive_impulse = genRespPassive(sys_passive,u_impulse,h3);
% 
% subplot(2,2,1),
% plot(t1,u_sin_1,'r');
% hold on
% plot(t1,X_passive_sin_1(1,:),'b')
% plot(t1,X_passive_sin_1(2,:),'g')
% grid on
% 
% subplot(2,2,2),
% plot(t1,u_sin_2,'r');
% hold on
% plot(t1,X_passive_sin_2(1,:),'b')
% plot(t1,X_passive_sin_2(2,:),'g')
% grid on
% 
% subplot(2,2,3),
% plot(t2,u_step,'r');
% hold on
% plot(t2,X_passive_step(1,:),'b')
% plot(t2,X_passive_step(2,:),'g')
% grid on
% 
% subplot(2,2,4),
% plot(t3,u_impulse,'r');
% hold on
% plot(t3,X_passive_impulse(1,:),'b')
% plot(t3,X_passive_impulse(2,:),'g')
% grid on


%% Skyhook
%Feedback based
cz = 1e5;
cx = 3e6;

Ash = [0, 1, 0, 0;
    -(k1+k2)/m, 0, (k1*l1 - k2*l2)/m, 0;
    0, 0, 0, 1;
    (k1*l1 - k2*l2)/j, 0, -(k1*l1*l1 + k2*l2*l2)/j, 0];
Bsh = [0,0,0,0;
    k1/m, k2/m, -1/m, -1/m;
    0,0,0,0;
    -k1*l1/j, k2*l2/j, l1/j, -l2/j];
Csh = [1,0, 0, 0;
    0, 1, 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1];
Dsh = zeros(4,4);

fb_tf = [0, cz*l2/(l1+l2),0, -cx/(l1+l2);
    0, cz*l1/(l1+l2), 0, cx/(l1+l2)];

%%
% sys = ss(Ash,Bsh,Csh,Dsh);
% dsys = c2d(sys, h1);
% X = [0;0;0;0];
% for iter = 1:(length(t1)-1)
%     zw1 = u_sin(iter);
%     zw2 = 0;
%     Fa1 = -cx*X(4,iter)/(l1+l2) - cz*l2*X(2,iter)/(l1+l2);
%     Fa2 = cx*X(4,iter)/(l1+l2) - cz*l1*X(2,iter)/(l1+l2);
%     X(:,iter+1) = dsys.A*X(:,iter) + dsys.B*[zw1;zw2;Fa1;Fa2];
% end
% figure,
% plot(t1,u_sin,'g')
% % Ask=[0 1 0 0
% %     -2*k/m 0 0 0
% %     0 0 0 1
% %     0 0 -2*k*L^2/j 0];
% % Bsk=[0 0 0 0
% %     k/m k/m -1/m -1/m
% %     0 0 0 0
% %     -L*k/j L*k/j L/j -L/j];
% % Csk=[0 1 0 0
% %     0 0 0 1];
% % Dsk=zeros(2,4);
% 
% sys_active = ss(Ash,Bsh,Csh,Dsh);
% %%Generate response for passive system
% X_active_sin_1 = genRespActive(sys_active,u_sin_1,h1,cx,cz);
% X_active_sin_2 = genRespActive(sys_active,u_sin_2,h1,cx,cz);
% X_active_step = genRespActive(sys_active,u_step,h2,cx,cz);
% X_active_impulse = genRespActive(sys_active,u_impulse,h3,cx,cz);
% 
% subplot(2,2,1),
% plot(t1,u_sin_1,'r');
% hold on
% plot(t1,X_active_sin_1(1,:),'b')
% plot(t1,X_active_sin_1(2,:),'g')
% grid on
% 
% subplot(2,2,2),
% plot(t1,u_sin_2,'r');
% hold on
% plot(t1,X_active_sin_2(1,:),'b')
% plot(t1,X_active_sin_2(2,:),'g')
% grid on
% 
% subplot(2,2,3),
% plot(t2,u_step,'r');
% hold on
% plot(t2,X_active_step(1,:),'b')
% plot(t2,X_active_step(2,:),'g')
% grid on
% 
% subplot(2,2,4),
% plot(t3,u_impulse,'r');
% hold on
% plot(t3,X_active_impulse(1,:),'b')
% plot(t3,X_active_impulse(2,:),'g')
% grid on
