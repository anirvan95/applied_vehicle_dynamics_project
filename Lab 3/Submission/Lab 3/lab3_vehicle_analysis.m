clc
close all
clear all

s=tf('s');

% systme parameters
m=22000;   %kg
j=700e3;   %kgm^2
c=40e3;    %Ns/m
k=2*300e3; %N/m
L=6;       %m

%% State space model for passive system

k1 = k;
k2 = k;
c1 = c;
c2 = c;
l1 = L;
l2 = L;

Apass = [0, 1, 0, 0;
    -(k1+k2)/m,  -(c1+c2)/m, (k1*l1 - k2*l2)/m, (c1*l1 - c2*l2)/m;
    0, 0, 0, 1;
    (k1*l1 - k2*l2)/j, (c1*l1 - c2*l2)/j, -(k1*l1*l1 + k2*l2*l2)/j, -(c1*l1*l1 + c2*l2*l2)/j];

Bpass = [0,0,0,0;
    k1/m, c1/m, k2/m, c2/m;
    0,0,0,0;
    -k1*l1/j, -c1*l1/j, k2*l2/j, c2*l2/j];

Cpass = [0,1,0,0;0,0,0,1];

Dpass = zeros(2,4);

sys_pass = ss(Apass, Bpass, Cpass, Dpass);
sigma(sys_pass)

%% State space model for skyhook control
Ask=[0 1 0 0
    -2*k/m 0 0 0
    0 0 0 1
    0 0 -2*k*L^2/j 0];
Bsk=[0 0 0 0
    k/m k/m -1/m -1/m
    0 0 0 0
    -L*k/j L*k/j L/j -L/j];
Csk=[0 1 0 0
    0 0 0 1];
Dsk=zeros(2,4);


sys_active = ss(Ask,Bsk,Csk,Dsk);

sigma(sys_pass,'r')
hold on
sigma(sys_active,'b')
legend('Passive','Active');

[wn,zeta] = damp(sys_active);

%% TF wrt z_w1
[zw1b,zw1a] = ss2tf(Ask,Bsk,Csk,Dsk,1);
tf_z_zw1 = tf(zw1b(1,:),zw1a);
tf_chi_zw1 = tf(zw1b(2,:),zw1a);

[zw2b,zw2a] = ss2tf(Ask,Bsk,Csk,Dsk,2);
tf_z_zw2 = tf(zw2b(1,:),zw2a);
tf_chi_zw2 = tf(zw2b(2,:),zw2a);

[fa1b,fa1a] = ss2tf(Ask,Bsk,Csk,Dsk,3);
tf_z_fa1 = tf(fa1b(1,:),fa1a);
tf_chi_fa1 = tf(fa1b(2,:),fa1a);

[fa2b,fa2a] = ss2tf(Ask,Bsk,Csk,Dsk,4);
tf_z_fa2 = tf(fa2b(1,:),fa2a);
tf_chi_fa2 = tf(fa2b(2,:),fa2a);

figure,
subplot(2,2,1),bode(tf_z_zw1);
title('TF z_{w1} to z')
grid on
subplot(2,2,2),bode(tf_z_zw2);
title('TF z_{w2} to z')
grid on
subplot(2,2,3),bode(tf_z_fa1);
title('TF Fa_1 to z')
grid on
subplot(2,2,4),bode(tf_z_fa2);
title('TF Fa_2 to z')
grid on
sgtitle('Bode plots of z in MIMO vehicle model');

figure,
subplot(2,2,1),bode(tf_chi_zw1);
title('TF z_{w1} to \chi')
grid on
subplot(2,2,2),bode(tf_chi_zw2);
title('TF z_{w2} to \chi')
grid on
subplot(2,2,3),bode(tf_chi_fa1);
title('TF Fa_1 to \chi')
grid on
subplot(2,2,4),bode(tf_chi_fa2);
title('TF Fa_2 to \chi')
grid on
sgtitle('Bode plots of \chi in MIMO vehicle model');


