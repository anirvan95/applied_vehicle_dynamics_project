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

%%Task 8.2
Apass = [0, 1, 0, 0;
    -(k1+k2)/m,  -(c1+c2)/m, (k1*l1 - k2*l2)/m, (c1*l1 - c2*l2)/m;
    0, 0, 0, 1;
    (k1*l1 - k2*l2)/j, (c1*l1 - c2*l2)/j, -(k1*l1*l1 + k2*l2*l2)/j, -(c1*l1*l1 + c2*l2*l2)/j];

Bpass = [0,0,0,0;
    k1/m, c1/m, k2/m, c2/m;
    0,0,0,0;
    -k1*l1/j, -c1*l1/j, k2*l2/j, c2*l2/j];

Cpass = eye(4);

Dpass = zeros(4,4);
sys_passive = ss(Apass,Bpass,Cpass,Dpass);
poles = eig(Apass);
[wn,zeta] = damp(sys_passive);

%%Task 9.3
%% Skyhook
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

Csh = eye(4);

Dsh = zeros(4,4);

fb_tf = [0, cz*l2/(l1+l2),0, -cx/(l1+l2);
    0, cz*l1/(l1+l2), 0, cx/(l1+l2)];

%%Select signal in Simulation 

data = sim('lab3_vehicle_skyhook_sim.slx');

figure,
subplot(2,2,1),
plot(data.input_singal,'k');
xlabel('Time (sec)');
ylabel('Amplitude Z_{w1} (m)');
grid on

subplot(2,2,2)
plot(data.passive_out.Time, data.passive_out.Data(:,1),'b','Linewidth',1)
hold on
plot(data.skyhook_out.Time, data.skyhook_out.Data(:,1),'r','Linewidth',1.5)
xlabel('Time (sec)');
ylabel('Amplitude Z (m)');
legend('Damped System','Skyhook System');
grid on

subplot(2,2,3)
plot(data.passive_out.Time, data.passive_out.Data(:,3),'g','Linewidth',1)
hold on
plot(data.skyhook_out.Time, data.skyhook_out.Data(:,3),'m','Linewidth',1.5)
xlabel('Time (sec)');
ylabel('Amplitude \chi (m)');
legend('Damped System','Skyhook System');
grid on

subplot(2,2,4)
plot(data.skyhook_force_output.Time, data.skyhook_force_output.Data(:,1),'b','Linewidth',1)
hold on
plot(data.skyhook_force_output.Time, data.skyhook_force_output.Data(:,2),'r','Linewidth',1)
xlabel('Time (sec)');
ylabel('Amplitude F (N)');
legend('Fa_1','Fa_2');
grid on

sgtitle('Sinusoidal (1 Hz) response of vehicle model');





