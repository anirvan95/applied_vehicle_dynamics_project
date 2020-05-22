%%File for model uncertainity and complete vehicle model simulation
clear all
s=tf('s');

% systme parameters
m=22000;   %kg
j=700e3;   %kgm^2
c=40e3;    %Ns/m
k=2*300e3; %N/m
L=6;       %m

k1 = k;
k2 = k;
c1 = c;
c2 = c;
l1 = L;
l2 = L;

%% State space for passive vehicle model
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

%% Skyhook controller
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


%% H_inf controller
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

sys = ss(Ask,Bsk,Csk,Dsk);
[wn,zeta] = damp(sys);

%Weighting functions

%For penalizing actuator force
Wa1=(0.00175*s+1)/(0.00025*s+1);
Wa2=Wa1;

%For penalizing bounce and pitch motions
eps=1;
wnb=wn(1);           %Find the right equation or value for wnb
wnchi=wn(3);          %Find the right equation or value for wnchi
s1b=-eps+1i*sqrt(wnb^2-eps^2);
s2b=-eps-1i*sqrt(wnb^2-eps^2);
s1chi=-eps+1i*sqrt(wnchi^2-eps^2);
s2chi=-eps-1i*sqrt(wnchi^2-eps^2);
kb=2.055e3;%input('Enter the gain for Wb = '); 
kchi=4.1404e4;%input('Enter the gain for Wchi = ');
Wb=(kb*s1b*s2b)/((s-s1b)*(s-s2b));
Wchi=(kchi*s1chi*s2chi)/((s-s1chi)*(s-s2chi));


%Extracting the extended model
[A_Pe,B_Pe,C_Pe,D_Pe] = linmod('Extended_model');% state space parameters of the extended system: Pe
Pe=ss(A_Pe,B_Pe,C_Pe,D_Pe);

%Calculating the controller
ncont = 2;%Number of control inputs
nmeas = 2;%Number of measured outputs provided to the controller
Pe=minreal(Pe);%This syntax cancels pole-zero pairs in transfer
%functions. The output system has minimal order and the same response
%characteristics as the original model.
[K,Pec,gamma,info]=hinfsyn(Pe,nmeas,ncont,'method','lmi'); % for working with the error
[Ainf, Binf, Cinf, Dinf]=ssdata(K);

%updating Csk and Dsk to generate outputs
Csk=eye(4);
Dsk=zeros(4,4);

%% Compare performance of passive, Skyhook and Hinf
%Task 11.1
data = sim('lab3_vehicle_complete_sim.slx');

figure,
subplot(2,2,1),
plot(data.input_singal,'k');
xlabel('Time (sec)');
ylabel('Amplitude Z_{w1} (m)');
grid on

subplot(2,2,2)
plot(data.passive_out.Time, data.passive_out.Data(:,1),'g','Linewidth',1.5)
hold on
plot(data.skyhook_out.Time, data.skyhook_out.Data(:,1),'r','Linewidth',1.5)
plot(data.hinf_out.Time, data.hinf_out.Data(:,1),'b','Linewidth',1.0)
xlabel('Time (sec)');
ylabel('Amplitude Z (m)');
legend('Passive','Skyhook','H_{\infty}');
grid on

subplot(2,2,3)
plot(data.passive_out.Time, data.passive_out.Data(:,3),'g','Linewidth',1.0)
hold on
plot(data.skyhook_out.Time, data.skyhook_out.Data(:,3),'r','Linewidth',1.0)
plot(data.hinf_out.Time, data.hinf_out.Data(:,3),'b','Linewidth',1.0)
xlabel('Time (sec)');
ylabel('Amplitude \chi (m)');
legend('Passive','Skyhook','H_{\infty}');
grid on

subplot(2,2,4)
plot(data.skyhook_force_output.Time, data.skyhook_force_output.Data(:,1),'-.r','Linewidth',1.5)
hold on
plot(data.hinf_force_output.Time, data.hinf_force_output.Data(:,1),'m','Linewidth',1.5)
plot(data.skyhook_force_output.Time, data.skyhook_force_output.Data(:,2),'-.b','Linewidth',1)
plot(data.hinf_force_output.Time, data.hinf_force_output.Data(:,2),'c','Linewidth',1)

xlabel('Time (sec)');
ylabel('Amplitude F (N)');
legend('Fa_1 Skyhook','Fa_1 H_{\infty}','Fa_2 Skyhook','Fa_2 H_{\infty}');
grid on

sgtitle('Sinusoidal 1 Hz response of vehicle model');



%% Study of model uncertainity
%Task 10.3
m_inc = m*1.15;
m_dec = m*0.85;

[Ash, Bsh, Ask, Bsk] = getUpdatedModel(m_inc,j,k,L);
data_inc = sim('lab3_vehicle_complete_sim.slx'); %%Change input to impulse in sim file

[Ash, Bsh, Ask, Bsk] = getUpdatedModel(m_dec,j,k,L);
data_dec = sim('lab3_vehicle_complete_sim.slx'); %%Change input to impulse in sim file

figure,
subplot(2,2,1),
plot(data_inc.input_singal,'k');
xlabel('Time (sec)');
ylabel('Amplitude Z_{w1} (m)');
grid on

subplot(2,2,2)
plot(data_inc.skyhook_out.Time, data_inc.skyhook_out.Data(:,1),'-.r','Linewidth',1.5)
hold on
plot(data_inc.hinf_out.Time, data_inc.hinf_out.Data(:,1),'-.b','Linewidth',1.5)
plot(data_dec.skyhook_out.Time, data_dec.skyhook_out.Data(:,1),'m','Linewidth',1.0)
plot(data_dec.hinf_out.Time, data_dec.hinf_out.Data(:,1),'c','Linewidth',1.0)
xlabel('Time (sec)');
ylabel('Amplitude Z (m)');
legend('Skyhook +15%','H_{\infty} +15%','Skyhook -15%','H_{\infty} -15%');
grid on

subplot(2,2,3)
plot(data_inc.skyhook_out.Time, data_inc.skyhook_out.Data(:,3),'-.r','Linewidth',1.0)
hold on
plot(data_inc.hinf_out.Time, data_inc.hinf_out.Data(:,3),'-.b','Linewidth',1.0)
plot(data_dec.skyhook_out.Time, data_dec.skyhook_out.Data(:,3),'m','Linewidth',1.0)
plot(data_dec.hinf_out.Time, data_dec.hinf_out.Data(:,3),'c','Linewidth',1.0)
xlabel('Time (sec)');
ylabel('Amplitude \chi (m)');
legend('Skyhook +15%','H_{\infty} +15%','Skyhook -15%','H_{\infty} -15%');
grid on

subplot(2,2,4)
plot(data_inc.skyhook_force_output.Time, data_inc.skyhook_force_output.Data(:,1),'-.r','Linewidth',1.5)
hold on
plot(data_inc.hinf_force_output.Time, data_inc.hinf_force_output.Data(:,1),'-.b','Linewidth',1.5)
plot(data_dec.skyhook_force_output.Time, data_dec.skyhook_force_output.Data(:,1),'m','Linewidth',1)
plot(data_dec.skyhook_force_output.Time, data_dec.skyhook_force_output.Data(:,1),'c','Linewidth',1)

xlabel('Time (sec)');
ylabel('Amplitude F (N)');
legend('Fa_1 Skyhook +15%','Fa_1 H_{\infty} +15%','Fa_1 Skyhook -15%','Fa_1 H_{\infty} -15%');
grid on

sgtitle('Mass uncertainity response of vehicle model');

