clc
clear all
close all
s=tf('s');

% system parameters
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


%% State space model for skyhook contorl
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

%% H_inf using linmod syntax

%state space: The same as skyhook
sys = ss(Ask,Bsk,Csk,Dsk);
[wn,zeta] = damp(sys);

%%Task 10.1
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

figure,
bodemag(Wb)
hold on
bodemag(Wchi)
bodemag(Wa1)
legend('W_b','W_\chi','W_a1');
grid on

%%Analysis Weighing function
% kb_vect = [100;1000;1e4;1e5];
% Wb1=(kb_vect(1)*s1b*s2b)/((s-s1b)*(s-s2b));
% Wb2=(kb_vect(2)*s1b*s2b)/((s-s1b)*(s-s2b));
% Wb3=(kb_vect(3)*s1b*s2b)/((s-s1b)*(s-s2b));
% Wb4=(kb_vect(4)*s1b*s2b)/((s-s1b)*(s-s2b));
% figure,
% bode(Wb1,'r')
% hold on
% bode(Wb2,'g')
% bode(Wb3,'b')
% bode(Wb4,'k')
% legend('kb=100','kb=1e3','kb=1e4','kb=1e5')
% 
% kchi_vect = [100;1000;1e4;1e5];
% Wchi1=(kchi_vect(1)*s1chi*s2chi)/((s-s1chi)*(s-s2chi));
% Wchi2=(kchi_vect(2)*s1chi*s2chi)/((s-s1chi)*(s-s2chi));
% Wchi3=(kchi_vect(3)*s1chi*s2chi)/((s-s1chi)*(s-s2chi));
% Wchi4=(kchi_vect(4)*s1chi*s2chi)/((s-s1chi)*(s-s2chi));
% figure,
% bode(Wchi1,'r')
% hold on
% bode(Wchi2,'g')
% bode(Wchi3,'b')
% bode(Wchi4,'k')
% legend('kchi=100','kchi=1e3','kchi=1e4','kchi=1e5')

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

%updating Csk and Dsk to generate all the outputs
Csk=eye(4);
Dsk=zeros(4,4);
%Now use the controller K in your simulation

%%Select signal in Simulation 
%%Task 10.1
data = sim('lab3_vehicle_H_inf_sim.slx');

figure,
subplot(2,2,1),
plot(data.input_singal,'k');
xlabel('Time (sec)');
ylabel('Amplitude Z_{w1} (m)');
grid on

subplot(2,2,2)
plot(data.passive_out.Time, data.passive_out.Data(:,1),'b','Linewidth',1)
hold on
plot(data.hinf_out.Time, data.hinf_out.Data(:,1),'r','Linewidth',1.5)
xlabel('Time (sec)');
ylabel('Amplitude Z (m)');
legend('Damped System','H_{\infty} System');
grid on

subplot(2,2,3)
plot(data.passive_out.Time, data.passive_out.Data(:,3),'g','Linewidth',1)
hold on
plot(data.hinf_out.Time, data.hinf_out.Data(:,3),'m','Linewidth',1.5)
xlabel('Time (sec)');
ylabel('Amplitude \chi (m)');
legend('Damped System','H_{\infty} System');
grid on

subplot(2,2,4)
plot(data.hinf_force_output.Time, data.hinf_force_output.Data(:,1),'b','Linewidth',1)
hold on
plot(data.hinf_force_output.Time, data.hinf_force_output.Data(:,2),'r','Linewidth',1)
xlabel('Time (sec)');
ylabel('Amplitude F (N)');
legend('Fa_1','Fa_2');
grid on

sgtitle('Sinusoidal 1 Hz response of vehicle model');


