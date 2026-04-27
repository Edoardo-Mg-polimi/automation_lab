clear
clc
%% 0) Parametri 
Kg = 3; % gain amplificatore

% Circuito LR
L = 412.5e-3; %[H] inductance of the coil
Rc = 10; %[ohm]
% nc = 2450; %number of coils
% lc = 0.0825; %[m] lenght of the coil
% rc = 0.008; %[m] coil radius
Rs = 1; %[ohm] current sense resistance
Rcal = 0.85; % calibration resistance gain

R = (Rc + Rs)* Rcal; %[ohm] total resistance
omega_coil = R/L;

% Meccanica
km = 6.5308e-5; %[N-m^2/A^2] electromagnet force constant
r = 1.27e-2; %[m] steel ball radius
m = 0.068; %[kg] steel ball mass
%Tb = 0.014; %[m] steel ball travel
l = 0.0394; %[m] total lenght
g = 9.81; %[m/s^2] gravity
%mu_0 = pi*4e-7; %[H/m] magnetic permeability constant
Kb = 2.83e-3; %[m/V] ball position sensitivity
s = tf("s");
I = [1, 0; 0, 1];

%% 1) Modello Circuito RL

%linear, SISO, Strictly proper, A.S., order 1
A_i = -R/L; 
B_i = 1/L; %system is reachable
C_i = 1; %system is observable 
D_i = 0; 

SS_i = ss(A_i, B_i, C_i, D_i);
G_i = Kg*tf(SS_i);
[G_i_num, G_i_den] = tfdata(G_i, 'v');
G_i
pole(G_i)
zero(G_i)

%figure
%bode(G_i)
%grid on
%title('Diagramma di Bode di G_i')

%figure
%nyquist(G_i)
%grid on
%title('Diagramma di Nyquist di G_i')

%% 2) Modello Forze

%equilibrium
x2_e = 0; 
x1_e = 0.007; %eq position "middle of the airgap"
u_e = (((g*m)/km)^(1/2))*(l - x1_e - (2*r));
x_e = [x1_e, x2_e]; %equilibrium states vector

%u_e = ;
%x1_e = - ((km/(g*m))^(1/2)) - (2*r) + l;

%Linearized model, SISO, Strictly proper, Unstable, order 2
A_p = [0, 1; (2*km*(u_e^2))/(m*((l - x1_e - (2*r)))^3) 0]; 
B_p = [0; (2*km*u_e)/(m*((l - x1_e - (2*r)))^2)];
C_p = [1, 0];
D_p = 0;

SS_p = ss(A_p, B_p, C_p, D_p);
G_p = tf(SS_p);
[G_p_num, G_p_den] = tfdata(G_p, 'v');
disp('linearized model tf')
G_p
pole(G_p)
zero(G_p)

%checking reachability of linearized system
Mr = ctrb(A_p,B_p);
if (rank(Mr) == 2)
    disp('The linearized model is reachable')
else 
    disp('The linearized model is not reachable');
end

%checking observability of linearized system
Mo = obsv(A_p,C_p);
if (rank(Mo) == 2)
    disp('The linearized model is observable')
else 
    disp('The linearized model is not observable');
end

%% 3) Regolatore Corrente PID

p = pole(G_p); % poles of the position plant
wm = abs(p(1));
%wc = 2*wm;
wc = 180;
phase_m = 80; %margine di fase in deg
fprintf("Requirements: wc=%f , phase margin=%f \n", wc, phase_m);
phiG_deg = -atand((wc*L)/R);%fase a omega c
Ti = 1/(wc * tand(180 + phiG_deg -phase_m));
Kp_i = sqrt(R^2 + (wc*L)^2) / ( Kg * sqrt(1 + 1/(wc^2 * Ti^2)) );
Ki_i = Kp_i/Ti;
fprintf("Kp = %f\n", Kp_i);
fprintf("Ki = %f\n", Ki_i);

R_i = Kp_i + Ki_i/s; %current controller
L_i = G_i*R_i %current open loop tf
pole(L_i)
zero(L_i)
H_i = feedback(L_i, 1) %current closed loop tf
pole(H_i)
zero(H_i)
%bode(H_i)


%% 4) Regolatore Posizione PID

G_out = H_i*G_p %current closed loop tf * position plant
pole(G_out)
zero(G_out)
%bode(G_out)

% figure
% nyquist(G_out)
% hold on
% plot(-1, 0, 'rx', 'MarkerSize', 14, 'LineWidth', 2)
% title('Nyquist of G_out')
% grid on

omega_z = 52.9420; %controller parameters, manually inserted
omega_p = 100;    
K_lead  = 105;

R_p = tf(K_lead * [1/omega_z, 1], [1/omega_p , 1]) %position controller
[R_p_num, R_p_den] = tfdata(R_p, 'v');

L_p = R_p*G_out %position open loop tf
pole(L_p)
zero(L_p)

H_p = feedback(L_p, 1) %position closed loop tf
pole(H_p)
zero(H_p)
%bode(H_p)


figure
step(H_p)
hold on
plot(-1, 0, 'rx', 'MarkerSize', 14, 'LineWidth', 2)
title('')
grid on

figure
nyquist(L_p)
hold on
plot(-1, 0, 'rx', 'MarkerSize', 14, 'LineWidth', 2)
title('Nyquist of L_p')
grid on