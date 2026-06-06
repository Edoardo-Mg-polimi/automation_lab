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

figure
bode(G_i)
grid on
title('Diagramma di Bode di G_i')

figure
nyquist(G_i)
grid on
title('Diagramma di Nyquist di G_i')

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
G_x = tf(SS_p);
[G_p_num, G_p_den] = tfdata(G_x, 'v');
disp('linearized model tf')
G_x
pole(G_x)
zero(G_x)

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

