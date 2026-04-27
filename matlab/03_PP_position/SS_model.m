% State-space models for current and position dynamics
clear all;
clc
fprintf('\n');
fprintf('============================================================\n');
fprintf(' State Space model\n');
fprintf('============================================================\n');
%% 0) Parametri 
Kg = 3;% gain amplificatore

% Circuito LR
L = 412.5e-3; %[H] inductance of the coil
Rc = 10; %[ohm]
% nc = 2450; %number of coils
% lc = 0.0825; %[m] lenght of the coil
% rc = 0.008; %[m] coil radius
Rs = 1; %[ohm] current sense resistance
Rcal = 0.85; % calibration resistance gain

R = (Rc + Rs)* Rcal; %[ohm] total resistance
V_max = 24; %tensione massima in ingresso alla bobina
I_max = 3; % 3 A massimi in ingresso alla bobina


% Meccanica
km = 6.5308e-5; %[N-m^2/A^2] electromagnet force constant
r = 1.27e-2; %[m] steel ball radius
m = 0.068; %[kg] steel ball mass
%Tb = 0.014; %[m] steel ball travel
l = 0.0394; %[m] total lenght
g = 9.81; %[m/s^2] gravity
%mu_0 = pi*4e-7; %[H/m] magnetic permeability constant
Kb = 2.83e-3; %[m/V] ball position sensitivity
%% 1) Modello Circuito RL
% Linear, SISO, ordine 1
A_i = -R/L;
B_i = 1/L;
C_i = 1;
D_i = 0;

SS_i = ss(A_i, B_i, C_i, D_i)

fprintf(' 1) MODELLO RL IN SPAZIO DI STATO\n');
fprintf('Autovalore RL: %f\n', eig(A_i));

% Verifica raggiungibilita RL
Mr_i = ctrb(A_i, B_i);
if rank(Mr_i) == 1
    disp('Il modello RL e'' raggiungibile')
else
    disp('Il modello RL non e'' raggiungibile')
end

% Verifica osservabilita RL
Mo_i = obsv(A_i, C_i);
if rank(Mo_i) == 1
    disp('Il modello RL e'' osservabile')
else
    disp('Il modello RL non e'' osservabile')
end

%% 2) Modello meccanico linearizzato
fprintf(' 2) MODELLO MECCANICO LINEARIZZATO\n');
%Equilibrium
x_e = l/2 - r; % posizione di equilibrio a metà (piedistallo - inizio palla)
xd_e = 0;
i_e = sqrt((m*g/km) * (x_e + 2*r -l)^2); % corrente di equilibrio ie(xe)

v_e = i_e * R;

k1 = (2 * km * i_e^2)/(m * (l -2*r -x_e)^3);
k2 = (2 * km * i_e)/(m * (l-2*r -x_e)^2);

%Linearized model, SISO, Strictly proper, Unstable, order 2
A_p = [0, 1;
       k1, 0]; 
B_p = [0; k2];
C_p = [1, 0];
D_p = 0;

SS_p = ss(A_p, B_p, C_p, D_p);
SS_p
disp('Autovalori modello posizione:')
disp(eig(A_p))

% 2.1) checking reachability of linearized system
Mr = ctrb(A_p,B_p);
if (rank(Mr) == 2)
    disp('The linearized model is reachable')
else 
    disp('The linearized model is not reachable');
end

% 2.2)checking observability of linearized system
Mo = obsv(A_p,C_p);
if (rank(Mo) == 2)
    disp('The linearized model is observable')
else 
    disp('The linearized model is not observable');
end
