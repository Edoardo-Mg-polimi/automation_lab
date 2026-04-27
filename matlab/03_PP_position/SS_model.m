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
fprintf(' 1) MODELLO CIRCUITO RL\n');
s = tf('s');
wp = R/L; % unico polo di G_i(s)
G_i = 1/(s*L +R)
[G_i_num, G_i_den] = tfdata(G_i, 'v');

% Rappresentazione in spazio di stato del RL (solo per verifiche)
A_i = -R/L;
B_i = 1/L;
C_i = 1;
D_i = 0;

Mr_i = ctrb(A_i, B_i);
if rank(Mr_i) == 1
    disp('Il circuito RL e'' controllabile')
else
    disp('Il circuito RL non e'' controllabile')
end

Mo_i = obsv(A_i, C_i);
if rank(Mo_i) == 1
    disp('Il circuito RL e'' osservabile')
else
    disp('Il circuito RL non e'' osservabile')
end

fprintf("amplificatore: Kg=%f\n", Kg);

% 1.2) Regolatore RL sintetizzato in frequenza
fprintf(' 1.2) REGOLATORE DI CORRENTE R_i(s)\n');
Kp_i = 14.559055;
Ki_i = 330.005238;
fprintf("Kp_i = %f\n", Kp_i);
fprintf("Ki_i = %f\n", Ki_i);

R_i = Kp_i + Ki_i/s;
[R_i_num, R_i_den] = tfdata(R_i, 'v');

% 1.3) Anello aperto di corrente
fprintf(' 1.3) ANELLO APERTO CORRENTE L_i(s)\n');
L_i = R_i * Kg * G_i;
[numLi, denLi] = tfdata(L_i, 'v');
poles_L_i = pole(L_i);
zeros_L_i = zero(L_i);
[gm, pm, wcg, wcp_i] = margin(L_i);
fprintf("Gain margin=%f ,Phase margin=%f ,w_c=%f \n", gm, pm, wcp_i);

% 1.4) Anello chiuso di corrente
fprintf(' 1.4) ANELLO CHIUSO CORRENTE H_i(s)\n');
H_i = minreal(L_i/(1+L_i))
poles_H_i = pole(H_i);

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
A_x = [0, 1;
       k1, 0]; 
B_x = [0; k2];
C_x = [1, 0];
D_x = 0;

E_x = eye(2);

SS_p = ss(A_x, B_x, C_x, D_x);
SS_p
disp('Autovalori modello posizione:')
disp(eig(A_x))

% 2.1) checking reachability of linearized system
Mr = ctrb(A_x,B_x);
if (rank(Mr) == 2)
    disp('The linearized model is reachable')
else 
    disp('The linearized model is not reachable');
end

% 2.2)checking observability of linearized system
Mo = obsv(A_x,C_x);
if (rank(Mo) == 2)
    disp('The linearized model is observable')
else 
    disp('The linearized model is not observable');
end

%% 3) Luenberger Observer con Ackermann
fprintf(' 3) LUENBERGER OBSERVER - ACKERMANN\n');
% Poli desiderati dell'osservatore
p_obs = [-150 -180];

% Guadagno osservatore con formula di Ackermann
L_obs = acker(A_x', C_x', p_obs)';

fprintf('\nGuadagno osservatore L_obs:\n');
disp(L_obs);

% Verifica poli dell'errore di stima
eig_obs = eig(A_x - L_obs*C_x);

fprintf('Poli osservatore:\n');
disp(eig_obs);

% Modello osservatore
A_obs = A_x - L_obs*C_x;
B_obs = [B_x L_obs];
C_obs = eye(2);
D_obs = zeros(2,2);

SS_obs = ss(A_obs, B_obs, C_obs, D_obs);

fprintf('\nModello state-space osservatore:\n');
SS_obs
