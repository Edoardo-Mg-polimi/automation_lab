% State-space models for current and position dynamics
clear all;
clc
fprintf('\n');
fprintf('============================================================\n');
fprintf(' State Space model - LQ control\n');
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
Kp_i = 72.795273;
Ki_i = 1650.026190;
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

%% 3) Kalman Filter per stima dello stato
fprintf(' 3) KALMAN FILTER - LQE\n');

% Modello:
%   x_dot = A_x*x + B_x*u + G_kf*w
%   y     = C_x*x + v
%
% Stato:
%   x = [delta_posizione; delta_velocita]
%
% Ingresso:
%   u = delta_i, variazione di corrente rispetto all'equilibrio
%
% Misura:
%   y = delta_posizione

% Disturbo di processo: lo modello come disturbo sull'accelerazione
G_kf = [0; 1];

% Covarianza rumore di processo
% Valore iniziale da tarare: più è grande, più il filtro segue la misura
sigma_a = 5;               % [m/s^2], incertezza equivalente sull'accelerazione
Q_kf = sigma_a^2;

% Covarianza rumore di misura
% Se y è in metri, questo è il rumore stimato del sensore di posizione
sigma_y = 0.10e-3;         % [m], es. 0.10 mm
R_kf = sigma_y^2;

% Guadagno di Kalman continuo
[L_kf, P_kf, eig_kf] = lqe(A_x, G_kf, C_x, Q_kf, R_kf);

fprintf('\nGuadagno Kalman L_kf:\n');
disp(L_kf);

fprintf('Poli del filtro di Kalman:\n');
disp(eig(A_x - L_kf*C_x));

% Modello del filtro:
%   xhat_dot = A_x*xhat + B_x*u + L_kf*(y - C_x*xhat)
%
% Forma State Space:
%   xhat_dot = (A_x - L_kf*C_x)*xhat + [B_x  L_kf]*[u; y]
%   output   = xhat

A_kf = A_x - L_kf*C_x;
B_kf = [B_x L_kf];
C_kf = eye(2);
D_kf = zeros(2,2);

E_kf = eye(2);

SS_kf = ss(A_kf, B_kf, C_kf, D_kf);

fprintf('\nModello state-space Kalman Filter:\n');
SS_kf

%% 4) LQ / LQR Controller
fprintf(' 4) LQ CONTROLLER - LQR\n');

% Verifica controllabilità
Mr_lq = ctrb(A_x, B_x);

if rank(Mr_lq) == size(A_x,1)
    disp('Il sistema meccanico linearizzato e'' controllabile, LQR possibile')
else
    error('Il sistema non e'' controllabile: LQR non possibile')
end

% 4.1) Scelta dei pesi LQ
% Stato: x = [delta_posizione; delta_velocita]
% Ingresso: delta_i, variazione di corrente rispetto alla corrente di equilibrio

% Valori massimi "accettabili" usati per normalizzare i pesi
delta_x_max  = 2e-3;   % [m] errore posizione ammesso, es. 2 mm
delta_xd_max = 0.10;   % [m/s] velocita' ammessa
delta_i_max  = 0.50;   % [A] variazione corrente ammessa

% Bryson rule: peso = 1 / valore_massimo^2
Q_lq = diag([1/delta_x_max^2, 1/delta_xd_max^2]);
R_lq = 1/delta_i_max^2;

fprintf('\nMatrice Q_lq:\n');
disp(Q_lq);

fprintf('Matrice R_lq:\n');
disp(R_lq);

% 4.2) Guadagno LQR
K_lq = lqr(A_x, B_x, Q_lq, R_lq);

fprintf('\nGuadagno di stato K_lq:\n');
disp(K_lq);

% 4.3) Verifica dei poli in anello chiuso
A_cl_lq = A_x - B_x*K_lq;
eig_cl_lq = eig(A_cl_lq);

fprintf('Poli ottenuti con LQR:\n');
disp(eig_cl_lq);

% Sistema in anello chiuso da delta_x_ref a delta_y
SS_cl_lq = ss(A_cl_lq, B_x, C_x, D_x);

% 4.4) Guadagno statico di scaling per inseguimento riferimento
Kdc = dcgain(SS_cl_lq);
K_ref = 1/Kdc;

fprintf('Guadagno reference K_ref:\n');
disp(K_ref);

% Formula equivalente, utile da confrontare:
Nbar_lq = -1/(C_x*((A_x - B_x*K_lq)\B_x));

fprintf('Prefiltro Nbar_lq:\n');
disp(Nbar_lq);



