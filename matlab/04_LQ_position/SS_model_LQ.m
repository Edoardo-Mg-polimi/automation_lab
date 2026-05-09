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
G_i_num = G_i_num(2);

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
pole_H_i = pole(H_i);
w_i = abs(pole_H_i);

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

%% 2.3) Modello esteso meccanica + anello chiuso corrente
fprintf(' 2.3) MODELLO ESTESO MECCANICA + CORRENTE\n');

% Stati:
% x_ext = [delta_x; delta_xdot; delta_i]
%
% Ingresso:
% u_ext = delta_i_ref
%
% Uscite misurate:
% y_ext = [delta_x; delta_i]

A_ext = [0    1      0;
         k1   0      k2;
         0    0     -w_i];

B_ext = [0;
         0;
         w_i];

C_ext = [1 0 0;
         0 0 1];

C_pos_ext = [1 0 0];

D_ext = zeros(2,1);

fprintf('Autovalori modello esteso aperto:\n');
disp(eig(A_ext));

% Controllabilità modello esteso
Mr_ext = ctrb(A_ext, B_ext);

if rank(Mr_ext) == size(A_ext,1)
    disp('Il modello esteso è controllabile')
else
    error('Il modello esteso non è controllabile')
end

% Osservabilità modello esteso
Mo_ext = obsv(A_ext, C_ext);

if rank(Mo_ext) == size(A_ext,1)
    disp('Il modello esteso è osservabile')
else
    error('Il modello esteso non è osservabile')
end

%% 3) Kalman Filter per stima dello stato
fprintf(' 3) KALMAN FILTER - LQE\n');

% Stati:
% x = [delta_x; delta_xdot; delta_i]
%
% Ingresso:
% u = delta_i_ref
%
% Misure:
% y = [delta_x_meas; delta_i_meas]

% Matrice di misura del sistema
C_meas = [1 0 0;
          0 0 1];

% Disturbi di processo:
% w1 = disturbo equivalente sull'accelerazione
% w2 = disturbo equivalente sulla dinamica di corrente
G_kf = [0 0;
        1 0;
        0 1];

% taratura 1: incertezza sul modello
sigma_a = 20;       % [m/s^2], incertezza sull'accelerazione
sigma_i = 5;    % [A/s], incertezza equivalente sulla dinamica corrente

Q_kf = diag([sigma_a^2, sigma_i^2]);

% taratura 2: incertezza sulle misurazioni
sigma_yx = 0.1e-3;  % [m], rumore sensore posizione
sigma_yi = 0.001;     % [A], rumore sensore corrente

R_kf = diag([sigma_yx^2, sigma_yi^2]);

% Guadagno Kalman continuo
[L_kf, P_kf, eig_kf] = lqe(A_ext, G_kf, C_meas, Q_kf, R_kf);

fprintf('\nGuadagno Kalman L_kf:\n');
disp(L_kf);

fprintf('Poli del filtro di Kalman:\n');
disp(eig_kf);

% Oppure, equivalente:
disp(eig(A_ext - L_kf*C_meas));

% Modello del filtro:
% xhat_dot = A_ext*xhat + B_ext*u + L_kf*(y - C_meas*xhat)
%
% quindi:
% xhat_dot = (A_ext - L_kf*C_meas)*xhat + [B_ext L_kf]*[u; y]
%
% ingressi filtro:
% input 1 = delta_x_meas
% input 2 = delta_i_meas
% input 3 = delta_i_ref

% uscite filtro:
% output = [delta_x_hat; delta_xdot_hat; delta_i_hat]

A_kf = A_ext - L_kf*C_meas;
B_kf = [L_kf B_ext];

C_kf = eye(3);
D_kf = zeros(3,3);

SS_kf = ss(A_kf, B_kf, C_kf, D_kf);

fprintf('\nModello state-space Kalman Filter:\n');
SS_kf
%% 4) LQ / LQR Controller a 3 stati
fprintf(' 4) LQ CONTROLLER - LQR\n');

% Verifica controllabilità
Mr_ext = ctrb(A_ext, B_ext);

if rank(Mr_ext) == size(A_ext,1)
    disp('Il sistema esteso è controllabile, pole placement possibile')
else
    error('Il sistema esteso non è controllabile')
end

% 4.1) Scelta dei pesi LQ
% Stato: x = [posizione; velocita; corrente]

% Bryson rule: peso = 1 / valore_massimo^2
delta_x_max  = 2.0e-3;   % [m]
delta_xd_max = 0.03;   % [m/s]

delta_iref_max = 1.4; % [A]

Q_lq = diag([1/delta_x_max^2, ...
               1/delta_xd_max^2, ...
               0]);

R_lq = 1/delta_iref_max^2;

fprintf('\nMatrice Q_lq:\n');
disp(Q_lq);

fprintf('Matrice R_lq:\n');
disp(R_lq);

% 4.2) Guadagno LQR
K_lq = lqr(A_ext, B_ext, Q_lq, R_lq);

fprintf('\nGuadagno di stato K_lq:\n');
disp(K_lq);

% 4.3) Verifica dei poli in anello chiuso
A_cl_lq = A_ext - B_ext*K_lq;
eig_cl_lq = eig(A_cl_lq);

fprintf('Poli ottenuti con LQR:\n');
disp(eig_cl_lq);

% Sistema in anello chiuso da delta_x_ref a delta_y
SS_cl_lq = ss(A_cl_lq, B_ext, C_ext, D_ext);

% 4.4) Guadagno statico di scaling per inseguimento riferimento
K_ref = -1/(C_pos_ext*((A_ext - B_ext*K_lq)\B_ext));


fprintf('Guadagno reference K_ref:\n');
disp(K_ref);


%% 5) LQR + Integral action
fprintf(' 5) LQR + Integral action \n');

% 5.1) extend the system
n = size(A_ext,1);

% Stato integrale:
% xi_dot = delta_x_ref - delta_x = r - C_pos_ext*x

A_aug = [A_ext              zeros(n,1);
        -C_pos_ext          0];

B_aug = [B_ext;
         0];

Br_aug = [zeros(n,1);
          1];

C_aug = [C_pos_ext 0];

% 5.2) Verifica controllabilità sistema aumentato
Mr_aug = ctrb(A_aug, B_aug);

if rank(Mr_aug) == size(A_aug,1)
    disp('Il sistema aumentato è controllabile, pole placement possibile')
else
    error('Il sistema aumentato NON è controllabile')
end

% 5.3) Scelta dei pesi LQI
% Stato aumentato:
% x_aug = [delta_x; delta_xdot; delta_i; xi]
%
% xi = integrale dell'errore di posizione
% xi_dot = delta_x_ref - delta_x


% gli altri parametri sono stati tarati prima
delta_xi_max     = 1;    % [m*s], tara l'azione integrale


Q_aug = diag([1/delta_x_max^2, ...
              1/delta_xd_max^2, ...
              0, ...
              1/delta_xi_max^2]);

R_aug = 1/delta_iref_max^2;

fprintf('\nMatrice Q_aug:\n');
disp(Q_aug);

fprintf('Matrice R_aug:\n');
disp(R_aug);

% 5.4) Guadagno LQR con azione integrale
K_aug = lqr(A_aug, B_aug, Q_aug, R_aug);

% Separazione dei guadagni
Kx_lqi = K_aug(1:n);     % feedback sugli stati [x, xdot, i]
Ki_lqi = K_aug(end);     % feedback sullo stato integrale xi

fprintf('\nGuadagno aumentato K_aug:\n');
disp(K_aug);

fprintf('Guadagno sugli stati Kx_lqi:\n');
disp(Kx_lqi);

fprintf('Guadagno integrale Ki_lqi:\n');
disp(Ki_lqi);

% 5.5) Check sul sistema finale ad anello chiuso
fprintf('\n 5.5) CHECK SISTEMA FINALE AD ANELLO CHIUSO - LQI\n');

% Matrice di stato del sistema aumentato in anello chiuso
%
% x_aug = [delta_x; delta_xdot; delta_i; xi]
% u = -K_aug*x_aug
%
% xi_dot = delta_x_ref - delta_x

A_cl_lqi = A_aug - B_aug*K_aug;

fprintf('\nMatrice A_cl_lqi:\n');
disp(A_cl_lqi);

% Poli del sistema finale
eig_cl_lqi = eig(A_cl_lqi);

fprintf('\nPoli del sistema LQI finale:\n');
disp(eig_cl_lqi);

% Check stabilità
if all(real(eig_cl_lqi) < 0)
    disp('Il sistema LQI finale è asintoticamente stabile');
else
    warning('Il sistema LQI finale NON è asintoticamente stabile');
end

% Sistema chiuso da riferimento di posizione a posizione
SS_cl_lqi = ss(A_cl_lqi, Br_aug, C_aug, 0);

fprintf('\nSistema chiuso LQI da delta_x_ref a delta_x:\n');
SS_cl_lqi

% Guadagno statico da riferimento a uscita
dc_gain_lqi = dcgain(SS_cl_lqi);

fprintf('\nGuadagno statico da delta_x_ref a delta_x:\n');
disp(dc_gain_lqi);

fprintf('Errore statico teorico per riferimento costante:\n');
disp(1 - dc_gain_lqi);

% Informazioni sui poli: frequenza naturale e smorzamento
[wn_lqi, zeta_lqi, p_lqi] = damp(SS_cl_lqi);

fprintf('\nTabella poli LQI:\n');

T_poles_lqi = table(p_lqi, real(p_lqi), imag(p_lqi), wn_lqi, zeta_lqi, ...
    'VariableNames', {'Pole', 'RealPart', 'ImagPart', 'NaturalFrequency', 'DampingRatio'});

disp(T_poles_lqi);

% Check risposta al gradino sul modello lineare
delta_x_ref_check = 0.7e-3;   % [m], esempio: riferimento di 0.7 mm
t_check = linspace(0, 5, 2000);

r_check = delta_x_ref_check * ones(size(t_check));

[y_check, t_out_check, x_aug_check] = lsim(SS_cl_lqi, r_check, t_check, zeros(n+1,1));

fprintf('\nRiferimento usato per il check:\n');
disp(delta_x_ref_check);

fprintf('Valore finale simulato di delta_x:\n');
disp(y_check(end));

fprintf('Errore finale simulato:\n');
disp(delta_x_ref_check - y_check(end));

% Ricostruzione del comando delta_i_ref richiesto dal controllore
u_check = -(K_aug * x_aug_check')';

fprintf('\nMassimo delta_i_ref richiesto dal controllore:\n');
disp(max(u_check));

fprintf('Minimo delta_i_ref richiesto dal controllore:\n');
disp(min(u_check));

% Corrente assoluta richiesta: i_ref = i_e + delta_i_ref
i_ref_abs_check = i_e + u_check;

fprintf('\nMassima corrente assoluta richiesta i_ref:\n');
disp(max(i_ref_abs_check));

fprintf('Minima corrente assoluta richiesta i_ref:\n');
disp(min(i_ref_abs_check));

if max(i_ref_abs_check) > I_max || min(i_ref_abs_check) < 0
    warning('La corrente richiesta supera i limiti fisici: possibile saturazione nel modello non lineare o in Simulink');
else
    disp('La corrente richiesta resta nei limiti fisici impostati');
end