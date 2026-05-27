% PI sintetis with BODE method for RL circuit (current control)
clear all;
clc
fprintf('\n');
fprintf('============================================================\n');
fprintf(' CURRENT CONTROL MODEL\n');
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


%% 1) Modello circuito RL
fprintf(' 1) MODELLO CIRCUITO RL\n');
s = tf('s');
wi = R/L;% unico polo di T(s) calcolabile anche con funzione pole
tau_i = 1/wi;
G_i = 1/(s*L +R)
[G_i_num, G_i_den]   = tfdata(G_i, 'v');
G_i_num = G_i_num(2);


fprintf("amplificatore: Kg=%f\n", Kg);
fprintf("Pole: %f, Tau: %f \n", wi, tau_i);

figure
bode(G_i)
grid on
title('Bode G_i(s) circito RL')


figure
nyquist(G_i)
grid on
title('Nyquist G_i(s) circuito RL')

% Validazione MODELLO in frequenza
% alpha = [0.01 0.1 0.2 0.5 1 2]; % coefficients
% N = length(alpha);
% dati = zeros(N,5); % [omega, modulo, modulo_dB, fase_deg]
% 
% fprintf("[omega, modulo, modulo_dB, fase_deg, absolute_delta_t] \n");
% for k = 1:N
%     omega = wp * alpha(k);
%     Tj = evalfr(T, 1j*omega);
%     modulo = abs(Tj);
%     modulo_dB = 20*log10(modulo);
%     fase_rad = angle(Tj);
%     fase_deg = rad2deg(angle(Tj));
%     period = (2*pi) / omega;
%     absolute_delta_t = period * (fase_deg/360);
%     dati(k,:) = [omega, modulo, modulo_dB, fase_deg, absolute_delta_t];
% end
% disp(dati)

%% 2) Modello meccanico linearizzato
%Equilibrium
x_e = l/2 - r; % posizione di equilibrio a metà (piedistallo - inizio palla)
xd_e = 0;
i_e = sqrt((m*g/km) * (x_e + 2*r -l)^2); % corrente di equilibrio ie(xe)

v_e = i_e * R;
fprintf("Tensione di equilibrio: v_e = %f \n", v_e);

k1 = (2 * km * i_e^2)/(m * (l -2*r -x_e)^3);
k2 = (2 * km * i_e)/(m * (l-2*r -x_e)^2);
fprintf("Costanti linearizzazione: k1=%f k2=%f \n",k1, k2);

% 2.1 - TF
G_x = k2/(s^2 -k1);
[numW, denW]   = tfdata(G_x, 'v');

p = pole(G_x)   % poli
wm = abs(p(1));
fprintf("Frequenza naturale G_x: wm=%f", wm);

%% 3) BODE syntesis 
wc = 10*wm;
phase_m = 90; %margine di fase in deg
fprintf("\n Requirements: wc=%f , phase margin=%f \n", wc, phase_m);

phiG_deg = -atand((wc*L)/R);%fase a omega c
Ti = 1/(wc * tand(180 + phiG_deg -phase_m));
Kp_i = sqrt(R^2 + (wc*L)^2) / ( Kg * sqrt(1 + 1/(wc^2 * Ti^2)) );
Ki_i = Kp_i/Ti;

%% 4) PI
fprintf("Regulator:\n");
fprintf("Kp_i = %f\n", Kp_i);
fprintf("Ki_i = %f\n", Ki_i);
R_i = Kp_i + Ki_i/s
[R_i_num, R_i_den] = tfdata(R_i, 'v');

%% 5) Limiti di delta I
%massimo gradino di corrente inseguibile dalla corrente di equilibrio
delta_i_max = (V_max - R*i_e) / (wc*L);
fprintf("Massimo gradino di corrente inseguibile: delta_i_max=%f A\n", delta_i_max);

%% 6) Check e diagrammi sistema RL controllato

% FDT con controllore ANELLO APERTO - L_i
L_i = R_i * Kg * G_i;
[numLi, denLi] = tfdata(L_i, 'v');
p_i = pole(L_i)
z_i = zero(L_i)
[gm, pm, wcg, wcp_i] = margin(L_i);
fprintf("Gain margin=%f ,Phase margin=%f ,w_c=%f \n", gm, pm, wcp_i);

figure
bode(L_i)
grid on
title('Bode L_i(s) anello aperto')

figure
nyquist(L_i)
grid on
title('Nyquist L_i(s) anello aperto')

% FDT ANELLO CHIUSO
H_i = minreal(L_i/(1+L_i))
poles_Hi = pole(H_i)

figure
bode(H_i)
grid on
title('Bode H_i(s) closed loop')

figure
nyquist(H_i)
grid on
title('Nyquist H_i(s) closed loop')