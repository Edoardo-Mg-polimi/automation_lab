% Modello e controllo in frequenza
clear all;
clc
fprintf('\n');
fprintf('============================================================\n');
fprintf(' MODELLO E CONTROLLO IN FREQUENZA - MAGLEV\n');
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

I = [1, 0; 0, 1];

%% 1) Modello circuito RL
fprintf(' 1) MODELLO CIRCUITO RL\n');
s = tf('s');
fprintf("fdt circuito RL: \n");
wp = R/L;% unico polo di T(s) calcolabile anche con funzione pole
G_i = 1/(s*L +R)
[G_i_num, G_i_den]   = tfdata(G_i, 'v');
poles_G_i = pole(G_i)

fprintf("amplificatore: Kg=%f\n", Kg);

figure
bode(G_i)
grid on
title('Bode G_i(s) circito RL')

figure
nyquist(G_i)
grid on
title('Nyquist G_i(s) circuito RL')



% 1.2) REGOLATORE RL
fprintf(' 1.2) REGOLATORE DI CORRENTE R_i(s)\n');
% Sintesi con metodo di BODE
Kp_i = 14.559055;
Ki_i = 330.005238;
fprintf("Kp_i = %f\n", Kp_i);
fprintf("Ki_i = %f\n", Ki_i);

R_i = Kp_i + Ki_i/s
[R_i_num, R_i_den] = tfdata(R_i, 'v');

% 1.3) FDT ANELLO APERTO
fprintf(' 1.3) ANELLO APERTO CORRENTE L_i(s)\n');
L_i = R_i * Kg * G_i;
[numLi, denLi] = tfdata(L_i, 'v');
poles_L_i = pole(L_i);
zeros_L_i = zero(L_i);
[gm, pm, wcg, wcp_i] = margin(L_i);
fprintf("Gain margin=%f ,Phase margin=%f ,w_c=%f \n", gm, pm, wcp_i);


% 1.4) FDT ANELLO CHIUSO
fprintf(' 1.4) ANELLO CHIUSO CORRENTE H_i(s)\n');
fprintf("\n Hi fdt in retroazione \n");
H_i = L_i/(1+L_i)
H_i = minreal(H_i);
poles_H_i = pole(H_i)


figure
bode(H_i)
grid on
title('Bode H_i(s) anello chiuso')

figure
nyquist(H_i)
grid on
title('Nyquist H_i(s) anello chiuso')

%% 2) Modello meccanico linearizzato
fprintf(' 2) MODELLO MECCANICO LINEARIZZATO\n');
%Equilibrium
x_e = l/2 - r; % posizione di equilibrio a metà (piedistallo - inizio palla)
xd_e = 0;
i_e = sqrt((m*g/km) * (x_e + 2*r -l)^2); % corrente di equilibrio ie(xe)

v_e = i_e * R;

k1 = (2 * km * i_e^2)/(m * (l -2*r -x_e)^3);
k2 = (2 * km * i_e)/(m * (l-2*r -x_e)^2);

% 2.1 - TF
G_x = k2/(s^2 -k1);
[G_x_num, G_x_den]   = tfdata(G_x, 'v');

p = pole(G_x);   % poli
wm = abs(p(1));
fprintf("Frequenza naturale G_x: wm=%f", wm);


% Diagrammi
figure
bode(G_x)
grid on
title('Diagramma di Bode di G_x')

figure
nyquist(G_x)
grid on
title('Diagramma di Nyquist di G_x')

% 2.1 Plant complessivo
fprintf(' 2.2) PLANT COMPLESSIVO G(s) = H_i(s) G_x(s)\n');
G = minreal(H_i * G_x)
poles_G = pole(G)
figure
bode(G)
grid on
title('Diagramma di Bode di G')

figure
nyquist(G)
grid on
title('Diagramma di Nyquist di G')






