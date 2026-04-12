% PI sintetis with BODE method for RL circuit (current control)
clear all;
clc
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
s = tf('s');
fprintf("T fdt circuito RL: \n");
wp = R/L;% unico polo di T(s) calcolabile anche con funzione pole
T = 1/(s*L +R)
[numT, denT]   = tfdata(T, 'v');

fprintf("amplificatore: Kg=%f\n", Kg);

%% 2) Modello meccanico linearizzato
%Linearizzo attorno al punto di equilibrio
x1e = l/2 - r; % posizione di equilibrio a metà (piedistallo - inizio palla)
x2e = 0;
x3e = sqrt((m*g/km) * (x1e + 2*r -l)^2); % corrente di equilibrio ie(xe)

u_e = x3e * R;

k1 = (2 * km * x3e^2)/(m * (l -2*r -x1e)^3);
k2 = (2 * km * x3e)/(m * (l-2*r -x1e)^2);

% 2.1 - TF
W = k2/(s^2 -k1);
[numW, denW]   = tfdata(W, 'v');

p = pole(W);   % poli
wm = abs(p(1));
%% 3) BODE syntesis 
wc = 5*wm;
phase_m = 60; %margine di fase in deg
fprintf("Requirements: wc=%f , phase margin=%f \n", wc, phase_m);

phiG_deg = -atand((wc*L)/R);%fase a omega c
Ti = 1/(wc * tand(180 + phiG_deg -phase_m));
Kp = sqrt(R^2 + (wc*L)^2) / ( Kg * sqrt(1 + 1/(wc^2 * Ti^2)) );
Ki = Kp/Ti;

fprintf("Kp = %f\n", Kp);
fprintf("Ki = %f\n", Ki);
%% 4) PI
PI = Kp + Ki/s
