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

%% Calcolo wc min
%Linearizzo attorno al punto di equilibrio
x1e = l/2 - r; % posizione di equilibrio a metà (piedistallo - inizio palla)
x2e = 0;
x3e = sqrt((m*g/km) * (x1e + 2*r -l)^2); % corrente di equilibrio ie(xe)

k1 = (2 * km * x3e^2)/(m * (l -2*r -x1e)^3);
k2 = (2 * km * x3e)/(m * (l-2*r -x1e)^2);
% TF
s = tf('s');
W = k2/(s^2 -k1);
[numW, denW]   = tfdata(W, 'v');

p = pole(W);   % poli

wc_min = max(abs(real(p)));
fprintf("Pulsazione di taglio minima: wc_min=%f rad/s", wc_min);

%% Calcolo wc MAX
% Modello meccanico linearizzato - attorno al punto di equilibrio a metà l
% cioè con x=l/2 -r

% x = l/2 -r
x1e = l/2 - r; % x = posizione di equilibrio a metà (piedistallo - inizio palla)
x2e = 0;% velocità di equilibrio
x3e = sqrt((m*g/km) * (x1e + 2*r -l)^2); % corrente di equilibrio ie(xe)

v_e = x3e * R;

fprintf("\nCorrente di equilibrio con xe=l/2 -r: ie(xe=l/2 -r)=%f A\n", x3e);
fprintf("Tensione di equilibrio con xe=l/2 -r: v_e(xe=l/2 -r)=%f V\n", v_e);

% x = 0
x1e = 0; % x = posizione di equilibrio a metà (piedistallo - inizio palla)
x2e = 0;% velocità di equilibrio
x3e_0 = sqrt((m*g/km) * (x1e + 2*r -l)^2); % corrente di equilibrio ie(xe)
v_e = x3e_0 * R;

fprintf("\nCorrente di equilibrio con xe=0: ie(xe=0)=%f A\n", x3e_0);
fprintf("Tensione di equilibrio con xe=0: v_e(xe=l/2 -r)=%f V\n", v_e);

%calcolo delta i
delta_x = l/2 - r;% massima variazione di posizione
delta_i = sqrt(m*g/km) * delta_x;%massima variazione di corrente
fprintf("\nMassima variazione di posizione: delta x=%f m\n", delta_x);
fprintf("Massima variazione di corrente: delta i=%f A\n", delta_i);

%Massima pulsazione di taglio
wc_max = (V_max - R*x3e)/(L*delta_i);
fprintf("\nCaso peggiore: salto istantaneo da x=0 a x=xe=l/2 -r");
fprintf("\nPulsazione di taglio massima: wc_max=%f rad/s\n", wc_max);


