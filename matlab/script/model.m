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


%% 1) Modello circuito LR
s = tf('s');
fprintf("T fdt circuito RL: \n");
wp = R/L;            % unico polo di T(s) calcolabile anche con funzione pole
T = 1/(s*L +R)
[numT, denT]   = tfdata(T, 'v');

p = pole(T);   % poli
fprintf("Polo RL: %f \n", p);
tau_RL = 1/abs(wp);
fprintf("Costante tempo RL: %f s \n", tau_RL);

figure
bode(T)
grid on
title('Diagramma di Bode di T')

figure
nyquist(T)
grid on
title('Diagramma di Nyquist di T')


% Validazione in frequenza
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


% 1.1) REGOLATORE RL
% Sintesi con metodo di BODE


% Sintesi diretta
wc = 5*wp;           % prova iniziale conservativa
wz = wp;             %cancellazione polo zero
% Kp = L* (wc/Kg);
% Ki = R* (wc/Kg);
Kp = 6.8573;
Ki = 470.5931;
fprintf("Anello corrente L_i = PI*T \n");
Ci  = Kp + Ki/s;
[numCi, denCi] = tfdata(Ci, 'v');

% 1.3) FDT ANELLO APERTO
Li = Ci * Kg * T;
[numLi, denLi] = tfdata(Li, 'v');
p_i = pole(Li)
z_i = zero(Li)
[gm, pm, wcg, wcp] = margin(Li);
fprintf("Gain margin=%f ,Phase margin=%f ,w_c=%f \n", gm, pm, wcp);

% Diagrammi
figure
bode(Li)
grid on
title('Bode L(s) anello aperto')

figure
nyquist(Li)
grid on
title('Nyquist L(s) anello aperto')

% 1.4) FDT ANELLO CHIUSO
fprintf("\n Hi fdt in retroazione \n");
Hi = Li/(1+Li);
Hi = minreal(Hi);
p = pole(Hi)
tau_Hi = 1/abs(p(1));
fprintf("Cosatnte di tempo RL controllato: %f \n", tau_Hi);


%% 2) Modello meccanico linearizzato
%Linearizzo attorno al punto di equilibrio
x1e = l/2 - r; % posizione di equilibrio a metà (piedistallo - inizio palla)
x2e = 0;
x3e = sqrt((m*g/km) * (x1e + 2*r -l)^2); % corrente di equilibrio ie(xe)

u_e = x3e * R;

k1 = (2 * km * x3e^2)/(m * (l -2*r -x1e)^3);
k2 = (2 * km * x3e)/(m * (l-2*r -x1e)^2);

% 2.1 - TF
fprintf("W fdt pallina: \n");
W = k2/(s^2 -k1)
[numW, denW]   = tfdata(W, 'v');

p = pole(W)   % poli
tau_W = 1/abs(p(1));
z = zero(W);   % zeri
fprintf("Costante di tempo pallina W: %f s\n", tau_W);


% 2.2 - Diagrammi
figure
bode(W)
grid on
title('Diagramma di Bode di W')

figure
nyquist(W)
grid on
title('Diagramma di Nyquist di W')


% 2.3 - Comparazione con controllo corrente
delta_x_max = (V_max - R*x3e) / (wcp*L*sqrt(m*g/km));
fprintf("\nMassima variazione di posizione controllabile da anello corrente: delta_x_max=%f m\n", delta_x_max);

%comparazione tra costanti di tempo
fprintf("TAU RL | TAU contr | TAU W \n");
fprintf("%f|%f|%f  ", tau_RL, tau_Hi, tau_W);
if (tau_Hi < tau_W)
    fprintf("PI ok: veloctà nel range");
end

 
% %Calcolo tramite state space representation
% % F = [0, 1;
% %     k1, 0];
% % 
% % G = [0; k2];
% % 
% % H = [1, 0];
% % 
% % system = ss(F, G, H, 0); 
% % W = tf(system)

%% Modello completo open loop senza controllori
% %State space representation dell'intero sistema
% % A = [0, 1, 0;
% %     k1, 0, k2;
% %     0, 0, -(R/L)];
% % 
% % B = [0;
% %      0;
% %      1/L];
% % 
% % C = [1, 0, 0];
% % 
% % D = 0;
% % sys_ss = ss(A,B,C,D);
% % G_ss = tf(sys_ss)
% 
% % Tramite le due TF a cascata
% G = T*W;
% [num, den] = tfdata(G, 'v');   % estrae numeratore e denominatore come vettori
% den0 = den(1);                 % coefficiente del termine di grado massimo
% 
% G = tf(num/den0, den/den0)


