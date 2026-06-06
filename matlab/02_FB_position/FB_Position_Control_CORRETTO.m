clear
clc
%% 0) Parametri 
Kg = 3; % gain amplificatore

% Circuito LR
Lc = 412.5e-3; %[H] inductance of the coil
Rc = 10; %[ohm]
% nc = 2450; %number of coils
% lc = 0.0825; %[m] lenght of the coil
% rc = 0.008; %[m] coil radius
Rs = 1; %[ohm] current sense resistance
Rcal = 0.85; % calibration resistance gain

R_tot = (Rc + Rs)* Rcal; %[ohm] total resistance
omega_coil = R_tot/Lc;

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

%% 1) Modello Circuito LR

%linear, SISO, Strictly proper, A.S., order 1
A_i = -R_tot/Lc; 
B_i = 1/Lc; %system is reachable
C_i = 1; %system is observable 
D_i = 0; 

SS_i = ss(A_i, B_i, C_i, D_i);
[G_i_num, G_i_den] = ss2tf(A_i, B_i, C_i, D_i);
G_i = tf(G_i_num, G_i_den);
G_i = Kg*G_i;
[G_i_num, G_i_den] = tfdata(G_i, 'v');
G_i
fprintf("Poles of G_i = \n");
pole(G_i)
fprintf("Zeros of G_i = \n");
zero(G_i)

%figure
%bode(G_i)
%grid on
%title('Diagramma di Bode di G_i')

%figure
%nyquist(G_i)
%grid on
%title('Diagramma di Nyquist di G_i')

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
G_p = tf(SS_p);
[G_p_num, G_p_den] = tfdata(G_p, 'v');
G_p
fprintf("Poles of G_p = \n");
pole(G_p)
fprintf("Zeros of G_p = \n");
zero(G_p)

%checking reachability of linearized system
Mr = ctrb(A_p,B_p);
if (rank(Mr) == 2)
    disp('The linearized model is reachable \n')
else 
    disp('The linearized model is not reachable \n');
end

%checking observability of linearized system
Mo = obsv(A_p,C_p);
if (rank(Mo) == 2)
    disp('The linearized model is observable \n')
else 
    disp('The linearized model is not observable \n');
end

%% 3) Regolatore Corrente PI

p = pole(G_p); % poles of the position plant
wm = abs(p(1));
wc = 10*wm;
% wc = 150;
phase_m = 90; %margine di fase in deg
fprintf("Requirements: wc=%f , phase margin=%f \n", wc, phase_m);
phiG_deg = -atand((wc*Lc)/R_tot);%fase a omega c
Ti = 1/(wc * tand(180 + phiG_deg -phase_m));
K_i_p = sqrt(R_tot^2 + (wc*Lc)^2) / ( Kg * sqrt(1 + 1/(wc^2 * Ti^2)) );
K_i_i = K_i_p/Ti;
fprintf("Kp = %f\n", K_i_p);
fprintf("Ki = %f\n", K_i_i);

R_i = K_i_p + K_i_i/s %current controller PI
[R_i_num, R_i_den] = tfdata(R_i, 'v');

L_i = G_i*R_i; %current open loop tf
L_i = minreal(L_i)
fprintf("Poles of L_i = \n");
pole(L_i)
fprintf("Zeros of L_i = \n");
zero(L_i)
[gm, pm, wcg, wcp_i] = margin(L_i);
fprintf("Gain Margin = %f ,Phase Margin = %f ,w_c = %f \n", gm, pm, wcp_i);

H_i = feedback(L_i, 1); %current closed loop tf
H_i = minreal(H_i)
fprintf("Poles of H_i = \n");
pole(H_i)
fprintf("Zeros of H_i = \n");
zero(H_i)
bw_i = bandwidth(H_i);
fprintf("Bandwidth of H_i = %f \n", bw_i);

% figure
% step(H_i)
% hold on
% plot(-1, 0, 'rx', 'MarkerSize', 14, 'LineWidth', 2)
% title('Step response of H_i')
% grid on

% figure
% bode(H_i)
% hold on
% plot(-1, 0, 'rx', 'MarkerSize', 14, 'LineWidth', 2)
% title('Bode Diagram of H_i')
% grid on


%% 4) Frequency-Based Position Controller 

G_out = H_i*G_p %current closed loop tf * position plant
fprintf("Poles of G_out = \n");
pole(G_out)
fprintf("Zeros of G_out = \n");
zero(G_out)

%% Stabilizing Regulator

%stabilizing regulator parameters, calculated using sisoTool and Root Locus
R_p_stab = 1984*(s^2 + 105.8*s + 5298)/(s^2 + 400*s + 4.09e04)

[R_p_stab_num, R_p_stab_den] = tfdata(R_p_stab, 'v');

L_p_stab = R_p_stab*G_out;
L_p_stab = minreal(L_p_stab);

H_p_stab = feedback(L_p_stab, 1);
H_p_stab = minreal(H_p_stab)
fprintf("Poles of H_p_stab = \n");
pole(H_p_stab)
fprintf("Zeros of H_p_stab = \n");
zero(H_p_stab)
bw_p_stab = bandwidth(H_p_stab);
fprintf("Bandwidth of H_p = %f \n", bw_p_stab);

% figure
% nyquist(L_p_stab)
% hold on
% plot(-1, 0, 'rx', 'MarkerSize', 14, 'LineWidth', 2)
% title('Nyquist Diagram of the Stabilized System')
% grid on

% figure
% rlocus(L_p_stab)
% hold on
% plot(-1, 0, 'rx', 'MarkerSize', 14, 'LineWidth', 2)
% title('Root Locus of the Stabilized System')
% grid on

%% Reference Tracking Regulator

%PID parameters, calculated using pidTuner
K_p_p = 0; 
K_p_i = 3.0183;
K_p_d = 0;

PID_p = K_p_p + K_p_i/s + s*K_p_d;

%%

L_p = PID_p * H_p_stab %position open loop tf
fprintf("Poles of L_p = \n");
pole(L_p)
fprintf("Zeros of L_p = \n");
zero(L_p)
[gm, pm, wcg, wcp_i] = margin(L_p);
fprintf("Gain Margin = %f ,Phase Margin = %f ,w_c = %f \n", gm, pm, wcp_i);   

H_p = feedback(L_p, 1) %position closed loop tf
fprintf("Poles of H_p = \n");
pole(H_p)
fprintf("Zeros of H_p = \n");
zero(H_p)
bw_p = bandwidth(H_p);
fprintf("Bandwidth of H_p = %f \n", bw_p);

% figure
% bode(H_p)
% hold on
% plot(-1, 0, 'rx', 'MarkerSize', 14, 'LineWidth', 2)
% title('Bode Diagram of H_p')
% grid on

% figure
% nyquist(L_p)
% hold on
% plot(-1, 0, 'rx', 'MarkerSize', 14, 'LineWidth', 2)
% title('Nyquist of L_p')
% grid on

