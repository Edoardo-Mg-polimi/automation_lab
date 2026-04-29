% State-space models for current and position dynamics
clear all;
clc
fprintf('\n');
fprintf('============================================================\n');
fprintf(' Non Linear model\n');
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

%% Modello
% Equilibrium
x_e = l/2 - r; % posizione di equilibrio a metà (piedistallo - inizio palla)
xd_e = 0;
i_e = sqrt((m*g/km) * (x_e + 2*r -l)^2); % corrente di equilibrio ie(xe)
X0 = [x_e; xd_e; i_e];

function [dx, y] = nl_maglev(X, v)
% Stati:
%   X(1) = x     posizione sfera [m]
%   X(2) = xdot  velocita sfera [m/s]
%   X(3) = i     corrente bobina [A]
%
% Ingresso:
%   u_cmd = comando al VoltPAQ [V], prima del guadagno Kg
%
% Uscite:
%   y(1) = x
%   y(2) = i

x1 = X(1);
x2 = X(2);
x3 = X(3);

u = kg * v;

% NL states
x1_d = x2;
x2_d = (km/m) * x3^2/(l -2*r -x1)^2 -g;
x3_d = (u - R*x3)/L;

dx = [x1_d, x2_d, x3_d];

% Uscite fisiche
y = [x; i];
end