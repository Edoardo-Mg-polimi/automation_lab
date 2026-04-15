clear; clc; close all;
s = tf('s');

%% SECTION 1 - Physical parameters
% =========================================================
Kg = 3;

L = 412.5e-3;
Rc = 10;
Rs = 1;
Rcal = 0.85;
R = (Rc + Rs)*Rcal;

km = 6.5308e-5;
r  = 1.27e-2;
m  = 0.068;
l  = 0.0394;
g  = 9.81;

%% SECTION 2 - Current closed loop H_i(s)
T_rl = 1/(L*s + R);

Kp_i = 6.8573;
Ki_i = 470.5931;

PI_i = Kp_i + Ki_i/s;
Li = Kg * PI_i * T_rl;
H_i = minreal(feedback(Li,1));


%% SECTION 3 - Mechanical plant W(s)
x1e = l/2 - r;
x3e = sqrt((m*g/km) * (x1e + 2*r - l)^2);

k1 = (2 * km * x3e^2) / (m * (l - 2*r - x1e)^3);
k2 = (2 * km * x3e)   / (m * (l - 2*r - x1e)^2);

W = minreal(k2 / (s^2 - k1));
nyquist(W);

%% SECTION 4 - Equivalent plant G(s)=H_i(s)*W(s)
G = minreal(H_i * W);

[numG, denG] = tfdata(G,'v');

disp('G(s) = ');
G

fprintf('Poles of G:\n');
disp(pole(G));

fprintf('Zeros of G:\n');
disp(zero(G));


