% PID synthesis with ROUTH criterion only on W(s)
% Hypothesis: H_i(s) ~= 1 in the bandwidth of the position loop

clear; clc; close all;
s = tf('s');

%% SECTION 1 - Physical parameters
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

% Current controller
Kp_i = 29.962945;
Ki_i = 5531.893013;

PI_i = Kp_i + Ki_i/s;
Li   = Kg * PI_i * T_rl;
H_i  = minreal(feedback(Li,1));

%% SECTION 3 - Mechanical plant W(s)
x1e = l/2 - r;
x3e = sqrt((m*g/km) * (x1e + 2*r - l)^2);

k1 = (2 * km * x3e^2) / (m * (l - 2*r - x1e)^3);
k2 = (2 * km * x3e)   / (m * (l - 2*r - x1e)^2);

W = minreal(k2 / (s^2 - k1));

% Full plant, useful only for final check of the approximation H_i ~= 1
G_real = minreal(H_i * W);

%% SECTION 4 - Check of the hypothesis H_i(s) ~= 1
% This is NOT a Routh requirement.
% It is a practical requirement: current loop must be much faster than
% the position loop, so that H_i(s) can be approximated as 1.

omega_i = bandwidth(H_i);   % current-loop bandwidth [rad/s]
omega_w = sqrt(k1);         % unstable mechanical pole magnitude [rad/s]

fprintf('\n================ CURRENT LOOP CHECK ================\n');
fprintf('Bandwidth of H_i(s)         = %.4f rad/s\n', omega_i);
fprintf('|unstable pole of W(s)|     = %.4f rad/s\n', omega_w);
fprintf('Ratio omega_i / sqrt(k1)    = %.4f\n', omega_i / omega_w);

%% SECTION 5 - REQUIREMENTS IMPOSED BY ROUTH ON W(s)
% W(s) = k2 / (s^2 - k1)
%
% With PD: C_PD(s) = Kp + Kd*s
% Closed-loop characteristic polynomial:
% p_PD(s) = s^2 + k2*Kd*s + (k2*Kp - k1)
%
% ===== REQUIREMENTS IMPOSED BY ROUTH FOR PD =====
% 1) Kd > 0
% 2) Kp > k1/k2
%
% These conditions guarantee only stability, not performance.

Kp_min = k1 / k2;

fprintf('\n================ ROUTH REQUIREMENTS (PD) ================\n');
fprintf('Requirement 1: Kd > 0\n');
fprintf('Requirement 2: Kp > k1/k2 = %.6f\n', Kp_min);

%% SECTION 6 - CHOICE OF PD PARAMETERS
% ===== THESE ARE DESIGN CHOICES, NOT ROUTH REQUIREMENTS =====
%
% Since Routh gives only the stable region, we choose zeta and wn
% to place the dominant poles of the PD closed loop.
%
% Practical requirement:
% position loop slower than current loop, e.g. by a factor ~5.
% This helps keeping H_i(s) ~= 1 in the position-loop bandwidth.

zeta_des = 0.80;            % design choice
wn_des   = omega_i / 5;     % design choice, practical separation of dynamics

% From coefficient matching on the PD closed loop:
% s^2 + k2*Kd*s + (k2*Kp-k1) = s^2 + 2*zeta*wn*s + wn^2
Kd = (2 * zeta_des * wn_des) / k2;
Kp = (wn_des^2 + k1) / k2;

fprintf('\n================ CHOSEN PD PARAMETERS ================\n');
fprintf('Chosen zeta_des            = %.4f\n', zeta_des);
fprintf('Chosen wn_des              = %.4f rad/s\n', wn_des);
fprintf('Computed Kd                = %.6f\n', Kd);
fprintf('Computed Kp                = %.6f\n', Kp);

if Kd <= 0
    error('Routh violation: Kd must be > 0');
end

if Kp <= Kp_min
    error('Routh violation: Kp must be > k1/k2 = %.6f', Kp_min);
end

C_PD   = Kp + Kd*s;
L_PD   = minreal(C_PD * W);
T_PD   = minreal(feedback(L_PD,1));
poles_PD = pole(T_PD);

fprintf('\nClosed-loop poles with PD on W(s):\n');
disp(poles_PD);

%% SECTION 7 - ADDITION OF THE INTEGRAL ACTION
% With PID: C_PID(s) = Kp + Ki/s + Kd*s
%
% Closed-loop characteristic polynomial:
% p_PID(s) = s^3 + k2*Kd*s^2 + (k2*Kp-k1)*s + k2*Ki
%
% ===== REQUIREMENTS IMPOSED BY ROUTH FOR PID =====
% 1) Kd > 0
% 2) Kp > k1/k2
% 3) Ki > 0
% 4) Ki < Kd*(k2*Kp - k1)
%
% Again, Routh gives only stability.
% Inside this interval we choose Ki small enough not to ruin the PD dynamics.

Ki_max = Kd * (k2*Kp - k1);

fprintf('\n================ ROUTH REQUIREMENTS (PID) ================\n');
fprintf('Requirement 1: Kd > 0\n');
fprintf('Requirement 2: Kp > k1/k2 = %.6f\n', Kp_min);
fprintf('Requirement 3: Ki > 0\n');
fprintf('Requirement 4: Ki < Kd*(k2*Kp-k1) = %.6f\n', Ki_max);

% ===== DESIGN CHOICE, NOT ROUTH =====
% Choose Ki as a fraction of the maximum admissible value.
alpha_Ki = 0.10;     % 10%% of Ki_max, conservative first choice
Ki = alpha_Ki * Ki_max;

fprintf('\n================ CHOSEN PID PARAMETERS ================\n');
fprintf('Chosen alpha_Ki            = %.4f\n', alpha_Ki);
fprintf('Computed Ki                = %.6f\n', Ki);

if Ki <= 0
    error('Routh violation: Ki must be > 0');
end

if Ki >= Ki_max
    error('Routh violation: Ki must be < Ki_max = %.6f', Ki_max);
end

C_PID    = Kp + Ki/s + Kd*s;
L_PID_W  = minreal(C_PID * W);
T_PID_W  = minreal(feedback(L_PID_W,1));

% Final check on the full plant G = H_i * W
L_PID_G  = minreal(C_PID * G_real);
T_PID_G  = minreal(feedback(L_PID_G,1));

poles_PID_W = pole(T_PID_W);
poles_PID_G = pole(T_PID_G);

fprintf('\nClosed-loop poles with PID on W(s):\n');
disp(poles_PID_W);

fprintf('\nClosed-loop poles with PID on G_real(s) = H_i(s)*W(s):\n');
disp(poles_PID_G);

%% SECTION 8 - MARGINS
% These are NOT Routh requirements.
% They are useful to evaluate robustness.

[GM_W, PM_W, Wcg_W, Wcp_W] = margin(L_PID_W);
[GM_G, PM_G, Wcg_G, Wcp_G] = margin(L_PID_G);

fprintf('\n================ ROBUSTNESS CHECK ================\n');
fprintf('Approx plant W(s):\n');
fprintf('  GM  = %.6f\n', GM_W);
fprintf('  PM  = %.6f deg\n', PM_W);
fprintf('  Wcg = %.6f rad/s\n', Wcg_W);
fprintf('  Wcp = %.6f rad/s\n', Wcp_W);

fprintf('\nFull plant G_real(s) = H_i(s)*W(s):\n');
fprintf('  GM  = %.6f\n', GM_G);
fprintf('  PM  = %.6f deg\n', PM_G);
fprintf('  Wcg = %.6f rad/s\n', Wcg_G);
fprintf('  Wcp = %.6f rad/s\n', Wcp_G);


