clear; close all; clc;

% File da plottare
files = {
    'PP_validation_poles_1_80_85.mat'
    'PP_validation_poles_2_80_85.mat'
    'PP_validation_poles_3_70_75_with_integral_sat.mat'
    'PP_validation_poles_5_75_80.mat'
};

% Nomi per la legenda
labels = {
    'PP poles 1, 80-85'
    'PP poles 2, 80-85'
    'PP poles 3, 70-75, integral sat'
    'PP poles 5, 75-80'
};

figure;
hold on; grid on;

for k = 1:length(files)

    % Carica il file .mat
    S = load(files{k});

    % Prende automaticamente la prima variabile salvata nel file
    varName = fieldnames(S);
    data = S.(varName{1});

    % Se per qualche motivo i dati fossero salvati come N x 11, li trasponiamo
    if size(data,2) == 11
        data = data';
    end

    % Estrazione dati secondo il tuo ordine
    t = data(1,:);
    x_meas = data(3,:);

    % Plot measured position
    plot(t, x_meas, 'LineWidth', 1.3);
end

xlabel('Time [s]');
ylabel('Measured position [m]');
title('Measured position comparison');
legend(labels, 'Interpreter', 'none', 'Location', 'best');