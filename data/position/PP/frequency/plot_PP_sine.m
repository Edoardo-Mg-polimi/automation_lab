clear; close all; clc;

% File da plottare
files = {
%    'data_26-May-2026_18-24-22.mat'
%    'data_26-May-2026_18-27-22.mat'
'data_26-May-2026_18-28-12.mat'
%'data_26-May-2026_18-31-13.mat'
% 'data_26-May-2026_18-32-11.mat'
% 'data_26-May-2026_18-38-01.mat'
% 'data_26-May-2026_18-40-39.mat'
% 'data_26-May-2026_18-45-56.mat'
% 'data_26-May-2026_18-46-27.mat'
% 'data_26-May-2026_18-47-30.mat'
% 'data_26-May-2026_18-49-47.mat'
% 'data_26-May-2026_18-50-31.mat'
% 'data_26-May-2026_18-51-41.mat'
% 'data_26-May-2026_18-52-51.mat'
% 'data_26-May-2026_18-53-37.mat'
% 'data_26-May-2026_18-54-12.mat'
% 'data_26-May-2026_18-54-43.mat'
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