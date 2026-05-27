clear; close all; clc;

scriptDir = fileparts(mfilename('fullpath'));
files = {
    fullfile(scriptDir, 'FB_freqvalid1rads_lastlesson.mat')
    fullfile(scriptDir, 'FB_freqvalid3rads_lastlesson.mat')
    fullfile(scriptDir, 'FB_freqvalid6rads_lastlesson.mat')
    fullfile(scriptDir, 'FB_freqvalid20rads_lastlesson.mat')
};

labels = {
    'FB 1 rad/s'
    'FB 3 rad/s'
    'FB 6 rad/s'
    'FB 20 rad/s'
};

w = 9; % [rad/s]
lowPassCutoffHz = w/(2*pi);%[Hz] REGOLA IL FILTRO

for k = 1:numel(files)
    [t, x_ref, x_meas] = loadFrequencySignal(files{k});
    x_meas_filt = lowpassNumeric(t, x_meas, lowPassCutoffHz);

    figure('Color', 'w', 'Name', labels{k});
    hold on;
    grid on;

    plot(t, x_ref, 'LineWidth', 1.4);
    plot(t, x_meas_filt, 'LineWidth', 1.4);

    xlabel('Time [s]');
    ylabel('Position [m]');
    title([labels{k}, ' - low-pass filtered measured position']);
    legend({'Position reference', 'Measured position (filtered)'}, 'Location', 'best');
end

function [t, x_ref, x_meas] = loadFrequencySignal(filePath)
    S = load(filePath);
    varNames = fieldnames(S);
    data = S.(varNames{1});

    if isvector(data)
        error('Expected a 2-D numeric array in %s.', filePath);
    end

    if size(data, 1) > size(data, 2) && size(data, 2) <= 32
        data = data.';
    end

    t = double(data(1, :));
    x_ref = double(data(2, :));
    x_meas = double(data(3, :));
end

function x_filt = lowpassNumeric(t, x, cutoffHz)
    t = double(t(:));
    x = double(x(:));

    dt = median(diff(t));
    fs = 1 / dt;

    % Controllo sul cutoff
    if cutoffHz >= fs/2
        error('cutoffHz must be lower than Nyquist frequency fs/2.');
    end

    % Butterworth passa basso
    order = 2;
    Wn = cutoffHz / (fs/2);

    [b, a] = butter(order, Wn, 'low');

    % Filtro zero-phase, avanti e indietro
    x_filt = filtfilt(b, a, x);

    x_filt = x_filt.';
end