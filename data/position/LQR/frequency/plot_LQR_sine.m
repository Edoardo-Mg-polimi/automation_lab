clear; close all; clc;

scriptDir = fileparts(mfilename('fullpath'));
files = {
    fullfile(scriptDir, 'LQR_freqValidation1rads_lastlesson.mat')
    fullfile(scriptDir, 'LQR_freqValidation6rads_lastlesson.mat')
    fullfile(scriptDir, 'LQR_freqValidation15rads_lastlesson.mat')
};

labels = {
    'LQR 1 rad/s'
    'LQR 6 rad/s'
    'LQR 15 rad/s'
};

w = 3; % [rad/s]
lowPassCutoffHz = w/(2*pi); % [Hz] REGOLA IL FILTRO

for k = 1:numel(files)
    [t, x_ref, x_meas] = loadFrequencySignal(files{k});
    [x_meas_filt, filtInfo] = lowpassNumeric(t, x_meas, lowPassCutoffHz);

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

% Grafico del filtro passa basso
plotFilterResponse(filtInfo);


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


function [x_filt, filtInfo] = lowpassNumeric(t, x, cutoffHz)
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

    % Salvo le informazioni del filtro per plottarlo dopo
    filtInfo.b = b;
    filtInfo.a = a;
    filtInfo.fs = fs;
    filtInfo.cutoffHz = cutoffHz;
    filtInfo.order = order;
end


function plotFilterResponse(filtInfo)
    b = filtInfo.b;
    a = filtInfo.a;
    fs = filtInfo.fs;
    cutoffHz = filtInfo.cutoffHz;
    order = filtInfo.order;

    n = 4096;

    % Risposta in frequenza del filtro
    [H, f] = freqz(b, a, n, fs);

    % Modulo del filtro applicato una volta
    magSingleDb = 20*log10(abs(H));

    % Modulo effettivo con filtfilt
    % filtfilt applica il filtro due volte, avanti e indietro
    magFiltfiltDb = 20*log10(abs(H).^2);

    % Fase del filtro applicato una volta
    phaseSingleDeg = unwrap(angle(H)) * 180/pi;

    figure('Color', 'w', 'Name', 'Low-pass filter response');

    subplot(2,1,1);
    hold on;
    grid on;

    semilogx(f(2:end), magSingleDb(2:end), 'LineWidth', 1.4);
    semilogx(f(2:end), magFiltfiltDb(2:end), '--', 'LineWidth', 1.4);
    xline(cutoffHz, ':', sprintf('Cutoff = %.2f Hz', cutoffHz), ...
        'LabelOrientation', 'horizontal');

    xlabel('Frequency [Hz]');
    ylabel('Magnitude [dB]');
    title(sprintf('Low-pass Butterworth filter - order %d', order));
    legend({'Single pass', 'Effective with filtfilt'}, 'Location', 'best');

    subplot(2,1,2);
    hold on;
    grid on;

    semilogx(f(2:end), phaseSingleDeg(2:end), 'LineWidth', 1.4);
    yline(0, '--');

    xlabel('Frequency [Hz]');
    ylabel('Phase [deg]');
    title('Single-pass phase response');
end