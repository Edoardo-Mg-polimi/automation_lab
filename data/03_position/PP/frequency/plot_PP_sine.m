clear; close all; clc;

scriptDir = fileparts(mfilename('fullpath'));
filePath = fullfile(scriptDir,'PP_freqvalid1rads_lastlesson.mat');

w = 20; % [rad/s]
lowPassCutoffHz = w/(2*pi); % [Hz] REGOLA IL FILTRO

[t, x_ref, x_meas] = loadFrequencySignal(filePath);
[x_meas_filt, filtInfo] = lowpassNumeric(t, x_meas, lowPassCutoffHz);
metrics = computeFrequencyMetrics(t, x_ref, x_meas_filt);

figure('Color', 'w');
hold on;
grid on;

plot(t, x_ref, 'LineWidth', 1.4);
plot(t, x_meas_filt, 'LineWidth', 1.4);

xlabel('Time [s]');
ylabel('Measured position [m]');
title('PP frequency validation - low-pass filtered measured position');
legend({'Position reference', 'Measured position (filtered)'}, 'Location', 'best');

printFrequencyMetrics('PP 1 rad/s', metrics);

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

    % Modulo del filtro singolo
    magSingleDb = 20*log10(abs(H));

    % Modulo effettivo con filtfilt
    % filtfilt applica il filtro due volte, avanti e indietro
    magFiltfiltDb = 20*log10(abs(H).^2);

    % Fase del filtro singolo
    phaseSingleDeg = unwrap(angle(H)) * 180/pi;

    figure('Color', 'w');
    
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

function metrics = computeFrequencyMetrics(t, referenceSignal, measuredSignal)
    t = double(t(:).');
    referenceSignal = double(referenceSignal(:).');
    measuredSignal = double(measuredSignal(:).');

    nSamples = min([numel(t), numel(referenceSignal), numel(measuredSignal)]);
    metrics = struct('attenuationDb', NaN, 'phaseShiftDeg', NaN, 'dominantFrequencyHz', NaN);

    if nSamples < 8
        return;
    end

    t = t(1:nSamples);
    referenceSignal = referenceSignal(1:nSamples);
    measuredSignal = measuredSignal(1:nSamples);

    validMask = isfinite(t) & isfinite(referenceSignal) & isfinite(measuredSignal);
    if nnz(validMask) < 8
        return;
    end

    t = t(validMask);
    referenceSignal = referenceSignal(validMask);
    measuredSignal = measuredSignal(validMask);

    startIdx = max(1, floor(0.5 * numel(t)));
    t = t(startIdx:end);
    referenceSignal = referenceSignal(startIdx:end);
    measuredSignal = measuredSignal(startIdx:end);

    if numel(t) < 8
        return;
    end

    dt = median(diff(t));
    if ~isfinite(dt) || dt <= 0
        return;
    end

    referenceSignal = referenceSignal - mean(referenceSignal, 'omitnan');
    measuredSignal = measuredSignal - mean(measuredSignal, 'omitnan');

    nfft = 2 ^ nextpow2(numel(t));
    referenceSpectrum = fft(referenceSignal, nfft);
    measuredSpectrum = fft(measuredSignal, nfft);
    freqAxis = (0:nfft - 1) * (1 / (dt * nfft));

    positiveBins = 2:max(2, floor(nfft / 2));
    [~, maxIdx] = max(abs(referenceSpectrum(positiveBins)));
    dominantBin = positiveBins(maxIdx);

    referenceComponent = referenceSpectrum(dominantBin);
    measuredComponent = measuredSpectrum(dominantBin);

    if abs(referenceComponent) <= eps
        return;
    end

    metrics.dominantFrequencyHz = freqAxis(dominantBin);
    metrics.attenuationDb = 20 * log10(abs(measuredComponent) / abs(referenceComponent));
    phaseShiftDeg = rad2deg(angle(measuredComponent) - angle(referenceComponent));
    metrics.phaseShiftDeg = mod(phaseShiftDeg + 180, 360) - 180;
end

function printFrequencyMetrics(label, metrics)
    fprintf('\n%s\n', label);
    fprintf('  Dominant frequency: %s Hz\n', format_metric(metrics.dominantFrequencyHz, '%.4g'));
    fprintf('  Attenuation: %s dB\n', format_metric(metrics.attenuationDb, '%.4g'));
    fprintf('  Phase shift: %s deg\n', format_metric(metrics.phaseShiftDeg, '%.4g'));
end

function textValue = format_metric(value, formatSpec)
    if isnan(value)
        textValue = 'N/A';
    else
        textValue = sprintf(formatSpec, value);
    end
end