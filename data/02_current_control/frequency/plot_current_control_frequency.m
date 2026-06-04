clear; close all; clc;

scriptDir = fileparts(mfilename('fullpath'));
matFiles = dir(fullfile(scriptDir, '*.mat'));

signalLabels = {
    'Reference signal'
    'Measured signal'
    'Signal 3'
    'Signal 4'
    'Signal 5'
    'Signal 6'
};

for fileIdx = 1:numel(matFiles)
    filePath = fullfile(matFiles(fileIdx).folder, matFiles(fileIdx).name);
    [t, signals] = loadValidationMatrix(filePath);
    labels = buildLabels(size(signals, 1), signalLabels);
    metrics = computeFrequencyMetrics(t, signals(1, :), signals(2, :));

    figure('Color', 'w', 'Name', erase(matFiles(fileIdx).name, '.mat'));
    plotValidationSeries(t, signals, labels, sprintf('Current control frequency validation - %s', erase(matFiles(fileIdx).name, '.mat')));
    printFrequencyMetrics(matFiles(fileIdx).name, metrics);
end

function [t, signals] = loadValidationMatrix(filePath)
    loaded = load(filePath);
    varNames = fieldnames(loaded);
    data = loaded.(varNames{1});

    if size(data, 1) > size(data, 2) && size(data, 2) <= 32
        data = data.';
    end

    if size(data, 1) < 2
        error('Expected at least one time row and one signal row in %s.', filePath);
    end

    t = double(data(1, :));
    signals = double(data(2:end, :));
end

function labels = buildLabels(numSignals, defaultLabels)
    labels = cell(numSignals, 1);
    for idx = 1:numSignals
        if idx <= numel(defaultLabels)
            labels{idx} = defaultLabels{idx};
        else
            labels{idx} = sprintf('Signal %d', idx);
        end
    end
end

function plotValidationSeries(t, signals, labels, plotTitle)
    if size(signals, 1) < 2
        error('Expected at least two signals to plot in %s.', plotTitle);
    end

    plot(t, signals(1, :), 'LineWidth', 1.3, 'Color', [0 0 1]);
    hold on;
    plot(t, signals(2, :), 'LineWidth', 1.3, 'Color', [1 0 0]);
    grid on;
    title(plotTitle);
    xlabel('Time [s]');
    ylabel('Current');
    legend({labels{1}, labels{2}}, 'Location', 'best');
    hold off;
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

function printFrequencyMetrics(fileName, metrics)
    fprintf('\n%s\n', fileName);
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
