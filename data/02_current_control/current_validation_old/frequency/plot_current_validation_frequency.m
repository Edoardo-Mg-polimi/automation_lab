clear; close all; clc;

scriptDir = fileparts(mfilename('fullpath'));
matFiles = dir(fullfile(scriptDir, '*.mat'));

if isempty(matFiles)
    error('No .mat files found in %s', scriptDir);
end

[~, order] = sort({matFiles.name});
matFiles = matFiles(order);

col_ref = [0 0.4470 0.7410];
col_meas = [0.8500 0.3250 0.0980];

for k = 1:numel(matFiles)
    filePath = fullfile(scriptDir, matFiles(k).name);
    S = load(filePath);
    varName = fieldnames(S);
    data = S.(varName{1});

    if size(data, 1) > size(data, 2) && size(data, 2) <= 32
        data = data.';
    end

    if size(data, 1) < 3
        error('Expected at least a time row and two signals in %s', matFiles(k).name);
    end

    t = data(1, :);
    baseLabel = erase(matFiles(k).name, '.mat');
    metrics = computeFrequencyMetrics(t, data(3, :), data(2, :));

    figure('Color', 'w', 'Name', baseLabel);
    hold on;
    grid on;

    plot(t, data(3, :), 'Color', col_ref, 'LineWidth', 1.3, ...
        'DisplayName', 'Reference');
    plot(t, data(2, :), 'Color', col_meas, 'LineWidth', 1.3, ...
        'DisplayName', 'Measured');

    xlabel('Time [s]');
    ylabel('Current [A]');
    title(sprintf('Current validation - %s', baseLabel));
    legend('show', 'Interpreter', 'none', 'Location', 'best');
    hold off;

    printFrequencyMetrics(matFiles(k).name, metrics);
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