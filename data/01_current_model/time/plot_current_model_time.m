clear; close all; clc;

L = 412.5e-3; % [H]
Rc = 10; % [ohm]
Rs = 1; % [ohm]
Rcal = 0.85; % calibration resistance gain
R = (Rc + Rs) * Rcal; % [ohm]
s = tf('s');
G_i = 1 / (L * s + R);

scriptDir = fileparts(mfilename('fullpath'));
matFiles = dir(fullfile(scriptDir, '*.mat'));

if isempty(matFiles)
    error('No .mat files found in %s', scriptDir);
end

[~, order] = sort({matFiles.name});
matFiles = matFiles(order);

figI = figure;
figure(figI); hold on; grid on;

figV = figure;
figure(figV); hold on; grid on;

col_current = [0 0.4470 0.7410];
col_voltage = [0.8500 0.3250 0.0980];

for k = 1:length(matFiles)
    filePath = fullfile(scriptDir, matFiles(k).name);
    S = load(filePath);
    varName = fieldnames(S);
    data = S.(varName{1});

    if size(data, 1) > size(data, 2) && size(data, 2) <= 32
        data = data.';
    end

    t = data(1, :);
    baseLabel = erase(matFiles(k).name, '.mat');

    figure(figI);
    if size(data, 1) >= 3
        plot(t, data(3, :), 'Color', col_current, 'LineWidth', 1.3, ...
            'DisplayName', [baseLabel ' current']);
    end

    if size(data, 1) >= 4
        figure(figV);
        plot(t, data(4, :), 'Color', col_voltage, 'LineWidth', 1.3, ...
            'DisplayName', [baseLabel ' voltage']);
    end

    inputSignal = [];
    if size(data, 1) >= 4
        inputSignal = data(4, :);
    end

    metrics = compute_system_metrics(t, data(3, :), inputSignal, G_i);
    print_system_metrics(baseLabel, metrics);
end

figure(figI);
xlabel('Time [s]');
ylabel('Current [A]');
title('Current model - time');
legend('show', 'Interpreter', 'none', 'Location', 'best');

figure(figV);
xlabel('Time [s]');
ylabel('Voltage [V]');
title('Voltage vs Time - current model');
legend('show', 'Interpreter', 'none', 'Location', 'best');

function metrics = compute_system_metrics(t, y, inputSignal, modelTF)
    t = t(:).';
    y = y(:).';

    if nargin < 3 || isempty(inputSignal)
        inputSignal = [];
    else
        inputSignal = inputSignal(:).';
    end

    if nargin < 4
        modelTF = [];
    end

    metrics = struct('riseTime', NaN, 'staticGain', NaN, 'steadyStateError', NaN, ...
        'dominantPole', NaN, 'yInitial', NaN, 'yFinal', NaN, 'modelFinal', NaN);

    nSamples = numel(t);
    if nSamples < 3 || numel(y) ~= nSamples
        return;
    end

    windowSize = max(3, round(0.1 * nSamples));
    idxInitial = 1:windowSize;
    idxFinal = max(1, nSamples - windowSize + 1):nSamples;

    y0 = mean(y(idxInitial), 'omitnan');
    yss = mean(y(idxFinal), 'omitnan');
    metrics.yInitial = y0;
    metrics.yFinal = yss;

    if ~isempty(inputSignal) && numel(inputSignal) == nSamples
        u0 = mean(inputSignal(idxInitial), 'omitnan');
        uss = mean(inputSignal(idxFinal), 'omitnan');
        if abs(uss - u0) <= eps && abs(uss) > eps
            u0 = 0;
        end

        stepAmp = uss - u0;

        if abs(uss) > eps
            metrics.staticGain = yss / uss;
        elseif abs(stepAmp) > eps
            metrics.staticGain = (yss - y0) / stepAmp;
        end

        if ~isempty(modelTF)
            modelGain = dcgain(modelTF);
            if isfinite(modelGain)
                metrics.modelFinal = modelGain * uss;
                metrics.steadyStateError = metrics.modelFinal - yss;
            end
        end
    else
        stepAmp = NaN;
    end

    responseAmp = yss - y0;
    if isnan(metrics.staticGain) && ~isnan(stepAmp) && abs(stepAmp) > eps
        metrics.staticGain = responseAmp / stepAmp;
    end

    if abs(responseAmp) > eps
        normalized = (y - y0) / responseAmp;
        idx10 = find(normalized >= 0.10, 1, 'first');
        idx90 = find(normalized >= 0.90, 1, 'first');
        idx63 = find(normalized >= 0.632, 1, 'first');

        if ~isempty(idx10) && ~isempty(idx90) && idx90 >= idx10
            metrics.riseTime = t(idx90) - t(idx10);
        end

        if ~isempty(idx63)
            tau = t(idx63) - t(1);
            if tau > 0
                metrics.dominantPole = -1 / tau;
            end
        end

        if isnan(metrics.dominantPole) && ~isnan(metrics.riseTime) && metrics.riseTime > 0
            metrics.dominantPole = -2.197 / metrics.riseTime;
        end
    end
end

function print_system_metrics(label, metrics)
    fprintf('\n%s\n', label);
    fprintf('  Rise time (10-90%%): %s s\n', format_metric(metrics.riseTime, '%.4g'));
    fprintf('  Static gain: %s\n', format_metric(metrics.staticGain, '%.4g'));
    fprintf('  Steady-state current from RL model: %s A\n', format_metric(metrics.modelFinal, '%.4g'));
    fprintf('  Steady-state error vs RL model: %s A\n', format_metric(metrics.steadyStateError, '%.4g'));
    fprintf('  Dominant pole estimate: %s 1/s\n', format_metric(metrics.dominantPole, '%.4g'));
end

function textValue = format_metric(value, formatSpec)
    if isnan(value)
        textValue = 'N/A';
    else
        textValue = sprintf(formatSpec, value);
    end
end
