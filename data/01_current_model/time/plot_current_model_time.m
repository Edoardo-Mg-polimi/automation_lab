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

col_meas = [1 0 0];
col_sim = [0 0.6 0];
col_voltage = [1 0 0];

for k = 1:length(matFiles)
    filePath = fullfile(scriptDir, matFiles(k).name);
    S = load(filePath);
    baseLabel = erase(matFiles(k).name, '.mat');
    [t, current, voltage] = extract_current_model_data(S);
    traceColor = col_meas;

    if is_simulation_label(baseLabel)
        traceColor = col_sim;
    end

    figure(figI);
    if ~isempty(current)
        plot(t, current, 'Color', traceColor, 'LineWidth', 1.3, ...
            'DisplayName', [baseLabel ' current']);
    end

    if ~isempty(voltage)
        figure(figV);
        plot(t, voltage, 'Color', col_voltage, 'LineWidth', 1.3, ...
            'DisplayName', [baseLabel ' voltage']);
    end

    inputSignal = voltage;

    metrics = compute_system_metrics(t, current, inputSignal, G_i);
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

function [t, current, voltage] = extract_current_model_data(S)
    names = fieldnames(S);
    raw = S.(names{1});
    voltage = [];

    if isa(raw, 'timeseries')
        t = raw.Time;
        current = vector_from_timeseries_data(raw.Data, numel(t));
        return;
    end

    if isstruct(raw) && isfield(raw, 'Time') && isfield(raw, 'Data')
        t = raw.Time;
        current = vector_from_timeseries_data(raw.Data, numel(t));
        return;
    end

    if ~isnumeric(raw)
        error('Unsupported data format in MAT file.');
    end

    data = raw;

    if size(data, 1) > size(data, 2) && size(data, 2) <= 32
        data = data.';
    end

    if size(data, 1) < 3
        error('Numeric data must contain at least time and current rows.');
    end

    t = data(1, :);
    current = data(3, :);

    if size(data, 1) >= 4
        voltage = data(4, :);
    end
end

function y = vector_from_timeseries_data(data, nTime)
    y = squeeze(data);

    if isvector(y)
        y = y(:).';
        return;
    end

    if size(y, 1) == nTime
        y = y(:, 1).';
    elseif size(y, 2) == nTime
        y = y(1, :);
    else
        y = y(:).';
        if numel(y) < nTime
            error('Timeseries data has fewer samples than its time vector.');
        end
        y = y(1:nTime);
    end
end

function isSim = is_simulation_label(label)
    isSim = ~isempty(strfind(lower(label), 'simulation'));
end

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
