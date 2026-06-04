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

col_meas = [1 0 0];

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
    plot(t, x_meas, 'Color', col_meas, 'LineWidth', 1.3);

    metrics = compute_system_metrics(t, x_meas, data(2, :), []);
    print_system_metrics(labels{k}, metrics);
end

xlabel('Time [s]');
ylabel('Measured position [m]');
title('Measured position comparison');
legend(labels, 'Interpreter', 'none', 'Location', 'best');

function metrics = compute_system_metrics(t, y, ref, inputSignal)
    t = t(:).';
    y = y(:).';

    if nargin < 3 || isempty(ref)
        ref = [];
    else
        ref = ref(:).';
    end

    if nargin < 4 || isempty(inputSignal)
        inputSignal = [];
    else
        inputSignal = inputSignal(:).';
    end

    metrics = struct('riseTime', NaN, 'staticGain', NaN, 'steadyStateError', NaN, ...
        'dominantPole', NaN, 'yInitial', NaN, 'yFinal', NaN);

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

    if ~isempty(ref) && numel(ref) == nSamples
        r0 = mean(ref(idxInitial), 'omitnan');
        rss = mean(ref(idxFinal), 'omitnan');
        stepAmp = rss - r0;
        metrics.steadyStateError = rss - yss;
    elseif ~isempty(inputSignal) && numel(inputSignal) == nSamples
        u0 = mean(inputSignal(idxInitial), 'omitnan');
        uss = mean(inputSignal(idxFinal), 'omitnan');
        stepAmp = uss - u0;
    else
        stepAmp = NaN;
    end

    responseAmp = yss - y0;
    if ~isnan(stepAmp) && abs(stepAmp) > eps
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
    fprintf('  Steady-state error: %s\n', format_metric(metrics.steadyStateError, '%.4g'));
    fprintf('  Dominant pole estimate: %s 1/s\n', format_metric(metrics.dominantPole, '%.4g'));
end

function textValue = format_metric(value, formatSpec)
    if isnan(value)
        textValue = 'N/A';
    else
        textValue = sprintf(formatSpec, value);
    end
end
