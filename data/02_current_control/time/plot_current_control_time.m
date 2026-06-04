clear; close all; clc;

scriptDir = fileparts(mfilename('fullpath'));

files = {
    fullfile(scriptDir, 'CurrentContr_timevalid.mat')
};

labels = {
    'Current validation'
};

figI = figure;
figure(figI); hold on; grid on;

figV = figure;
figure(figV); hold on; grid on;

for k = 1:length(files)
    S = load(files{k});
    varName = fieldnames(S);
    data = S.(varName{1});

    if size(data, 1) > size(data, 2) && size(data, 2) <= 32
        data = data.';
    end

    t = data(1, :);
    i_meas = data(3, :);
    % colors: reference = blue, measured = red
    col_ref = [0 0 1];
    col_meas = [1 0 0];
    col_vol = col_meas;

    refPlotted = false;
    % plot currents on currents figure
    figure(figI);
    if size(data, 1) >= 2
        i_ref = data(2, :);
        plot(t, i_ref, 'Color', col_ref, 'LineWidth', 1.3);
        refPlotted = true;

    metrics = compute_system_metrics(t, data(3, :), data(2, :), data(4, :));
    print_system_metrics(labels{k}, metrics);
    end
    plot(t, i_meas, 'Color', col_meas, 'LineWidth', 1.3);

    % plot voltage (row 4) on voltage figure if present
    if size(data, 1) >= 4
        v = data(4, :);
        figure(figV);
        plot(t, v, 'Color', col_vol, 'LineWidth', 1.3);
    end
end

% finalize currents figure
figure(figI);
xlabel('Time [s]');
ylabel('Measured current [A]');
title('Measured current comparison');
% build legend: include reference if present
if exist('refPlotted', 'var') && refPlotted
    legendEntries = [{'Reference'}, labels];
else
    legendEntries = labels;
end
legend(legendEntries, 'Interpreter', 'none', 'Location', 'best');

% finalize voltage figure
figure(figV);
xlabel('Time [s]');
ylabel('Voltage [V]');
title('Voltage vs Time');
if length(files) > 0
    legend(labels, 'Interpreter', 'none', 'Location', 'best');
end

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
