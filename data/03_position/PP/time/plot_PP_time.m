clear; close all; clc;

scriptDir = fileparts(mfilename('fullpath'));
matFiles = dir(fullfile(scriptDir, '*.mat'));

if isempty(matFiles)
    error('No .mat files found in %s.', scriptDir);
end

[~, order] = sort({matFiles.name});
matFiles = matFiles(order);

specialFileName = 'PP_validation_poles_3_70_75_with_integral_sat.mat';
simFile = fullfile(scriptDir, '..', '..', '..', '..', ...
    'matlab', '03_PP_position', 'PPI_NL_simulation_ramp.mat');

col_ref = [0 0 1];
col_meas = [1 0 0];
col_sim = [0 0.6 0];

for k = 1:numel(matFiles)
    filePath = fullfile(matFiles(k).folder, matFiles(k).name);
    data = loadNumericMatrix(filePath);
    baseLabel = erase(matFiles(k).name, '.mat');

    validateMeasuredData(data, matFiles(k).name);

    t = data(1, :);
    x_ref = data(2, :);
    x_meas = data(3, :);
    speed_filt = data(6, :);
    speed_obs = data(7, :);
    i_ref = data(8, :);
    i_meas = data(9, :);

    simData = [];
    if strcmp(matFiles(k).name, specialFileName)
        if ~isfile(simFile)
            warning('Simulation file not found: %s', simFile);
        else
            simData = loadNumericMatrix(simFile);
            validateSimulationData(simData, simFile);
        end
    end

    plotPosition(t, x_ref, x_meas, simData, baseLabel, col_ref, col_meas, col_sim);
    plotPositionError(t, x_ref, x_meas, baseLabel, col_ref, col_meas);
    plotCurrent(t, i_ref, i_meas, baseLabel, col_ref, col_meas);
    plotObserverSpeed(t, speed_filt, speed_obs, baseLabel, col_ref, col_meas);

    metrics = compute_system_metrics(t, x_meas, x_ref, []);
    print_system_metrics(baseLabel, metrics);
end

function data = loadNumericMatrix(filePath)
    S = load(filePath);
    data = findNumericMatrix(S);

    if isempty(data)
        error('No numeric matrix found in %s.', filePath);
    end

    data = double(data);

    if isvector(data)
        error('Expected a 2-D numeric matrix in %s.', filePath);
    end

    if size(data, 1) > size(data, 2) && size(data, 2) <= 32
        data = data.';
    end
end

function data = findNumericMatrix(value)
    data = [];

    if isnumeric(value) && ismatrix(value) && numel(value) > 1
        data = value;
        return;
    end

    if isstruct(value)
        names = fieldnames(value);
        for idx = 1:numel(names)
            candidate = findNumericMatrix(value.(names{idx}));
            if ~isempty(candidate)
                data = candidate;
                return;
            end
        end
    end
end

function validateMeasuredData(data, fileName)
    if size(data, 1) < 9
        error(['Expected at least 9 rows in %s: time, position reference, ', ...
            'measured position, speed signals and currents.'], fileName);
    end
end

function validateSimulationData(data, fileName)
    if size(data, 1) < 3
        error('Expected at least time, position reference and simulated position in %s.', fileName);
    end
end

function plotPosition(t, x_ref, x_meas, simData, baseLabel, col_ref, col_meas, col_sim)
    figure('Color', 'w', 'Name', [baseLabel ' - position']);
    hold on; grid on; box on;

    plot(t, x_ref, 'Color', col_ref, 'LineWidth', 1.4, ...
        'DisplayName', 'Position reference');
    plot(t, x_meas, 'Color', col_meas, 'LineWidth', 1.3, ...
        'DisplayName', 'Measured position');

    if ~isempty(simData)
        plot(simData(1, :), simData(3, :), '--', 'Color', col_sim, 'LineWidth', 1.5, ...
            'DisplayName', 'Simulated position');
    end

    xlabel('Time [s]');
    ylabel('Position [m]');
    title(sprintf('PP position - %s', baseLabel), 'Interpreter', 'none');
    legend('show', 'Interpreter', 'none', 'Location', 'best');
end

function plotCurrent(t, i_ref, i_meas, baseLabel, col_ref, col_meas)
    figure('Color', 'w', 'Name', [baseLabel ' - current']);
    hold on; grid on; box on;

    plot(t, i_ref, 'Color', col_ref, 'LineWidth', 1.4, ...
        'DisplayName', 'Current reference');
    plot(t, i_meas, 'Color', col_meas, 'LineWidth', 1.3, ...
        'DisplayName', 'Measured current');

    xlabel('Time [s]');
    ylabel('Current [A]');
    title(sprintf('PP current - %s', baseLabel), 'Interpreter', 'none');
    legend('show', 'Interpreter', 'none', 'Location', 'best');
end

function plotObserverSpeed(t, speed_filt, speed_obs, baseLabel, col_ref, col_meas)
    figure('Color', 'w', 'Name', [baseLabel ' - observer speed']);
    hold on; grid on; box on;

    plot(t, speed_filt, 'Color', col_ref, 'LineWidth', 1.4, ...
        'DisplayName', 'Filtered derivative speed');
    plot(t, speed_obs, 'Color', col_meas, 'LineWidth', 1.3, ...
        'DisplayName', 'Observer speed');

    xlabel('Time [s]');
    ylabel('Speed [m/s]');
    title(sprintf('PP observer speed - %s', baseLabel), 'Interpreter', 'none');
    legend('show', 'Interpreter', 'none', 'Location', 'best');
end

function plotPositionError(t, x_ref, x_meas, baseLabel, col_ref, col_meas)
    figure('Color', 'w', 'Name', [baseLabel ' - position error']);
    hold on; grid on; box on;

    err = x_ref - x_meas;
    plot(t, err, 'Color', [0 0 0.7], 'LineWidth', 1.4, 'DisplayName', 'Position error');

    xlabel('Time [s]');
    ylabel('Position error [m]');
    title(sprintf('PP position error - %s', baseLabel), 'Interpreter', 'none');
    legend('show', 'Interpreter', 'none', 'Location', 'best');
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
