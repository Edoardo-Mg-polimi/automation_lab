function plot_LQR_time()
close all; clc;

scriptDir = fileparts(mfilename('fullpath'));
matFiles = dir(fullfile(scriptDir, '*.mat'));

if isempty(matFiles)
    error('No .mat files found in %s.', scriptDir);
end

[~, order] = sort({matFiles.name});
matFiles = matFiles(order);

simFile = fullfile('/Users', 'edoardo', 'Desktop', 'Scolastica', 'open', ...
    'automation_lab', 'matlab', '04_LQ_position', 'LQRI_NL_simulation_ramp.mat');

if ~isfile(simFile)
    error('Simulation file not found: %s', simFile);
end

simData = loadSimulationData(simFile);
validateSimulationData(simData, simFile);

colRef = [0 0 1];
colMeas = [1 0 0];
colSim = [0 0.6 0];
colDer = [0.1 0.35 0.9];
colKf = [0.75 0 0.75];

for k = 1:numel(matFiles)
    filePath = fullfile(matFiles(k).folder, matFiles(k).name);
    data = loadMeasuredData(filePath);
    validateMeasuredData(data, matFiles(k).name);

    [~, baseLabel] = fileparts(matFiles(k).name);

    t = data(1, :);
    xRef = data(2, :);
    xMeas = data(3, :);
    speedDerivativeFiltered = data(6, :);
    speedKf = data(7, :);
    iRef = data(8, :);
    iMeas = data(9, :);

    plotPosition(t, xRef, xMeas, simData, baseLabel, colRef, colMeas, colSim);
    plotPositionError(t, xRef, xMeas, data, baseLabel);
    plotCurrent(t, iRef, iMeas, baseLabel, colRef, colMeas);
    plotSpeed(t, speedKf, speedDerivativeFiltered, baseLabel, colKf, colDer);

    metrics = compute_system_metrics(t, xMeas, xRef, []);
    print_system_metrics(baseLabel, metrics);
end
end

function data = loadMeasuredData(filePath)
    S = load(filePath);
    data = findNumericMatrix(S);

    if isempty(data)
        error('No numeric matrix found in %s.', filePath);
    end

    data = normalizeDataMatrix(data, filePath);
end

function data = loadSimulationData(filePath)
    S = loadMatFile(filePath);

    data = extractSimulationMatrix(S);

    if isempty(data)
        data = extractOctaveMatlabRefsMatrix(S);
    end

    if isempty(data)
        error('No plottable simulation data found in %s.', filePath);
    end

    data = normalizeDataMatrix(data, filePath);
end

function S = loadMatFile(filePath)
    warningState = warning('query', 'all');
    warning('off', 'all');

    try
        S = load(filePath);
        warning(warningState);
    catch err
        warning(warningState);
        rethrow(err);
    end
end

function data = normalizeDataMatrix(data, filePath)
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

function data = extractSimulationMatrix(value)
    data = [];

    if isUsableNumericMatrix(value)
        data = value;
        return;
    end

    if isTimeseriesLike(value)
        data = timeseriesToMatrix(value);
        return;
    end

    if isSignalElementLike(value)
        data = extractSimulationMatrix(value.Values);
        return;
    end

    datasetData = extractSimulinkDatasetMatrix(value);
    if ~isempty(datasetData)
        data = datasetData;
        return;
    end

    if isstruct(value)
        names = fieldnames(value);
        for idx = 1:numel(names)
            if isInternalMatlabField(names{idx})
                continue;
            end

            candidate = extractSimulationMatrix(value.(names{idx}));
            if ~isempty(candidate)
                data = candidate;
                return;
            end
        end
    end
end

function tf = isUsableNumericMatrix(value)
    tf = isnumeric(value) && isfloat(value) && ismatrix(value) && ...
        ~isvector(value) && numel(value) > 1;
end

function tf = isInternalMatlabField(name)
    tf = strcmp(name, '__refs_') || strcmp(name, '__subsystem_') || ...
        strcmp(name, '__header__') || strcmp(name, '__version__') || ...
        strcmp(name, '__globals__');
end

function tf = isTimeseriesLike(value)
    tf = false;

    if ~(isobject(value) || isstruct(value))
        return;
    end

    try
        tf = isprop(value, 'Time') && isprop(value, 'Data');
    catch
        tf = false;
    end

    if ~tf && isstruct(value)
        tf = isfield(value, 'Time') && isfield(value, 'Data');
    end
end

function tf = isSignalElementLike(value)
    tf = false;

    if ~(isobject(value) || isstruct(value))
        return;
    end

    try
        tf = isprop(value, 'Values');
    catch
        tf = false;
    end

    if ~tf && isstruct(value)
        tf = isfield(value, 'Values');
    end
end

function data = timeseriesToMatrix(ts)
    if numel(ts) > 1
        data = combineTimeseriesArray(ts);
        return;
    end

    t = double(ts.Time(:).');
    y = double(ts.Data);

    if isempty(t) || isempty(y)
        data = [];
        return;
    end

    if ndims(y) > 2
        y = reshape(y, numel(t), []);
    end

    if isvector(y)
        signals = y(:).';
    elseif size(y, 1) == numel(t)
        signals = y.';
    elseif size(y, 2) == numel(t)
        signals = y;
    else
        error('Timeseries data dimensions do not match its time vector.');
    end

    data = [t; signals];
end

function data = combineTimeseriesArray(tsArray)
    data = [];
    signals = {};
    time = [];

    for idx = 1:numel(tsArray)
        candidate = timeseriesToMatrix(tsArray(idx));

        if isempty(candidate)
            continue;
        end

        if isempty(time)
            time = candidate(1, :);
        elseif numel(time) ~= size(candidate, 2) || any(abs(time - candidate(1, :)) > 1e-12)
            error('Timeseries array elements do not share the same time vector.');
        end

        signals{end + 1} = candidate(2:end, :); %#ok<AGROW>
    end

    if ~isempty(time) && ~isempty(signals)
        data = vertcat(time, signals{:});
    end
end

function data = extractSimulinkDatasetMatrix(value)
    data = [];

    if ~isobject(value) || ~ismethod(value, 'getElement')
        return;
    end

    hasNumElementsProperty = isprop(value, 'numElements');
    hasNumElementsMethod = ismethod(value, 'numElements');

    if ~(hasNumElementsProperty || hasNumElementsMethod)
        return;
    end

    try
        if hasNumElementsProperty
            nElements = value.numElements;
        else
            nElements = value.numElements();
        end
    catch
        return;
    end

    signals = {};
    time = [];

    for idx = 1:nElements
        element = value.getElement(idx);
        candidate = extractSimulationMatrix(element);

        if isempty(candidate)
            continue;
        end

        if isempty(time)
            time = candidate(1, :);
        elseif numel(time) ~= size(candidate, 2) || any(abs(time - candidate(1, :)) > 1e-12)
            continue;
        end

        signals{end + 1} = candidate(2:end, :); %#ok<AGROW>
    end

    if ~isempty(time) && ~isempty(signals)
        data = vertcat(time, signals{:});
    end
end

function data = extractOctaveMatlabRefsMatrix(S)
    data = [];

    if ~isstruct(S) || ~isfield(S, '__refs_')
        return;
    end

    refs = S.('__refs_');
    names = fieldnames(refs);
    matrices = {};
    vectors = {};

    for idx = 1:numel(names)
        value = refs.(names{idx});

        if ~isnumeric(value) || ~isfloat(value) || numel(value) <= 1
            continue;
        end

        if isvector(value)
            vectors{end + 1} = double(value(:).'); %#ok<AGROW>
        elseif ismatrix(value)
            matrices{end + 1} = double(value); %#ok<AGROW>
        end
    end

    for matrixIdx = 1:numel(matrices)
        matrixData = matrices{matrixIdx};

        for vectorIdx = 1:numel(vectors)
            time = vectors{vectorIdx};

            if numel(time) == size(matrixData, 2)
                data = [time; matrixData];
                return;
            end

            if numel(time) == size(matrixData, 1)
                data = [time; matrixData.'];
                return;
            end
        end
    end
end

function validateMeasuredData(data, fileName)
    if size(data, 1) < 11
        error(['Expected at least 11 rows in %s: time, position reference, ', ...
            'measured position, estimated position, derivative speed, ', ...
            'filtered derivative speed, KF speed, current reference, ', ...
            'measured current, voltage reference and position error.'], fileName);
    end
end

function validateSimulationData(data, fileName)
    if size(data, 1) < 3
        error('Expected at least time, position reference and simulated position in %s.', fileName);
    end
end

function plotPosition(t, xRef, xMeas, simData, baseLabel, colRef, colMeas, colSim)
    figure('Color', 'w', 'Name', [baseLabel ' - position']);
    hold on; grid on; box on;

    hRef = plot(t, xRef, 'Color', colRef, 'LineWidth', 1.4);
    hMeas = plot(t, xMeas, 'Color', colMeas, 'LineWidth', 1.3);
    hSim = plot(simData(1, :), simData(3, :), 'Color', colSim, 'LineWidth', 1.4);

    xlabel('Time [s]');
    ylabel('Position [m]');
    title(sprintf('LQR position - %s', baseLabel), 'Interpreter', 'none');
    legend([hRef hMeas hSim], {'Position reference', 'Measured position', ...
        'Simulated position'}, 'Interpreter', 'none', 'Location', 'northeast');
end

function plotPositionError(t, xRef, xMeas, data, baseLabel)
    figure('Color', 'w', 'Name', [baseLabel ' - position error']);
    hold on; grid on; box on;

    if size(data, 1) >= 11
        err = data(11, :);
    else
        err = xRef - xMeas;
    end

    hErr = plot(t, err, 'Color', [0 0 0.7], 'LineWidth', 1.4);

    xlabel('Time [s]');
    ylabel('Position error [m]');
    title(sprintf('LQR position error - %s', baseLabel), 'Interpreter', 'none');
    legend(hErr, {'Position error'}, 'Interpreter', 'none', 'Location', 'northeast');
end

function plotCurrent(t, iRef, iMeas, baseLabel, colRef, colMeas)
    figure('Color', 'w', 'Name', [baseLabel ' - current']);
    hold on; grid on; box on;

    hRef = plot(t, iRef, 'Color', colRef, 'LineWidth', 1.4);
    hMeas = plot(t, iMeas, 'Color', colMeas, 'LineWidth', 1.3);

    xlabel('Time [s]');
    ylabel('Current [A]');
    title(sprintf('LQR current - %s', baseLabel), 'Interpreter', 'none');
    legend([hRef hMeas], {'Current reference', 'Measured current'}, ...
        'Interpreter', 'none', 'Location', 'northeast');
end

function plotSpeed(t, speedKf, speedDerivativeFiltered, baseLabel, colKf, colDer)
    figure('Color', 'w', 'Name', [baseLabel ' - speed estimates']);
    hold on; grid on; box on;

    hKf = plot(t, speedKf, 'Color', colKf, 'LineWidth', 1.4);
    hDer = plot(t, speedDerivativeFiltered, 'Color', colDer, 'LineWidth', 1.3);

    xlabel('Time [s]');
    ylabel('Speed [m/s]');
    title(sprintf('LQR speed estimates - %s', baseLabel), 'Interpreter', 'none');
    legend([hKf hDer], {'KF estimated speed', 'Derivative-filter estimated speed'}, ...
        'Interpreter', 'none', 'Location', 'northeast');
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
