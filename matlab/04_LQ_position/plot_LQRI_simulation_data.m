function plot_LQRI_simulation_data()
close all; clc;

scriptDir = fileparts(mfilename('fullpath'));
matFiles = dir(fullfile(scriptDir, 'LQRI_NL_simulation_*.mat'));

if isempty(matFiles)
    error('No LQRI_NL_simulation_*.mat files found in %s.', scriptDir);
end

[~, order] = sort({matFiles.name});
matFiles = matFiles(order);

for k = 1:numel(matFiles)
    filePath = fullfile(matFiles(k).folder, matFiles(k).name);
    data = loadSimulationData(filePath);
    [~, plotName] = fileparts(matFiles(k).name);

    plotMainSimulation(data, plotName, 'LQRI');
    plotAllSignals(data, plotName, 'LQRI');
end
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

    data = normalizeSimulationMatrix(data, filePath);
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

function data = normalizeSimulationMatrix(data, filePath)
    data = double(data);

    if isvector(data)
        error('Expected a matrix with time and signal rows in %s.', filePath);
    end

    if size(data, 1) > size(data, 2) && size(data, 2) <= 32
        data = data.';
    end

    if size(data, 1) < 2
        error('Expected at least time and one signal row in %s.', filePath);
    end
end

function plotMainSimulation(data, plotName, controllerName)
    t = data(1, :);
    signals = data(2:end, :);

    figure('Color', 'w', 'Name', [plotName ' main']);
    hold on;
    grid on;

    colRef = [0 0 1];
    colSim = [0 0.6 0];

    plot(t, signals(1, :), 'Color', colRef, 'LineWidth', 1.4, ...
        'DisplayName', 'Position reference');

    if size(signals, 1) >= 2
        plot(t, signals(2, :), 'Color', colSim, 'LineWidth', 1.4, ...
            'DisplayName', 'Position response');
    end

    xlabel('Time [s]');
    ylabel('Position [m]');
    title(sprintf('%s nonlinear simulation - %s', controllerName, plotName), 'Interpreter', 'none');
    legend('show', 'Location', 'best');
end

function plotAllSignals(data, plotName, controllerName)
    t = data(1, :);
    signals = data(2:end, :);
    nSignals = size(signals, 1);

    figure('Color', 'w', 'Name', [plotName ' all signals']);

    for idx = 1:nSignals
        subplot(nSignals, 1, idx);
        plot(t, signals(idx, :), 'Color', simulationSignalColor(idx), 'LineWidth', 1.2);
        grid on;
        ylabel(signalLabel(idx));

        if idx == 1
            title(sprintf('%s simulation signals - %s', controllerName, plotName), 'Interpreter', 'none');
        end

        if idx == nSignals
            xlabel('Time [s]');
        end
    end
end

function col = simulationSignalColor(idx)
    colRef = [0 0 1];
    colSim = [0 0.6 0];
    colDefault = [0 0 0];

    if any(idx == [1 3 5])
        col = colRef;
    elseif idx == 8
        col = colDefault;
    else
        col = colSim;
    end
end

function label = signalLabel(idx)
    labels = {
        'x ref'
        'x'
        'i ref'
        'i'
        'v ref'
        'dx'
        'dx estimated'
        'error'
    };

    if idx <= numel(labels)
        label = labels{idx};
    else
        label = sprintf('Signal %d', idx);
    end
end
