clear; close all; clc;

scriptDir = fileparts(mfilename('fullpath'));
matFiles = dir(fullfile(scriptDir, 'LQRI_NL_simulation_*.mat'));

if isempty(matFiles)
    error('No LQRI_NL_simulation_*.mat files found in %s.', scriptDir);
end

[~, order] = sort({matFiles.name});
matFiles = matFiles(order);

for k = 1:numel(matFiles)
    filePath = fullfile(matFiles(k).folder, matFiles(k).name);
    data = loadSimulationMatrix(filePath);
    plotMainSimulation(data, erase(matFiles(k).name, '.mat'), 'LQRI');
    plotAllSignals(data, erase(matFiles(k).name, '.mat'), 'LQRI');
end

function data = loadSimulationMatrix(filePath)
    S = load(filePath);
    data = findNumericMatrix(S);

    if isempty(data)
        error('No numeric matrix found in %s.', filePath);
    end

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

function plotMainSimulation(data, plotName, controllerName)
    t = data(1, :);
    signals = data(2:end, :);

    figure('Color', 'w', 'Name', [plotName ' main']);
    hold on;
    grid on;

    plot(t, signals(1, :), 'LineWidth', 1.4, 'DisplayName', 'Position reference');

    if size(signals, 1) >= 2
        plot(t, signals(2, :), 'LineWidth', 1.4, 'DisplayName', 'Position response');
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
    tiledlayout(nSignals, 1, 'TileSpacing', 'compact');

    for idx = 1:nSignals
        nexttile;
        plot(t, signals(idx, :), 'LineWidth', 1.2);
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

function label = signalLabel(idx)
    labels = {
        'x ref'
        'x'
        'x estimated'
        'dx'
        'dx filtered'
        'dx estimated'
        'i ref'
        'i'
        'v ref'
        'error'
    };

    if idx <= numel(labels)
        label = labels{idx};
    else
        label = sprintf('Signal %d', idx);
    end
end
