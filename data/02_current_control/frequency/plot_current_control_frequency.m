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

    figure('Color', 'w', 'Name', erase(matFiles(fileIdx).name, '.mat'));
    plotValidationSeries(t, signals, labels, sprintf('Current control frequency validation - %s', erase(matFiles(fileIdx).name, '.mat')));
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

    plot(t, signals(1, :), 'LineWidth', 1.3, 'Color', [0 0.4470 0.7410]);
    hold on;
    plot(t, signals(2, :), 'LineWidth', 1.3, 'Color', [0.8500 0.3250 0.0980]);
    grid on;
    title(plotTitle);
    xlabel('Time [s]');
    ylabel('Current');
    legend({labels{1}, labels{2}}, 'Location', 'best');
    hold off;
end