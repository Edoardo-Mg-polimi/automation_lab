clear; close all; clc;

scriptDir = fileparts(mfilename('fullpath'));
matFiles = dir(fullfile(scriptDir, '*.mat'));

if isempty(matFiles)
    error('No .mat files found in %s', scriptDir);
end

[~, order] = sort({matFiles.name});
matFiles = matFiles(order);

for k = 1:numel(matFiles)
    filePath = fullfile(matFiles(k).folder, matFiles(k).name);
    [t, signals] = loadCalibrationMatrix(filePath);
    plotCalibrationMatrix(t, signals, erase(matFiles(k).name, '.mat'));
end

function [t, signals] = loadCalibrationMatrix(filePath)
    S = load(filePath);
    varNames = fieldnames(S);
    data = S.(varNames{1});

    if size(data, 1) > size(data, 2) && size(data, 2) <= 32
        data = data.';
    end

    if size(data, 1) < 2
        error('Expected at least one time row and one signal row in %s.', filePath);
    end

    t = double(data(1, :));
    signals = double(data(2:end, :));
end

function plotCalibrationMatrix(t, signals, plotName)
    nSignals = size(signals, 1);

    figure('Color', 'w', 'Name', plotName);

    for idx = 1:nSignals
        subplot(nSignals, 1, idx);
        plot(t, signals(idx, :), 'LineWidth', 1.3);
        grid on;
        ylabel(sprintf('Signal %d', idx));

        if idx == 1
            title(sprintf('RL calibration - %s', plotName));
        end

        if idx == nSignals
            xlabel('Time [s]');
        end
    end
end