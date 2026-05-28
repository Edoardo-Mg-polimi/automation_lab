clear; close all; clc;

scriptDir = fileparts(mfilename('fullpath'));
matFiles = dir(fullfile(scriptDir, '*.mat'));

if isempty(matFiles)
    error('No .mat files found in %s', scriptDir);
end

[~, order] = sort({matFiles.name});
matFiles = matFiles(order);

col_ref = [0 0.4470 0.7410];
col_meas = [0.8500 0.3250 0.0980];

for k = 1:numel(matFiles)
    filePath = fullfile(scriptDir, matFiles(k).name);
    S = load(filePath);
    varName = fieldnames(S);
    data = S.(varName{1});

    if size(data, 1) > size(data, 2) && size(data, 2) <= 32
        data = data.';
    end

    if size(data, 1) < 3
        error('Expected at least a time row and two signals in %s', matFiles(k).name);
    end

    t = data(1, :);
    baseLabel = erase(matFiles(k).name, '.mat');

    figure('Color', 'w', 'Name', baseLabel);
    hold on;
    grid on;

    plot(t, data(2, :), 'Color', col_ref, 'LineWidth', 1.3, ...
        'DisplayName', 'Reference');
    plot(t, data(3, :), 'Color', col_meas, 'LineWidth', 1.3, ...
        'DisplayName', 'Measured');

    xlabel('Time [s]');
    ylabel('Current [A]');
    title(sprintf('Current validation - %s', baseLabel));
    legend('show', 'Interpreter', 'none', 'Location', 'best');
    hold off;
end