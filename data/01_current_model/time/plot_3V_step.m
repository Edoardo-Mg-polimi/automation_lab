clear; close all; clc;

scriptDir = fileparts(mfilename('fullpath'));

realFile = fullfile(scriptDir, '3V.mat');
simFile = fullfile(scriptDir, 'NL_simulation_3V.mat');

if ~isfile(realFile)
    error('Real data file not found: %s', realFile);
end

if ~isfile(simFile)
    error('Simulation data file not found: %s', simFile);
end

realMat = load(realFile);
simMat = load(simFile);

[tReal, iReal, vReal] = extract_current_data(realMat, '');
[tSim, iSim, ~] = extract_current_data(simMat, 'current');

tReal = tReal(:);
iReal = iReal(:);
tSim = tSim(:);
iSim = iSim(:);

if ~isempty(vReal)
    vReal = vReal(:);
end

% Align both traces at the step start to make the transient comparable.
t0Real = estimate_step_time(tReal, iReal, vReal);
t0Sim = estimate_step_time(tSim, iSim, []);
tReal = tReal - t0Real;
tSim = tSim - t0Sim;

[tReal, iReal] = sort_signal(tReal, iReal);
[tSim, iSim] = sort_signal(tSim, iSim);

validReal = isfinite(tReal) & isfinite(iReal);
validSim = isfinite(tSim) & isfinite(iSim);
tReal = tReal(validReal);
iReal = iReal(validReal);
tSim = tSim(validSim);
iSim = iSim(validSim);

tMin = max(min(tReal), min(tSim));
tMax = min(max(tReal), max(tSim));

if tMax <= tMin
    error('Real and simulation time vectors do not overlap after alignment.');
end

idxCompare = tReal >= tMin & tReal <= tMax;
tCompare = tReal(idxCompare);
iRealCompare = iReal(idxCompare);
iSimCompare = interp1(tSim, iSim, tCompare, 'linear');
errCompare = iSimCompare - iRealCompare;

rmse = sqrt(mean(errCompare.^2, 'omitnan'));
mae = mean(abs(errCompare), 'omitnan');
maxAbsError = max(abs(errCompare), [], 'omitnan');
finalWindow = max(3, round(0.1 * numel(errCompare)));
idxFinal = max(1, numel(errCompare) - finalWindow + 1):numel(errCompare);
finalError = mean(errCompare(idxFinal), 'omitnan');

[riseTimeReal, tauReal, poleReal] = estimate_step_metrics(tReal, iReal);
[riseTimeSim, tauSim, poleSim] = estimate_step_metrics(tSim, iSim);

riseRealStr = format_metric(riseTimeReal, 's');
tauRealStr = format_metric(tauReal, 's');
poleRealStr = format_metric(poleReal, 'rad/s');
riseSimStr = format_metric(riseTimeSim, 's');
tauSimStr = format_metric(tauSim, 's');
poleSimStr = format_metric(poleSim, 'rad/s');

fig = figure('Name', '3V current step - real vs simulation');
set(fig, 'Color', 'w');

col_meas = [1 0 0];
col_sim = [0 0.6 0];

hold on; grid on; box on;
plot(tReal, iReal, 'Color', col_meas, 'LineWidth', 1.4, 'DisplayName', 'Real data');
plot(tSim, iSim, '--', 'Color', col_sim, 'LineWidth', 1.6, 'DisplayName', 'Simulation');
xline(0, ':', 'Step', 'HandleVisibility', 'off');
xlabel('Time from step [s]');
ylabel('Current [A]');
title('3 V step current: real data vs simulation');
legend('show', 'Location', 'best', 'Interpreter', 'none');

fprintf('\n3 V current step comparison\n');
fprintf('  Real file: %s\n', realFile);
fprintf('  Simulation file: %s\n', simFile);
fprintf('  Compared interval: %.6g s to %.6g s\n', tMin, tMax);
fprintf('  RMSE: %.6g A\n', rmse);
fprintf('  MAE: %.6g A\n', mae);
fprintf('  Max abs error: %.6g A\n', maxAbsError);
fprintf('  Final mean error: %.6g A\n', finalError);
fprintf('  Rise time (real): %s\n', riseRealStr);
fprintf('  Tau (real): %s\n', tauRealStr);
fprintf('  Pole (real): %s\n', poleRealStr);
fprintf('  Rise time (sim): %s\n', riseSimStr);
fprintf('  Tau (sim): %s\n', tauSimStr);
fprintf('  Pole (sim): %s\n', poleSimStr);

function [t, current, voltage] = extract_current_data(S, preferredName)
    voltage = [];

    if nargin >= 2 && ~isempty(preferredName) && isfield(S, preferredName)
        raw = S.(preferredName);
    else
        names = fieldnames(S);
        raw = S.(names{1});
    end

    if isa(raw, 'timeseries')
        t = raw.Time;
        current = vector_from_timeseries_data(raw.Data, numel(t));
        return;
    end

    if isstruct(raw)
        if isfield(raw, 'Time') && isfield(raw, 'Data')
            t = raw.Time;
            current = vector_from_timeseries_data(raw.Data, numel(t));
            return;
        end

        if isfield(raw, 'time') && isfield(raw, 'current')
            t = raw.time;
            current = raw.current;
            if isfield(raw, 'voltage')
                voltage = raw.voltage;
            end
            return;
        end
    end

    if isnumeric(raw)
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
        return;
    end

    error('Unsupported data format in MAT file.');
end

function y = vector_from_timeseries_data(data, nTime)
    y = squeeze(data);

    if isvector(y)
        y = y(:);
        return;
    end

    if size(y, 1) == nTime
        y = y(:, 1);
    elseif size(y, 2) == nTime
        y = y(1, :).';
    else
        y = y(:);
        if numel(y) < nTime
            error('Timeseries data has fewer samples than its time vector.');
        end
        y = y(1:nTime);
    end
end

function tStep = estimate_step_time(t, current, voltage)
    if nargin >= 3 && ~isempty(voltage) && numel(voltage) == numel(t)
        signal = voltage(:);
    else
        signal = current(:);
    end

    t = t(:);
    signal = signal(:);
    signal = signal - median(signal(1:max(1, round(0.05 * numel(signal)))), 'omitnan');
    spanValue = max(signal, [], 'omitnan') - min(signal, [], 'omitnan');

    if ~isfinite(spanValue) || spanValue <= eps
        tStep = t(1);
        return;
    end

    threshold = min(signal, [], 'omitnan') + 0.1 * spanValue;
    if abs(max(signal, [], 'omitnan')) < abs(min(signal, [], 'omitnan'))
        threshold = max(signal, [], 'omitnan') - 0.1 * spanValue;
        idx = find(signal <= threshold, 1, 'first');
    else
        idx = find(signal >= threshold, 1, 'first');
    end

    if isempty(idx)
        tStep = t(1);
    else
        tStep = t(idx);
    end
end

function [riseTime, tau, pole] = estimate_step_metrics(t, y)
    t = t(:);
    y = y(:);
    valid = isfinite(t) & isfinite(y);
    t = t(valid);
    y = y(valid);

    if numel(t) < 5
        riseTime = NaN;
        tau = NaN;
        pole = NaN;
        return;
    end

    [t, y] = sort_signal(t, y);

    if any(t <= 0)
        idxBase = t <= 0;
    else
        idxBase = 1:max(1, round(0.05 * numel(t)));
    end

    base = mean(y(idxBase), 'omitnan');
    idxFinal = max(1, numel(t) - max(3, round(0.1 * numel(t))) + 1):numel(t);
    final = mean(y(idxFinal), 'omitnan');
    delta = final - base;

    if ~isfinite(delta) || abs(delta) <= eps
        riseTime = NaN;
        tau = NaN;
        pole = NaN;
        return;
    end

    level10 = base + 0.1 * delta;
    level90 = base + 0.9 * delta;

    if delta >= 0
        idx10 = find(y >= level10, 1, 'first');
        idx90 = find(y >= level90, 1, 'first');
    else
        idx10 = find(y <= level10, 1, 'first');
        idx90 = find(y <= level90, 1, 'first');
    end

    if isempty(idx10) || isempty(idx90) || idx90 <= idx10
        riseTime = NaN;
        tau = NaN;
        pole = NaN;
        return;
    end

    riseTime = t(idx90) - t(idx10);
    tau = riseTime / log(9);

    if ~isfinite(tau) || tau <= 0
        pole = NaN;
    else
        pole = -1 / tau;
    end
end

function str = format_metric(value, unit)
    if ~isfinite(value)
        str = 'n/a';
    else
        str = sprintf('%.6g %s', value, unit);
    end
end

function [tSorted, ySorted] = sort_signal(t, y)
    [tSorted, order] = sort(t(:));
    ySorted = y(order);

    [tSorted, uniqueIdx] = unique(tSorted, 'stable');
    ySorted = ySorted(uniqueIdx);
end
