clc
clearvars -except filename accelAxis windowSec biasSec fc filterOrder x0
close all

%% User Configuration
if ~exist('filename', 'var') || isempty(filename)
    filename = "C:\Users\gupta\maarco\HardIceData\HardIce_01_26_60RPM_test1_u2.txt";
end

if ~exist('accelAxis', 'var') || isempty(accelAxis)
    accelAxis = "lin_accel_x";
end

if ~exist('windowSec', 'var')
    windowSec = [];
end

if ~exist('biasSec', 'var') || isempty(biasSec)
    biasSec = 2.0;
end

if ~exist('fc', 'var') || isempty(fc)
    fc = 2.0;
end

if ~exist('filterOrder', 'var') || isempty(filterOrder)
    filterOrder = 4;
end

if ~exist('x0', 'var') || isempty(x0)
    x0 = [0; 0];
end

%% Load IMU Data
data = load(filename);

timeSec = data(:, 1) / 1000;
timeSec = timeSec - timeSec(1);

switch char(accelAxis)
    case {'lin_accel_x', 'acc_lin_x'}
        accelRaw = data(:, 11);
    case {'lin_accel_y', 'acc_lin_y'}
        accelRaw = data(:, 12);
    case {'lin_accel_z', 'acc_lin_z'}
        accelRaw = data(:, 13);
    otherwise
        error('Unsupported accelAxis "%s". Use lin_accel_x, lin_accel_y, or lin_accel_z.', accelAxis);
end

%% Select Window
windowMask = true(size(timeSec));
if ~isempty(windowSec)
    if numel(windowSec) ~= 2 || windowSec(2) <= windowSec(1)
        error('windowSec must be a two-element vector [start end] with end > start.');
    end

    windowMask = timeSec >= windowSec(1) & timeSec <= windowSec(2);
    if nnz(windowMask) < 4
        error('The selected window does not contain enough IMU samples.');
    end
end

timexp = timeSec(windowMask);
accelWindow = accelRaw(windowMask);

%% Remove Bias
biasMask = timexp <= biasSec;
if nnz(biasMask) < 2
    biasMask = false(size(timexp));
    biasMask(1:min(numel(timexp), 5)) = true;
end

accelBias = mean(accelWindow(biasMask));
accelInput = accelWindow - accelBias;

%% Filter And Integrate
[accelFiltered, solverTime, solverState, sampleRateHz, cutoffHzUsed] = ...
    run_filter_ode45(timexp, accelInput, fc, filterOrder, x0);

displacementM = solverState(:, 1) - solverState(1, 1);
velocityMs = solverState(:, 2) - solverState(1, 2);

fprintf('\n--- Filter Configuration ---\n');
fprintf('File: %s\n', filename);
fprintf('Axis: %s\n', accelAxis);
fprintf('Selected Window: %.2f s -> %.2f s\n', timexp(1), timexp(end));
fprintf('Bias Removed: %.6f m/s^2\n', accelBias);
fprintf('Sample Rate: %.3f Hz\n', sampleRateHz);
fprintf('Cutoff Used: %.3f Hz\n', cutoffHzUsed);
fprintf('-----------------------------\n');

%% Plot Results
figure('Name', 'Butterworth Filter Comparison');
plot(timexp, accelWindow, 'Color', [0.7 0.7 0.7], 'DisplayName', 'Raw');
hold on;
plot(timexp, accelFiltered, 'r', 'LineWidth', 1.5, 'DisplayName', 'Filtered');
legend('Location', 'best');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
title(sprintf('%s Raw vs Filtered', accelAxis), 'Interpreter', 'none');
grid on;

figure('Name', 'Integrated IMU Response');
subplot(2, 1, 1);
plot(solverTime, displacementM, 'b', 'LineWidth', 1.5);
ylabel('Displacement (m)');
title('ODE45 Integrated Displacement');
grid on;

subplot(2, 1, 2);
plot(solverTime, velocityMs, 'm', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('ODE45 Integrated Velocity');
grid on;
