function [filteredSignal, solverTime, solverState, sampleRateHz, cutoffHzUsed] = ...
    run_filter_ode45(timeVec, accelSignal, cutoffHz, filterOrder, initialState)

if nargin < 5 || isempty(initialState)
    initialState = [0; 0];
end

if nargin < 4 || isempty(filterOrder)
    filterOrder = 4;
end

if nargin < 3 || isempty(cutoffHz)
    cutoffHz = 2.0;
end

timeVec = timeVec(:);
accelSignal = accelSignal(:);

validMask = isfinite(timeVec) & isfinite(accelSignal);
timeVec = timeVec(validMask);
accelSignal = accelSignal(validMask);

[timeVec, sortIdx] = sort(timeVec);
accelSignal = accelSignal(sortIdx);

[timeVec, uniqueIdx] = unique(timeVec, 'stable');
accelSignal = accelSignal(uniqueIdx);

if numel(timeVec) < 4
    error('run_filter_ode45 requires at least four valid samples.');
end

dtAll = diff(timeVec);
dtPositive = dtAll(dtAll > 0);

if isempty(dtPositive)
    error('Could not infer a valid sample rate from the provided time vector.');
end

sampleRateHz = 1 / mean(dtPositive);
nyquistHz = sampleRateHz / 2;
cutoffHzUsed = cutoffHz;

if cutoffHzUsed <= 0 || cutoffHzUsed >= nyquistHz
    cutoffHzUsed = 0.99 * nyquistHz;
    warning('Requested cutoff %.3f Hz is invalid for Fs = %.3f Hz. Using %.3f Hz instead.', ...
        cutoffHz, sampleRateHz, cutoffHzUsed);
end

Wn = cutoffHzUsed / nyquistHz;
[bFilt, aFilt] = butter(filterOrder, Wn, 'low');
filteredSignal = filtfilt(bFilt, aFilt, accelSignal);

tspan = [timeVec(1), timeVec(end)];
[solverTime, solverState] = ode45(@(t, x) accel_integrator(t, x, filteredSignal, timeVec), tspan, initialState);

end

function dxdt = accel_integrator(t, x, accx, timexp)
% x(1) is position
% x(2) is velocity
g = 9.81; %#ok<NASGU>
L = 1.0; %#ok<NASGU>
b = 0.5; %#ok<NASGU>
accx = interpn(timexp, accx, t);

dxdt = zeros(2, 1);
dxdt(1) = x(2);
dxdt(2) = accx;
end
