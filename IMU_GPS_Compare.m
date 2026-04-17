clc
clearvars -except sensorFile gpsFile windowSec accelAxis biasSec fc filterOrder x0 forcePairedFiles
close all

%% User Configuration
defaultGpsFile = "field_tests/volleyball/12_5_25/straight_line_1ft_height_gps.csv";
defaultSensorFile = paired_sensor_file(defaultGpsFile);
defaultWindowSec = [52 108];

if ~exist('forcePairedFiles', 'var') || isempty(forcePairedFiles)
    forcePairedFiles = true;
end

hasGpsFile = exist('gpsFile', 'var') && ~isempty(gpsFile);
hasSensorFile = exist('sensorFile', 'var') && ~isempty(sensorFile);

if ~hasGpsFile && ~hasSensorFile
    gpsFile = defaultGpsFile;
    sensorFile = defaultSensorFile;
elseif hasGpsFile && ~hasSensorFile
    sensorFile = paired_sensor_file(gpsFile);
elseif ~hasGpsFile && hasSensorFile
    gpsFile = paired_gps_file(sensorFile);
elseif string(sensorFile) == defaultSensorFile && string(gpsFile) ~= defaultGpsFile
    % Common command-window workflow: set gpsFile only and let this script pair it.
    sensorFile = paired_sensor_file(gpsFile);
elseif forcePairedFiles
    expectedSensorFile = paired_sensor_file(gpsFile);
    if ~same_file_run(sensorFile, expectedSensorFile)
        warning(['sensorFile "%s" does not match gpsFile "%s". Using inferred paired sensor file "%s". ' ...
            'Set forcePairedFiles = false before running if you intentionally want to compare different files.'], ...
            sensorFile, gpsFile, expectedSensorFile);
        sensorFile = expectedSensorFile;
    end
end

if ~exist('windowSec', 'var')
    if string(gpsFile) == defaultGpsFile && string(sensorFile) == defaultSensorFile
        windowSec = defaultWindowSec;
    else
        windowSec = [];
    end
end

if ~exist('accelAxis', 'var') || isempty(accelAxis)
    accelAxis = "acc_lin_x";
end

switch char(accelAxis)
    case 'lin_accel_x'
        accelAxis = "acc_lin_x";
    case 'lin_accel_y'
        accelAxis = "acc_lin_y";
    case 'lin_accel_z'
        accelAxis = "acc_lin_z";
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

%% Load Sensor And GPS Tables
sensorTable = read_clean_table(sensorFile);
gpsTable = read_clean_table(gpsFile);

sensorTimestampNs = numeric_column(sensorTable, "timestamp_ns");
sensorAccel = numeric_column(sensorTable, accelAxis);

gpsTimestampNs = numeric_column(gpsTable, "timestamp_ns");
gpsLatitude = numeric_column(gpsTable, "latitude");
gpsLongitude = numeric_column(gpsTable, "longitude");

validSensor = ~isnan(sensorTimestampNs) & ~isnan(sensorAccel);
validGps = ~isnan(gpsTimestampNs) & ~isnan(gpsLatitude) & ~isnan(gpsLongitude);
rawSensorCount = numel(sensorTimestampNs);
rawGpsCount = numel(gpsTimestampNs);

sensorTimestampNs = sensorTimestampNs(validSensor);
sensorAccel = sensorAccel(validSensor);

gpsTimestampNs = gpsTimestampNs(validGps);
gpsLatitude = gpsLatitude(validGps);
gpsLongitude = gpsLongitude(validGps);

if numel(sensorTimestampNs) < 4
    error('Sensor file "%s" only has %d valid samples out of %d rows. At least 4 are required.', ...
        sensorFile, numel(sensorTimestampNs), rawSensorCount);
end

if numel(gpsTimestampNs) < 4
    error('GPS file "%s" only has %d valid latitude/longitude samples out of %d rows. At least 4 are required.', ...
        gpsFile, numel(gpsTimestampNs), rawGpsCount);
end

[sensorTimestampNs, sensorOrder] = sort(sensorTimestampNs);
sensorAccel = sensorAccel(sensorOrder);

[gpsTimestampNs, gpsOrder] = sort(gpsTimestampNs);
gpsLatitude = gpsLatitude(gpsOrder);
gpsLongitude = gpsLongitude(gpsOrder);

%% Find Overlap
overlapStartNs = max(sensorTimestampNs(1), gpsTimestampNs(1));
overlapEndNs = min(sensorTimestampNs(end), gpsTimestampNs(end));

if overlapEndNs <= overlapStartNs
    error(['No overlap exists between the selected sensor and GPS files.\n' ...
        'Sensor file: %s\nSensor timestamp range: %.0f -> %.0f\n' ...
        'GPS file: %s\nGPS timestamp range: %.0f -> %.0f\n' ...
        'This usually means sensorFile and gpsFile are from different runs.'], ...
        sensorFile, sensorTimestampNs(1), sensorTimestampNs(end), ...
        gpsFile, gpsTimestampNs(1), gpsTimestampNs(end));
end

sensorOverlapMask = sensorTimestampNs >= overlapStartNs & sensorTimestampNs <= overlapEndNs;
gpsOverlapMask = gpsTimestampNs >= overlapStartNs & gpsTimestampNs <= overlapEndNs;

sensorTimeOverlap = (sensorTimestampNs(sensorOverlapMask) - overlapStartNs) / 1e9;
sensorAccelOverlap = sensorAccel(sensorOverlapMask);

gpsTimeOverlap = (gpsTimestampNs(gpsOverlapMask) - overlapStartNs) / 1e9;
gpsLatitudeOverlap = gpsLatitude(gpsOverlapMask);
gpsLongitudeOverlap = gpsLongitude(gpsOverlapMask);

%% Apply Manual Window
overlapDurationSec = min(sensorTimeOverlap(end), gpsTimeOverlap(end));
if isempty(windowSec)
    windowSec = [0 overlapDurationSec];
elseif numel(windowSec) ~= 2 || windowSec(2) <= windowSec(1)
    error('windowSec must be empty or a two-element vector [start end] with end > start.');
end

if windowSec(1) < 0 || windowSec(2) > overlapDurationSec
    error('windowSec must stay inside the overlap duration of %.2f seconds.', overlapDurationSec);
end

sensorWindowMask = sensorTimeOverlap >= windowSec(1) & sensorTimeOverlap <= windowSec(2);
gpsWindowMask = gpsTimeOverlap >= windowSec(1) & gpsTimeOverlap <= windowSec(2);

if nnz(sensorWindowMask) < 4
    error('The selected window does not contain enough sensor samples.');
end

if nnz(gpsWindowMask) < 4
    error('The selected window does not contain enough GPS samples.');
end

sensorTimeWindow = sensorTimeOverlap(sensorWindowMask) - windowSec(1);
sensorAccelWindow = sensorAccelOverlap(sensorWindowMask);

gpsTimeWindow = gpsTimeOverlap(gpsWindowMask) - windowSec(1);
gpsLatitudeWindow = gpsLatitudeOverlap(gpsWindowMask);
gpsLongitudeWindow = gpsLongitudeOverlap(gpsWindowMask);

%% Filter And Integrate IMU Acceleration
biasMask = sensorTimeWindow <= biasSec;
if nnz(biasMask) < 2
    biasMask = false(size(sensorTimeWindow));
    biasMask(1:min(numel(sensorTimeWindow), 5)) = true;
end

accelBias = mean(sensorAccelWindow(biasMask));
sensorAccelInput = sensorAccelWindow - accelBias;

[accelFiltered, solverTime, solverState, sampleRateHz, cutoffHzUsed] = ...
    run_filter_ode45(sensorTimeWindow, sensorAccelInput, fc, filterOrder, x0);

imuDisplacementM = solverState(:, 1) - solverState(1, 1);

%% Convert GPS To Projected Distance
[gpsEastM, gpsNorthM] = latlon_to_local(gpsLatitudeWindow, gpsLongitudeWindow);
gpsDirection = [gpsEastM(end) - gpsEastM(1), gpsNorthM(end) - gpsNorthM(1)];
directionNorm = norm(gpsDirection);

if directionNorm <= eps
    error('GPS window does not contain enough motion to define a projection axis.');
end

gpsUnitDirection = gpsDirection / directionNorm;
gpsProjectedM = gpsEastM * gpsUnitDirection(1) + gpsNorthM * gpsUnitDirection(2);
gpsProjectedM = gpsProjectedM - gpsProjectedM(1);

fprintf('\n--- IMU vs GPS Comparison ---\n');
fprintf('Sensor File: %s\n', sensorFile);
fprintf('GPS File: %s\n', gpsFile);
fprintf('Axis: %s\n', accelAxis);
fprintf('Overlap Duration: %.2f s\n', overlapDurationSec);
fprintf('Window: %.2f s -> %.2f s\n', windowSec(1), windowSec(2));
fprintf('Bias Removed: %.6f m/s^2\n', accelBias);
fprintf('Sensor Sample Rate: %.3f Hz\n', sampleRateHz);
fprintf('Cutoff Used: %.3f Hz\n', cutoffHzUsed);
fprintf('------------------------------\n');

%% Plot Comparison
figure('Name', 'Selected Acceleration Axis');
plot(sensorTimeWindow, sensorAccelWindow, 'Color', [0.7 0.7 0.7], 'DisplayName', 'Raw');
hold on;
plot(sensorTimeWindow, accelFiltered, 'r', 'LineWidth', 1.5, 'DisplayName', 'Filtered');
legend('Location', 'best');
xlabel('Time From Window Start (s)');
ylabel('Acceleration (m/s^2)');
title(sprintf('%s Raw vs Filtered', accelAxis), 'Interpreter', 'none');
grid on;

figure('Name', 'IMU vs GPS Distance Comparison');
subplot(2, 1, 1);
plot(solverTime, imuDisplacementM, 'b', 'LineWidth', 1.5, 'DisplayName', 'IMU displacement');
hold on;
plot(gpsTimeWindow, gpsProjectedM, 'g', 'LineWidth', 1.5, 'DisplayName', 'GPS projected distance');
legend('Location', 'best');
xlabel('Time From Window Start (s)');
ylabel('Distance (m)');
title('IMU Displacement vs GPS Projected Distance');
grid on;

subplot(2, 1, 2);
plot(gpsEastM, gpsNorthM, 'k.-', 'LineWidth', 1.0, 'DisplayName', 'GPS path');
hold on;
plot(gpsEastM(1), gpsNorthM(1), 'go', 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
plot(gpsEastM(end), gpsNorthM(end), 'ro', 'MarkerFaceColor', 'r', 'DisplayName', 'End');
legend('Location', 'best');
xlabel('East (m)');
ylabel('North (m)');
title('Selected GPS Window');
axis equal;
grid on;

function tableData = read_clean_table(filename)
fid = fopen(filename, 'r');
if fid < 0
    error('Could not open file: %s', filename);
end

rawBytes = fread(fid, '*uint8')';
fclose(fid);

rawBytes = rawBytes(rawBytes ~= 0);
if isempty(rawBytes)
    error('File is empty or only contains null bytes: %s', filename);
end

tempFile = [tempname, '.csv'];

tempFid = fopen(tempFile, 'w');
if tempFid < 0
    error('Could not create temporary file for: %s', filename);
end

fwrite(tempFid, rawBytes, 'uint8');
fclose(tempFid);

cleanupTempFile = onCleanup(@() delete_if_exists(tempFile));
tableData = readtable(tempFile, 'TextType', 'string');
end

function values = numeric_column(tableData, columnName)
columnName = char(columnName);

if ~ismember(columnName, tableData.Properties.VariableNames)
    availableColumns = strjoin(tableData.Properties.VariableNames, ', ');
    error('Missing required column "%s". Available columns: %s', columnName, availableColumns);
end

rawValues = tableData.(columnName);
if isnumeric(rawValues)
    values = double(rawValues);
else
    values = str2double(string(rawValues));
end

values = values(:);
end

function [eastM, northM] = latlon_to_local(latitude, longitude)
lat0 = latitude(1);
lon0 = longitude(1);
cosLat0 = cosd(lat0);

eastM = (longitude - lon0) * 111320 * cosLat0;
northM = (latitude - lat0) * 111320;
end

function delete_if_exists(filename)
if exist(filename, 'file')
    delete(filename);
end
end

function sensorFile = paired_sensor_file(gpsFile)
gpsFileChar = char(gpsFile);
[folderName, baseName, extension] = fileparts(gpsFileChar);
gpsSuffix = '_gps';

if ~endsWith(baseName, gpsSuffix)
    error('Could not infer sensorFile because gpsFile does not end with "_gps.csv": %s', gpsFileChar);
end

runName = baseName(1:end - numel(gpsSuffix));
sensorFile = string(fullfile(folderName, [runName, '_sensor', extension]));
end

function gpsFile = paired_gps_file(sensorFile)
sensorFileChar = char(sensorFile);
[folderName, baseName, extension] = fileparts(sensorFileChar);
sensorSuffix = '_sensor';

if ~endsWith(baseName, sensorSuffix)
    error('Could not infer gpsFile because sensorFile does not end with "_sensor.csv": %s', sensorFileChar);
end

runName = baseName(1:end - numel(sensorSuffix));
gpsFile = string(fullfile(folderName, [runName, '_gps', extension]));
end

function isSameRun = same_file_run(fileA, fileB)
[~, baseA, extA] = fileparts(char(fileA));
[~, baseB, extB] = fileparts(char(fileB));
isSameRun = strcmp(baseA, baseB) && strcmp(extA, extB);
end
