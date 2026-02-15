% Define the filename
close all
% filename =['C:\Users\jcohe\OneDrive\Documents\Python_Projects\PhD_Research\Data_Review\Classifier\Hard Ice Useful Data\HardIce_01_26_60RPM_test1_u2.txt'];% Replace with your actual filename

% filename =['C:\Users\jcohe\OneDrive\Documents\Python_Projects\PhD_Research\Data_Review\Classifier\Wet Sand Useful Data\WetSand2_Feb5_26_u4.txt'];

filename =['C:\Users\jcohe\OneDrive\Documents\Python_Projects\PhD_Research\Data_Review\Classifier\Wet Sand Useful Data\WetSand1_Feb5_26_u1.txt'];

% filename =['Data_Review\Classifier\Soft Snow Useful Data\SoftSnow_02_02_60RPM_test4_u1.txt'];

file2 = 'Mean_RPM_iR.txt';

% Extract just the file name without extension
[filepath, name, ~] = fileparts(filename);
% Define your output directory
outputFolder = 'C:\Users\jcohe\OneDrive\Documents\Python_Projects\PhD_Research\Data_Review\Classifier\Normalized_Data';

P = 70;
% Load the data from the text file
data = load(filename);
data_Beq = load(file2);

% Separate the data into individual vectors
timeArd = data(:,1)/1000;
voltageB = data(:,2); % Right Motor
currentB = data(:,3); % Right Motor
voltageA = data(:,4); % Left Motor
currentA = data(:,5); % Left Motor
IavLSave = (data(:,6)./51.2)- 10.0;
IavRSave = (data(:,7)./51.2)- 10.0;
eulX = data(:,8);
eulY = data(:,9);
eulZ = data(:,10);
lin_accel_x = data(:,11);
lin_accel_y = data(:,12);
lin_accel_z = data(:,13);
sonar_distance_mm = data(:,14);
ToF = data(:,15);
RPM_L = data(:,16);
RPM_R = data(:,17);

%% Torque Calculations

RPM_mean = data_Beq(1:18,1);
RPM_mean = [0;RPM_mean];
iR_mean = data_Beq(19:36,1);
iR_mean = [0;iR_mean];
p = polyfit(RPM_mean,iR_mean,5);

iL_poly = polyval(p,RPM_L);
iR_poly = polyval(p,RPM_R);

% RPM vs. t
% Torque vs. t
%T = kt * I - Beq * rpm
% Beq = kt * (i/rpm)
kt = 0.3366;

% Step 1: Calculate Beq
Beq_L = kt*(iL_poly/RPM_L);
Beq_R = kt*(iR_poly/RPM_R);

% Step 2: Calculate Torque
T_L = kt * IavLSave - Beq_L * RPM_L;
T_R = kt * IavRSave - Beq_R * RPM_R;

%% Continued Plots
L_Rotations = data(:,18);
R_Rotations = data(:,19);

% Roll, Pitch, Yaw values
pitchDeg = eulZ;
rollDeg = -eulY;
yawDeg = -eulX;

% Calculate the time differences between consecutive signals
timeDifferences = diff(timeArd);
% 1. Calculate the average time difference (in seconds)
averageTimeDifference = mean(timeDifferences);
% 2. Calculate the average logging frequency (in Hz)
averageFrequency = 1 / averageTimeDifference;
% 3. Display the results
fprintf('\n--- Logging Frequency Analysis ---\n');
fprintf('Average Time Difference (Delta t): %.4f seconds\n', averageTimeDifference);
fprintf('Average Logging Frequency (f): %.2f Hz\n', averageFrequency);
fprintf('----------------------------------\n');

timetof = diff(ToF);
%%
figure('Name', 'Group 1: Distance Sensors');
subplot(2,1,1);
plot(timeArd, ToF, 'b.', 'DisplayName', 'ToF');
title('ToF over time'); ylabel('ToF mm'); grid on;

subplot(2,1,2);
plot(timeArd, sonar_distance_mm, 'r.', 'DisplayName', 'Sonar');
title('Sonar over time'); xlabel('Time (s)'); ylabel('Sonar mm'); grid on;
ylim([-inf, 600])
%%

figure('Name', 'Group 2: Motor Data');
subplot(2,2,1);
plot(timeArd, RPM_L, 'b', timeArd, RPM_R, 'r');
title('Motor RPM'); ylabel('RPM'); legend('Left', 'Right'); grid on;

subplot(2,2,2);
plot(timeArd, L_Rotations, 'b--', timeArd, R_Rotations, 'r--');
title('Motor Rotations'); xlabel('Time (s)'); ylabel('Rotations'); legend('Left', 'Right'); grid on;

subplot(2,2,3);
plot(timeArd, T_L, 'b--', timeArd, T_R, 'r--');
title('Motor Torque'); xlabel('Time (s)'); ylabel('Torque Nm'); legend('Left', 'Right'); grid on;

%%
figure('Name', 'Group 3: Electrical Data');
subplot(3,1,1);
plot(timeArd, voltageA, 'b', timeArd, voltageB, 'r');
title('Battery Voltage'); ylabel('Volts (V)'); legend('Battery Left', 'Battery Right'); grid on;

subplot(3,1,2);
plot(timeArd, currentA, 'b', timeArd, currentB, 'r');
title('Battery Current'); ylabel('milli amps (mA)'); legend('Battery Left', 'Battery Right'); grid on;

subplot(3,1,3);
plot(timeArd, IavLSave, 'g',timeArd, IavRSave, 'p');
title('Motor Current'); xlabel('Time (s)'); ylabel('Amps (A)'); legend('Motor Left ', 'Motor Right'); grid on;

%%
figure('Name', 'Group 4: Attitude');
plot(timeArd, rollDeg, 'r', 'DisplayName', 'Roll');
hold on;
plot(timeArd, pitchDeg, 'g', 'DisplayName', 'Pitch');
plot(timeArd, yawDeg, 'b', 'DisplayName', 'Yaw');
title('Euler Angles');
xlabel('Time (s)'); ylabel('Degrees');
legend; grid on;
%%
figure('Name', 'Group 5: IMU Acceleration');
plot(timeArd, lin_accel_x, 'r', timeArd, lin_accel_y, 'g', timeArd, lin_accel_z, 'b');
title('Linear Acceleration');
xlabel('Time (s)'); ylabel('m/s^2');
legend('Acc X', 'Acc Y', 'Acc Z'); grid on;
%%
figure('Name', 'Group 6: Timing');
plot(timeArd(2:end), timeDifferences, 'ko');
title('Time Delta between Samples');
xlabel('Time (s)'); ylabel('Delta T (s)');
grid on;

%%
%Link the zoom of all figures
allAxes = findall(0, 'type', 'axes');
linkaxes(allAxes, 'x');

% %% Sensor Suite
% Sensors = [T_L,T_R,IavLSave,IavRSave,...
%     lin_accel_x,lin_accel_y,lin_accel_z,...
%     sonar_distance_mm,ToF,RPM_L,RPM_R,];
% 
% sensorNames = {'TorqL', 'TorqR', 'IavL', 'IavR', ...
%                'AccX', 'AccY', 'AccZ', ...
%                'Sonar', 'ToF', 'RPML', 'RPMR'};
% 
% [N, numSensors] = size(Sensors);
% 
% %% Data Segmentation
% 
% % D = entire dataset
% % D = {X1,X2,XN} where X is a single datapoint at index i
% % N = total number of points in a dataset
% %N2 = length(lin_accel_x);
% % O is the overlap size (number of shared samples between consecutive segments); 
% desiredOverlapTime = 1; % seconds
% O = round(desiredOverlapTime * averageFrequency);
% % S is window size (segment length), data in a window; 
% desiredWindowTime = 2; % seconds
% S = round(desiredWindowTime * averageFrequency);
% 
% % K is the number of complete segments or windows.
% K = floor((N-O)/(S-O));
% numFeaturesPerSensor = 14; 
% 
% fprintf('\n--- Segmentation Analysis ---\n');
% fprintf('Total Samples: %d\n', N);
% fprintf('Total Segments: %d\n', K);
% 
% % Initialize storage for segments and features
% featureMatrix = zeros(K, numSensors * numFeaturesPerSensor);
% allFeatureNames = {};
% 
% %segments = zeros(K, S); %rows,columns
% 
% %% Savitzky-Golay Noise Filter
% polyOrder = 3;
% frameLen = 11; % Must be odd and > polyOrder + 1
% 
% % Define Filter Bank: [SensorIndex, polyOrder, frameLen]
% % Defaults: 3, 11. Adjust based on sensor noise characteristics.
% filterConfigs = [
%     1, 5, 5;  % TorqL (0.5s window)
%     2, 2, 5;  % TorqR
%     3, 2, 9;  % IavL (Current can handle more smoothing)
%     4, 2, 9;  % IavR
%     5, 3, 5;  % AccX (Keep Order high/Frame small for vibration)
%     6, 3, 5;  % AccY
%     7, 3, 5;  % AccZ
%     8, 2, 11; % Sonar (Needs the most smoothing)
%     9, 2, 7;  % ToF
%     10, 2, 5; % RPML
%     11, 2, 5; % RPMR
% ];
% 
% % Create Filtered Matrix
% Sensors_Filtered = Sensors;
% 
% % Loop through the specific configuration list
% for i = 1:size(filterConfigs, 1)
%     idx   = filterConfigs(i, 1);
%     order = filterConfigs(i, 2);
%     len   = filterConfigs(i, 3);
% 
%     % Safety check: frameLen must be odd and > order + 1
%     if mod(len, 2) == 0, len = len + 1; end
%     if len <= order, len = order + 2; if mod(len,2)==0, len=len+1; end; end
% 
%     % Apply specific filter
%     Sensors_Filtered(:, idx) = sgolayfilt(Sensors(:, idx), order, len);
% end
% 
% checkIdx = [8, 10, 11];
% figure('Name', 'Custom Filtering Bank Verification');
% 
% for j = 1:length(checkIdx)
%     sIdx = checkIdx(j);
%     subplot(3, 1, j);
%     plot(timeArd, Sensors(:, sIdx), 'b', 'DisplayName', 'Raw'); 
%     hold on;
%     plot(timeArd, Sensors_Filtered(:, sIdx), 'r', 'LineWidth', 1.2, 'DisplayName', 'Filtered');
%     title(['Sensor: ', sensorNames{sIdx}]);
%     grid on; legend('Location', 'northeast');
% end
% xlabel('Time (s)');
% 
% %% Customized Filtering Bank Verification (All 11 Sensors)
% % figure('Name', 'Full Sensor Filter Verification', 'Units', 'normalized', 'Position', [0.1, 0.1, 0.8, 0.8]);
% % figure('Name', 'Full Sensor Filter Verification');
% % 
% % for sIdx = 1:numSensors
% %     subplot(4, 3, sIdx); % 4 rows, 3 columns layout
% % 
% %     % Plot Raw data in grey
% %     plot(timeArd, Sensors(:, sIdx), 'Color', [0.7 0.7 0.7], 'DisplayName', 'Raw'); 
% %     hold on;
% % 
% %     % Plot Filtered data in red
% %     plot(timeArd, Sensors_Filtered(:, sIdx), 'r', 'LineWidth', 1.1, 'DisplayName', 'Filtered');
% % 
% %     title(['Sensor: ', sensorNames{sIdx}]);
% %     grid on;
% % 
% %     % Only show legend on the first plot to save space
% %     if sIdx == 1
% %         legend('Location', 'northeast');
% %     end
% % end
% % 
% % % Label the bottom-most plots
% % subplot(4,3,10); xlabel('Time (s)');
% % subplot(4,3,11); xlabel('Time (s)');
% 
% %% Feature Loop
% 
% for i = 1:K
%     start_idx = (i-1) * (S - O) + 1;
%     end_idx = start_idx + S - 1;
% 
%     segmentFeatures = []; % Temporary row for this segment
% 
%     for s = 1:numSensors
%         % Extract the segment for the current sensor
%         sig = Sensors_Filtered(start_idx:end_idx, s);
% 
%         % --- Time Domain ---
%         f_var  = var(sig);
%         f_rms  = rms(sig);
%         f_skew = skewness(sig, 1);
%         f_p2p  = peak2peak(sig);
%         f_ener = sum(sig.^2);
% 
%         % --- FFT Domain ---
%         sig_fft = abs(fft(sig));
%         mag = sig_fft(1:floor(S/2)+1);
%         f_fft_m = mean(mag);
%         [f_fft_max, max_idx] = max(mag);
%         freq_axis = (0:floor(S/2)) * (averageFrequency / S); 
%         fft_freq_max = freq_axis(max_idx); 
%         fft_power = sum(mag.^2) / S;    
%         fft_bw = obw(sig, averageFrequency); % Bandwidth
% 
%         % --- PSD Domain ---
%         [pxx, f_axis] = periodogram(sig, rectwin(S), S, averageFrequency);
%         f_psd_m = mean(pxx);
%         f_psd_s = std(pxx);
%         [~, p_idx] = max(pxx);
%         f_psd_f = f_axis(p_idx);
%         f_psd_p = sum(pxx);       
% 
%         % Combine all features for this specific sensor
%         current_sensor_feats = [f_var, f_rms, f_skew, f_p2p, f_ener, ...
%                                 f_fft_m, f_fft_max,fft_freq_max,fft_power,fft_bw,...
%                                 f_psd_m, f_psd_s,f_psd_p, f_psd_f];
% 
%         segmentFeatures = [segmentFeatures, current_sensor_feats];
% 
%         % Build Name list only on the first iteration
%         if i == 1
%             subNames = {'Var','RMS','Skew','P2P','Energy','FFTMean','FFTMax','FFT_FreqMax','FFT_Power','FFT_BW','PSDMean','PSDStd','PSDPower','PSDPeakF'};
%             for n = 1:length(subNames)
%                 allFeatureNames{end+1} = [sensorNames{s}, '_', subNames{n}];
%             end
%         end
%     end
% 
%     featureMatrix(i, :) = segmentFeatures;
% end
% 
% %% NaN Replacement
% 
% % 1. Identify "Bad Columns" (Constant values where Std Dev is 0)
% % These usually come from sensors that didn't move or change during the test
% badCols = std(featureMatrix) == 0;
% 
% if any(badCols)
%     fprintf('Removing %d constant features to prevent NaN during normalization.\n', sum(badCols));
% 
%     % Remove columns from the matrix
%     featureMatrix(:, badCols) = [];
% 
%     % CRITICAL: Remove the names from the cell array too
%     allFeatureNames(badCols) = [];
% end
% 
% %% Normalization and Model Labeling
% % Vector Normalization (Z-score)
% trainMean = mean(featureMatrix);
% trainStd = std(featureMatrix);
% featureMatrix_Norm = (featureMatrix - trainMean) ./ trainStd;
% % Convert to Table
% finalFeatureTable = array2table(featureMatrix_Norm, 'VariableNames', allFeatureNames);
% % Display a snippet
% head(finalFeatureTable(:, 1:14))
% %Open the table in the Variable Viewer window
% openvar('finalFeatureTable')
% 
% %% Data Save
% 
% % Save the RAW features instead of normalized
% rawFeatureTable = array2table(featureMatrix, 'VariableNames', allFeatureNames);
% % Create full output path with .csv extension
% outputFile = fullfile(outputFolder, [name '.csv']);
% % Write table
% writetable(rawFeatureTable, outputFile);
% 
% fprintf('\n--- Save Complete ---\n');
% fprintf('Saved to:\n%s\n', outputFile);
% 
% %% Plotting the Features
% % figure('Name', 'Segmented Features: Accel X, Accel Y');
% % 
% % subplot(3,2,1);
% % plot(featureMatrix(:,1), '-r', 'LineWidth', 1.5); 
% % hold on;
% % plot(featureMatrix(:,11), '-b', 'LineWidth', 1.5);
% % title('Variance per Segment'); xlabel('Segment Index'); ylabel('Variance'); grid on;
% % legend('Accel X','Accel Y');
% % 
% % subplot(3,2,2)
% % plot(featureMatrix(:,2), '-r', 'LineWidth', 1.5);
% % hold on;
% % plot(featureMatrix(:,12), '-b', 'LineWidth', 1.5);
% % title('RMS per Segment'); ylabel('RMS (m/s^2)'); grid on;
% % legend('Accel X','Accel Y');
% % 
% % subplot(3,2,3)
% % plot(featureMatrix(:,3), '-r', 'LineWidth', 1.5);
% % hold on;
% % plot(featureMatrix(:,13), '-b', 'LineWidth', 1.5);
% % title('Skewness per Segment'); ylabel(''); grid on;
% % legend('Accel X','Accel Y');
% % 
% % subplot(3,2,4)
% % plot(featureMatrix(:,4), '-r', 'LineWidth', 1.5);
% % hold on;
% % plot(featureMatrix(:,14), '-b', 'LineWidth', 1.5);
% % title('Peak 2 Peak per Segment'); ylabel('P2P (m/s^2)'); grid on;
% % legend('Accel X','Accel Y');
% % 
% % subplot(3,2,5)
% % plot(featureMatrix(:,5), '-r', 'LineWidth', 1.5);
% % hold on;
% % plot(featureMatrix(:,15), '-b', 'LineWidth', 1.5);
% % title('Signal Energy per Segment'); ylabel('Energy (m^2/s^4)'); grid on;
% % legend('Accel X','Accel Y');
% % 
% % subplot(3,2,6)
% % plot(featureMatrix(:,6), '-r', 'LineWidth', 1.5);
% % hold on;
% % plot(featureMatrix(:,16), '-b', 'LineWidth', 1.5);
% % title('FFT Mean per Segment'); ylabel('FFT (m/s^2)'); grid on;
% % legend('Accel X','Accel Y');
% % 
% % %% Feature-level fusion
% % 
% % %% Vector normalization
% % 
% % 
