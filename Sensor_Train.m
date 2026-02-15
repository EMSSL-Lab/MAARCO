% Define the filename
close all

basePath = 'C:\Users\jcohe\OneDrive\Documents\Python_Projects\PhD_Research\Data_Review\Classifier';
folders = {'Hard Ice Useful Data', 'Soft Snow Useful Data', 'Wet Sand Useful Data'}; % your data folders
file2 = 'Mean_RPM_iR.txt'; % same for all

outputFolder = fullfile(basePath, 'Normalized_Data');
if ~exist(outputFolder, 'dir')
    mkdir(outputFolder);
end

%% Loop through each folder
for f = 1:length(folders)
    
    dataFolder = fullfile(basePath, folders{f});
    files = dir(fullfile(dataFolder, '*.txt')); % get all txt files in folder
    
    for k = 1:length(files)
        
        filename = fullfile(dataFolder, files(k).name);
        [~, name, ~] = fileparts(filename); % name for saving CSV
        
        %% --- Load data ---
        data = load(filename);
        data_Beq = load(file2);
        P = 70;

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

        %% Sensor Suite
        Sensors = [T_L,T_R,IavLSave,IavRSave,...
            lin_accel_x,lin_accel_y,lin_accel_z,...
            sonar_distance_mm,ToF,RPM_L,RPM_R,];
        
        sensorNames = {'TorqL', 'TorqR', 'IavL', 'IavR', ...
                       'AccX', 'AccY', 'AccZ', ...
                       'Sonar', 'ToF', 'RPML', 'RPMR'};
        
        [N, numSensors] = size(Sensors);

        %% Data Segmentation

        desiredOverlapTime = 1; % seconds
        O = round(desiredOverlapTime * averageFrequency);
        % S is window size (segment length), data in a window; 
        desiredWindowTime = 2; % seconds
        S = round(desiredWindowTime * averageFrequency);
        
        % K is the number of complete segments or windows.
        K = floor((N-O)/(S-O));
        numFeaturesPerSensor = 14; 
        
        fprintf('\n--- Segmentation Analysis ---\n');
        fprintf('Total Samples: %d\n', N);
        fprintf('Total Segments: %d\n', K);
        
        % Initialize storage for segments and features
        featureMatrix = zeros(K, numSensors * numFeaturesPerSensor);
        allFeatureNames = {};
        
        %segments = zeros(K, S); %rows,columns
        
        %% Savitzky-Golay Noise Filter
        polyOrder = 3;
        frameLen = 11; % Must be odd and > polyOrder + 1
        
        % Define Filter Bank: [SensorIndex, polyOrder, frameLen]
        % Defaults: 3, 11. Adjust based on sensor noise characteristics.
        filterConfigs = [
            1, 5, 5;  % TorqL (0.5s window)
            2, 2, 5;  % TorqR
            3, 2, 9;  % IavL (Current can handle more smoothing)
            4, 2, 9;  % IavR
            5, 3, 5;  % AccX (Keep Order high/Frame small for vibration)
            6, 3, 5;  % AccY
            7, 3, 5;  % AccZ
            8, 2, 11; % Sonar (Needs the most smoothing)
            9, 2, 7;  % ToF
            10, 2, 5; % RPML
            11, 2, 5; % RPMR
        ];
        
        % Create Filtered Matrix
        Sensors_Filtered = Sensors;
        
        % Loop through the specific configuration list
        for i = 1:size(filterConfigs, 1)
            idx   = filterConfigs(i, 1);
            order = filterConfigs(i, 2);
            len   = filterConfigs(i, 3);
            
            % Safety check: frameLen must be odd and > order + 1
            if mod(len, 2) == 0, len = len + 1; end
            if len <= order, len = order + 2; if mod(len,2)==0, len=len+1; end; end
            
            % Apply specific filter
            Sensors_Filtered(:, idx) = sgolayfilt(Sensors(:, idx), order, len);
        end
        
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
        
        %% Customized Filtering Bank Verification (All 11 Sensors)
        % figure('Name', 'Full Sensor Filter Verification', 'Units', 'normalized', 'Position', [0.1, 0.1, 0.8, 0.8]);
        % figure('Name', 'Full Sensor Filter Verification');
        % 
        % for sIdx = 1:numSensors
        %     subplot(4, 3, sIdx); % 4 rows, 3 columns layout
        % 
        %     % Plot Raw data in grey
        %     plot(timeArd, Sensors(:, sIdx), 'Color', [0.7 0.7 0.7], 'DisplayName', 'Raw'); 
        %     hold on;
        % 
        %     % Plot Filtered data in red
        %     plot(timeArd, Sensors_Filtered(:, sIdx), 'r', 'LineWidth', 1.1, 'DisplayName', 'Filtered');
        % 
        %     title(['Sensor: ', sensorNames{sIdx}]);
        %     grid on;
        % 
        %     % Only show legend on the first plot to save space
        %     if sIdx == 1
        %         legend('Location', 'northeast');
        %     end
        % end
        % 
        % % Label the bottom-most plots
        % subplot(4,3,10); xlabel('Time (s)');
        % subplot(4,3,11); xlabel('Time (s)');
        
        %% Feature Loop
        
        for i = 1:K
            start_idx = (i-1) * (S - O) + 1;
            end_idx = start_idx + S - 1;
            
            segmentFeatures = []; % Temporary row for this segment
            
            for s = 1:numSensors
                % Extract the segment for the current sensor
                sig = Sensors_Filtered(start_idx:end_idx, s);
                
                % --- Time Domain ---
                f_var  = var(sig);
                f_rms  = rms(sig);
                f_skew = skewness(sig, 1);
                f_p2p  = peak2peak(sig);
                f_ener = sum(sig.^2);
                
                % --- FFT Domain ---
                sig_fft = abs(fft(sig));
                mag = sig_fft(1:floor(S/2)+1);
                f_fft_m = mean(mag);
                [f_fft_max, max_idx] = max(mag);
                freq_axis = (0:floor(S/2)) * (averageFrequency / S); 
                fft_freq_max = freq_axis(max_idx); 
                fft_power = sum(mag.^2) / S;    
                fft_bw = obw(sig, averageFrequency); % Bandwidth
        
                % --- PSD Domain ---
                [pxx, f_axis] = periodogram(sig, rectwin(S), S, averageFrequency);
                f_psd_m = mean(pxx);
                f_psd_s = std(pxx);
                [~, p_idx] = max(pxx);
                f_psd_f = f_axis(p_idx);
                f_psd_p = sum(pxx);       
        
                % Combine all features for this specific sensor
                current_sensor_feats = [f_var, f_rms, f_skew, f_p2p, f_ener, ...
                                        f_fft_m, f_fft_max,fft_freq_max,fft_power,fft_bw,...
                                        f_psd_m, f_psd_s,f_psd_p, f_psd_f];
                
                segmentFeatures = [segmentFeatures, current_sensor_feats];
                
                % Build Name list only on the first iteration
                if i == 1
                    subNames = {'Var','RMS','Skew','P2P','Energy','FFTMean','FFTMax','FFT_FreqMax','FFT_Power','FFT_BW','PSDMean','PSDStd','PSDPower','PSDPeakF'};
                    for n = 1:length(subNames)
                        allFeatureNames{end+1} = [sensorNames{s}, '_', subNames{n}];
                    end
                end
            end
            
            featureMatrix(i, :) = segmentFeatures;
        end
        
        %% NaN Replacement
        
        % 1. Identify "Bad Columns" (Constant values where Std Dev is 0)
        % These usually come from sensors that didn't move or change during the test
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
        %% Normalization and Model Labeling
        % Vector Normalization (Z-score)
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
        %% Data Save
        
        % Save the RAW features instead of normalized
        rawFeatureTable = array2table(featureMatrix, 'VariableNames', allFeatureNames);
        % Create full output path with .csv extension
        outputFile = fullfile(outputFolder, [name '.csv']);
        % Write table
        writetable(rawFeatureTable, outputFile);
        
        fprintf('\n--- Save Complete ---\n');
        fprintf('Saved to:\n%s\n', outputFile);
    end
end