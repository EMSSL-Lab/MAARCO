import socket
import os
import numpy as np
import joblib
import pandas as pd
from scipy.signal import savgol_filter, periodogram
from scipy.stats import skew
from collections import deque
from datetime import datetime

import csv
import time

np.seterr(all='ignore') # Mutes all math warnings globally

try:
    home = os.path.expanduser("~")
    # 2. Update the Text File Path
    BEQ_PATH = os.path.join(home, "MAARCO/examples/Mean_RPM_iR.txt")
    data_Beq = np.loadtxt(BEQ_PATH)
    # data_Beq = np.loadtxt('~/MAARCO/examples/Mean_RPM_iR.txt')
    
    # MATLAB: RPM_mean = data_Beq(1:18,1); RPM_mean = [0;RPM_mean];
    rpm_mean = np.concatenate(([0], data_Beq[0:18]))
    
    # MATLAB: iR_mean = data_Beq(19:36,1); iR_mean = [0;iR_mean];
    ir_mean = np.concatenate(([0], data_Beq[18:36]))
    
    # MATLAB: p = polyfit(RPM_mean,iR_mean,5);
    p_coeffs = np.polyfit(rpm_mean, ir_mean, 5)
    print("Polynomial coefficients calculated successfully.")
except Exception as e:
    print(f"Error loading Mean_RPM_iR.txt: {e}")
    # Fallback/Default coefficients if file is missing
    p_coeffs = None

# --- 1. Model Loading ---
MODEL_DIR = os.path.join(home, "MAARCO/examples/Model_py")

# MODEL_PATH_1 = os.path.join(MODEL_DIR, "terrain_dt_model_80_20_All.pkl")
# MODEL_PATH_2 = os.path.join(MODEL_DIR, "terrain_gb_model_80_20_All.pkl")
# MODEL_PATH_3 = os.path.join(MODEL_DIR, "terrain_knn_model_80_20_All.pkl")
# MODEL_PATH_4 = os.path.join(MODEL_DIR, "terrain_rf_model_80_20_All.pkl")
# MODEL_PATH_5 = os.path.join(MODEL_DIR, "terrain_svm_model_80_20_All.pkl")

# Map user input to file paths
MODELS = {
    "1": ("Decision Tree All", "terrain_dt_model_80_20_All.pkl"),
    "2": ("Gradient Boosting All", "terrain_gb_model_80_20_All.pkl"),
    "3": ("K-Nearest Neighbors All", "terrain_knn_model_80_20_All.pkl"),
    "4": ("Random Forest All", "terrain_rf_model_80_20_All.pkl"),
    "5": ("SVM All", "terrain_svm_model_80_20_All.pkl"),

    "6": ("Decision Tree TOF", "terrain_dt_model_80_20_TOF.pkl"),
    "7": ("Gradient Boosting TOF", "terrain_gb_model_80_20_TOF.pkl"),
    "8": ("K-Nearest Neighbors TOF", "terrain_knn_model_80_20_TOF.pkl"),
    "9": ("Random Forest TOF", "terrain_rf_model_80_20_TOF.pkl"),
    "10": ("SVM TOF", "terrain_svm_model_80_20_TOF.pkl"),
    
    "11": ("Decision Tree SON", "terrain_dt_model_80_20_SON.pkl"),
    "12": ("Gradient Boosting SON", "terrain_gb_model_80_20_SON.pkl"),
    "13": ("K-Nearest Neighbors SON", "terrain_knn_model_80_20_SON.pkl"),
    "14": ("Random Forest SON", "terrain_rf_model_80_20_SON.pkl"),
    "15": ("SVM SON", "terrain_svm_model_80_20_SON.pkl"),
    
    "16": ("Decision Tree TO", "terrain_dt_model_80_20_TO.pkl"),
    "17": ("Gradient Boosting TO", "terrain_gb_model_80_20_TO.pkl"),
    "18": ("K-Nearest Neighbors TO", "terrain_knn_model_80_20_TO.pkl"),
    "19": ("Random Forest TO", "terrain_rf_model_80_20_TO.pkl"),
    "20": ("SVM TO", "terrain_svm_model_80_20_TO.pkl"),
    
    "21": ("Decision Tree ST", "terrain_dt_model_80_20_ST.pkl"),
    "22": ("Gradient Boosting ST", "terrain_gb_model_80_20_ST.pkl"),
    "23": ("K-Nearest Neighbors ST", "terrain_knn_model_80_20_ST.pkl"),
    "24": ("Random Forest ST", "terrain_rf_model_80_20_ST.pkl"),
    "25": ("SVM ST", "terrain_svm_model_80_20_ST.pkl"),
    
    "26": ("Decision Tree RT", "terrain_dt_model_80_20_RT.pkl"),
    "27": ("Gradient Boosting RT", "terrain_gb_model_80_20_RT.pkl"),
    "28": ("K-Nearest Neighbors RT", "terrain_knn_model_80_20_RT.pkl"),
    "29": ("Random Forest RT", "terrain_rf_model_80_20_RT.pkl"),
    "30": ("SVM RT", "terrain_svm_model_80_20_RT.pkl"),
    
    "31": ("Decision Tree Acc", "terrain_dt_model_80_20_Acc.pkl"),
    "32": ("Gradient Boosting Acc", "terrain_gb_model_80_20_Acc.pkl"),
    "33": ("K-Nearest Neighbors Acc", "terrain_knn_model_80_20_Acc.pkl"),
    "34": ("Random Forest Acc", "terrain_rf_model_80_20_Acc.pkl"),
    "35": ("SVM Acc", "terrain_svm_model_80_20_Acc.pkl"),

    "36": ("Decision Tree AR", "terrain_dt_model_80_20_AR.pkl"),
    "37": ("Gradient Boosting AR", "terrain_gb_model_80_20_AR.pkl"),
    "38": ("K-Nearest Neighbors AR", "terrain_knn_model_80_20_AR.pkl"),
    "39": ("Random Forest AR", "terrain_rf_model_80_20_AR.pkl"),
    "40": ("SVM AR", "terrain_svm_model_80_20_AR.pkl"),

    "41": ("Decision Tree CR", "terrain_dt_model_80_20_CR.pkl"),
    "42": ("Gradient Boosting CR", "terrain_gb_model_80_20_CR.pkl"),
    "43": ("K-Nearest Neighbors CR", "terrain_knn_model_80_20_CR.pkl"),
    "44": ("Random Forest CR", "terrain_rf_model_80_20_CR.pkl"),
    "45": ("SVM CR", "terrain_svm_model_80_20_CR.pkl"),



}

print("\n--- Available Terrain Models ---")
for key, (name, _) in MODELS.items():
    print(f"[{key}] {name}")

choice = input("Select a model number (default is 4): ") or "4"

if choice in MODELS:
    selected_name, selected_file = MODELS[choice]
    MODEL_PATH = os.path.join(MODEL_DIR, selected_file)
    print(f"Loading {selected_name}...")

    # Load the selected model
    rf_model = joblib.load(MODEL_PATH)
    
    # Safety check for feature names (most scikit-learn models use this)
    if hasattr(rf_model, "feature_names_in_"):
        expected_features = rf_model.feature_names_in_
    else:
        # Fallback if the model doesn't have names stored (e.g., some older versions)
        print("Warning: Model does not contain feature names. Ensure feature order is correct.")
        # You might want to define a hardcoded list here if the model fails
else:
    print("Invalid selection. Exiting.")
    exit()

# rf_model = joblib.load(MODEL_PATH)
# Get the exact feature order the model was trained on
# expected_features = rf_model.feature_names_in_

# --- 2. Configuration ---
FS = 10  
WINDOW_SEC = 2
OVERLAP_SEC = 1
S = int(WINDOW_SEC * FS)
STEP = S - int(OVERLAP_SEC * FS)

FILTER_CONFIGS = {
    0: (5, 5), 1: (2, 5), 2: (2, 9), 3: (2, 9),
    4: (3, 5), 5: (3, 5), 6: (3, 5), 7: (2, 11),
    8: (2, 7), 9: (2, 5), 10: (2, 5), 11: (2, 5), 12: (2, 5)
}

# Feature names mapping to match your MATLAB output names
SENSOR_NAMES = ['TorqL', 'TorqR', 'IavL', 'IavR', 'AccX', 'AccY', 'AccZ', 'Sonar', 'ToF', 'RPML', 'RPMR', 'rollDeg', 'pitchDeg']
SUB_NAMES = ['Var','RMS','Skew','P2P','Energy','FFTMean','FFTMax','FFT_FreqMax','FFT_Power','FFT_BW','PSDMean','PSDStd','PSDPower','PSDPeakF']

def extract_features_to_df(segment):
    """Processes segment and returns a DataFrame with named features for the model."""
    feats_dict = {}
    for s_idx in range(segment.shape[1]):
        sig = segment[:, s_idx]
        order, frame = FILTER_CONFIGS.get(s_idx, (3, 11))
        
        # 1. Start with the desired frame size or the actual data length
        window_size = min(frame, len(sig))
        
        # 2. Savgol requirement: window_size must be > order
        if window_size <= order:
            window_size = order + 1
            
        # 3. Savgol requirement: window_size must be ODD
        if window_size % 2 == 0:
            # If we are at the end of the data, we must go down; 
            # otherwise, going up is usually safer for smoothing.
            if window_size + 1 <= len(sig):
                window_size += 1
            else:
                window_size -= 1

        # Final safety: If window_size is now <= order because we subtracted, 
        # it means the buffer is simply too small for this sensor's config.
        if window_size <= order:
            sig_f = sig # Skip filtering and use raw signal for this window
        else:
            sig_f = savgol_filter(sig, window_size, order)

        # Stats
        mag = np.abs(np.fft.fft(sig_f))[:S//2 + 1]
        freqs, psd = periodogram(sig_f, fs=FS, window='boxcar')
        freq_axis = np.linspace(0, FS/2, len(mag))
        
        # Calculate the 14 features
        vals = [
            #Time Domain
            np.var(sig_f), #Variance
            np.sqrt(np.mean(sig_f**2)), #Root Mean Square 
            skew(sig_f), #Skewness
            np.ptp(sig_f), #Peak 2 Peak
            np.sum(sig_f**2), #Energy
            
            #Frequency Domain
            np.mean(mag), # FFT Mean
            np.max(mag),  # FFT Max
            np.argmax(mag)*(FS/S), # FFT Freq at Max
            np.sum(mag**2)/S, # FFT Power
            np.sqrt(np.sum((freq_axis - (np.argmax(mag)*(FS/S)))**2 * mag)/np.sum(mag)) if np.sum(mag)!=0 else 0, # FFT Bandwidth
            
            # Ppower Spectral Density (PSD)
            np.mean(psd), # PSD Mean
            np.std(psd), # PSD Std
            np.sum(psd),# PSD Power
            freqs[np.argmax(psd)] # PSD Peak Frequency
        ]
        
        # Map to names
        for i, val in enumerate(vals):
            feat_name = f"{SENSOR_NAMES[s_idx]}_{SUB_NAMES[i]}"
            feats_dict[feat_name] = [val]
            
    return pd.DataFrame(feats_dict)

# --- 3. Live Loop ---
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 5005))
rust_addr = ("127.0.0.1", 5006)  # <--- ADD THIS LINE
buffer = deque(maxlen=S)
new_data_count = 0


print("System Online. Awaiting data from Rust...")

# --- 4. CSV Logging Configuration ---
# Creates a filename like: live_terrain_20260224_1722.csv
# Create the directory if it doesn't exist
LOG_DIR = "ML_data"
if not os.path.exists(LOG_DIR):
    os.makedirs(LOG_DIR)

timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")

# 2. Clean the model name for the filename (remove spaces)
clean_model_name = selected_name.replace(" ", "_")

# 3. Combine them into the filename
# Example result: live_terrain_Random_Forest_All_20260226_1705.csv
LOG_FILE = os.path.join(LOG_DIR, f"live_terrain_{clean_model_name}_{timestamp_str}.csv")

# LOG_FILE = os.path.join(LOG_DIR, f"live_terrain_{timestamp_str}.csv")
fieldnames = ['timestamp', 'prediction', 'confidence', 'yaw', 'kp', 'kd']

print(f"Logging predictions to: {LOG_FILE}")

# Create the file and write the header
with open(LOG_FILE, mode='w', newline='') as f:
    writer = csv.DictWriter(f, fieldnames=fieldnames)
    writer.writeheader()

start_time = time.time() # To track relative time

while True:
    data, addr = sock.recvfrom(1024)
    try:
        # 1. Clean the string (remove quotes and split)
        raw_str = data.decode().replace('"', '')
        parts = [float(x) for x in raw_str.split(',')]
        # print(f"Received {len(parts)} ")

        if len(parts) >= 21:
            # --- Replicating your MATLAB manual math ---
            # Indices match your list: 0=Time, 5=IavL, 6=IavR, 8=EulY, 9=EulZ, etc.
            
            # Current conversion (MATLAB: (data(:,6)./51.2)- 10.0)
            iav_l = (parts[5] / 51.2) - 10.0
            iav_r = (parts[6] / 51.2) - 10.0
            
            # RPMs
            rpm_l = parts[15]
            rpm_r = parts[16]
            
            # --- Torque Calculations ---
            kt = 0.3366
            # iL_poly = polyval(p, RPM_L)
            il_poly = np.polyval(p_coeffs, rpm_l)
            ir_poly = np.polyval(p_coeffs, rpm_r)
            
            # Step 1: Calculate Beq (with protection against div by zero)
            beq_l = kt * (il_poly / rpm_l) if rpm_l != 0 else 0
            beq_r = kt * (ir_poly / rpm_r) if rpm_r != 0 else 0
            
            # Step 2: Calculate Torque
            # T_L = kt * IavLSave - Beq_L * RPM_L
            t_l = (kt * iav_l) - (beq_l * rpm_l)
            t_r = (kt * iav_r) - (beq_r * rpm_r)

            # IMU Inversions (MATLAB: rollDeg = -eulY; pitchDeg = eulZ - pitch_correct)
            # pitch_correct= - 4; %old
            # roll_correct = 1; %old
            
            pitch_correct=  8.5; #new
            roll_correct = -9.4; #new
            
            # pitch_correct=  9.56; %Dec4
            # roll_correct = 3.46; %Dec4

            roll_deg = -parts[8]  - roll_correct 
            pitch_deg = parts[9] - pitch_correct 
            
            yaw = parts[19]
            kp = parts[20]
            kd = parts[21]
            # 2. Construct the 13-sensor vector for the ML model
            # Order: [TorqL, TorqR, IavL, IavR, AccX, AccY, AccZ, Sonar, ToF, RPML, RPMR, roll, pitch]
            ml_vector = [
                t_l, t_r, iav_l, iav_r,
                parts[10], parts[11], parts[12], # Acc X, Y, Z
                parts[13], parts[14],            # Sonar, ToF
                rpm_l, rpm_r,
                roll_deg, pitch_deg
            ]
            
            buffer.append(ml_vector)
            new_data_count += 1
        if len(buffer) == S and new_data_count >= STEP:
            # A. Feature Extraction
            df_features = extract_features_to_df(np.array(buffer))
            
            # B. Align and Predict
            df_final = df_features[expected_features].fillna(0).replace([np.inf, -np.inf], 0)
            prediction = rf_model.predict(df_final)[0]
            confidence = np.max(rf_model.predict_proba(df_final)) * 100
            
            # print(f"Terrain: {prediction:<12} | Conf: {confidence:.2f}%")
            # new_data_count = 0

            # --- C. Log to CSV ---
            current_elapsed = time.time() - start_time
            with open(LOG_FILE, mode='a', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writerow({
                    'timestamp': round(current_elapsed, 2),
                    'prediction': prediction,
                    'confidence': round(confidence, 2),
                    'yaw': yaw,
                    'kp': kp,
                    'kd': kd
                })
            print(f"Time: {current_elapsed:>6.1f}s | Terrain: {prediction:<12} | Conf: {confidence:.2f}%")
            # print(f"Time: {current_elapsed:>6.1f}s | Terrain: {prediction:<12} | Conf: {confidence:.2f}%")

            message = f"{current_elapsed},{prediction},{confidence:.0f}"
            sock.sendto(message.encode(), rust_addr)

            new_data_count = 0


    except Exception as e:
        # import traceback
        # print(f"CRITICAL ERROR: {e}")
        # traceback.print_exc() # This will show exactly which line in the ML logic failed
        continue
        