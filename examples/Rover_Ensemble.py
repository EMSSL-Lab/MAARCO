import socket
import os
import numpy as np
import joblib
import pandas as pd
from scipy.signal import savgol_filter, periodogram
from scipy.stats import skew
from collections import deque

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
model_files = {
    "DT":  "terrain_dt_model_80_20_All.pkl",
    # "GB":  "terrain_gb_model_80_20_All.pkl",
    "KNN": "terrain_knn_model_80_20_All.pkl",
    "RF":  "terrain_rf_model_80_20_All.pkl",
    "SVM": "terrain_svm_model_80_20_All.pkl"
}

models = {}
for name, filename in model_files.items():
    path = os.path.join(MODEL_DIR, filename)
    models[name] = joblib.load(path)

# Use the RF model specifically for feature ordering (assuming they all use the same)
expected_features = models["RF"].feature_names_in_

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

# Port 5006 for Python -> Rust communication
PYTHON_SENDER_PORT = 5006
rust_addr = ("127.0.0.1", PYTHON_SENDER_PORT)

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
buffer = deque(maxlen=S)
new_data_count = 0

print("System Online. Awaiting data from Rust...")

while True:
    data, addr = sock.recvfrom(1024)
    try:
        # 1. Clean the string (remove quotes and split)
        raw_str = data.decode().replace('"', '')
        parts = [float(x) for x in raw_str.split(',')]
        # print(f"Received {len(parts)} ")

        if len(parts) >= 19:
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

            # C. Collect predictions from all 5 models
            all_preds = []
            for name, mdl in models.items():
                pred = mdl.predict(df_final)[0]
                all_preds.append(pred)
            
            # D. Calculate Majority (Mode)
            # Using pandas to easily find the most frequent string/label
            final_prediction = pd.Series(all_preds).mode()[0]
            
            # E. Calculate "Agreement" (How many models agreed with the winner)
            agreement_count = all_preds.count(final_prediction)
            agreement_pct = (agreement_count / 5) * 100
            
            # print(f"Majority: {final_prediction:<12} | Agreement: {agreement_pct:.0f}% | Votes: {all_preds}")
            
            # Format: "Terrain,Agreement%" e.g., "Gravel,80"
            message = f"{final_prediction},{agreement_pct:.0f}"
            sock.sendto(message.encode(), rust_addr)

            print(f"Sent to Rust: {message}")
            new_data_count = 0

    except Exception as e:
        # import traceback
        # print(f"CRITICAL ERROR: {e}")
        # traceback.print_exc() # This will show exactly which line in the ML logic failed
        continue
        