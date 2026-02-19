import socket
import numpy as np
import joblib
import pandas as pd
from scipy.signal import savgol_filter, periodogram
from scipy.stats import skew
from collections import deque

# --- 1. Model Loading ---
MODEL_PATH_1 = r"/home/jcohe/MAARCO/examples/Models/terrain_dt_model_80_20_All.pkl"
MODEL_PATH_2 = r"/home/jcohe/MAARCO/examples/Models/terrain_gb_model_80_20_All.pkl"
MODEL_PATH_3 = r"/home/jcohe/MAARCO/examples/Models/terrain_knn_model_80_20_All.pkl"
MODEL_PATH_4 = r"/home/jcohe/MAARCO/examples/Models/terrain_rf_model_80_20_All.pkl"
MODEL_PATH_5 = r"/home/jcohe/MAARCO/examples/Models/terrain_svm_model_80_20_All.pkl"
rf_model = joblib.load(MODEL_PATH_4)
# Get the exact feature order the model was trained on
expected_features = rf_model.feature_names_in_

# --- 2. Configuration ---
FS = 25  
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
        sig_f = savgol_filter(sig, min(frame, S-1 if S%2==0 else S), order)
        
        # Stats
        mag = np.abs(np.fft.fft(sig_f))[:S//2 + 1]
        freqs, psd = periodogram(sig_f, fs=FS, window='boxcar')
        freq_axis = np.linspace(0, FS/2, len(mag))
        
        # Calculate the 14 features
        vals = [
            np.var(sig_f), np.sqrt(np.mean(sig_f**2)), skew(sig_f), np.ptp(sig_f), np.sum(sig_f**2),
            np.mean(mag), np.max(mag), np.argmax(mag)*(FS/S), np.sum(mag**2)/S, 
            np.sqrt(np.sum((freq_axis - (np.argmax(mag)*(FS/S)))**2 * mag)/np.sum(mag)) if np.sum(mag)!=0 else 0,
            np.mean(psd), np.std(psd), np.sum(psd), freqs[np.argmax(psd)]
        ]
        
        # Map to names
        for i, val in enumerate(vals):
            feat_name = f"{SENSOR_NAMES[s_idx]}_{SUB_NAMES[i]}"
            feats_dict[feat_name] = [val]
            
    return pd.DataFrame(feats_dict)

# --- 3. Live Loop ---
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("127.0.0.1", 5005))
buffer = deque(maxlen=S)
new_data_count = 0

print("System Online. Awaiting data from Rust...")

while True:
    data, addr = sock.recvfrom(1024)
    try:
        raw_values = [float(x) for x in data.decode().split(',')]
        if len(raw_values) >= 13:
            buffer.append(raw_values[:13])
            new_data_count += 1
    except: continue

    if len(buffer) == S and new_data_count >= STEP:
        # A. Feature Extraction
        df_features = extract_features_to_df(np.array(buffer))
        
        # B. Align with Model (Ensure columns are in correct order)
        df_final = df_features[expected_features].fillna(0).replace([np.inf, -np.inf], 0)
        
        # C. Classification
        prediction = rf_model.predict(df_final)[0]
        probs = rf_model.predict_proba(df_final)
        confidence = np.max(probs) * 100
        
        # D. Output
        print(f"Terrain: {prediction:<12} | Confidence: {confidence:.2f}%")
        new_data_count = 0