# import pandas as pd
# import joblib

# # 1. Load the saved model
# model_path = "terrain_rf_model.pkl"
# rf_model = joblib.load(model_path)

# def predict_terrain(csv_path):
#     # 2. Load the new data
#     new_data = pd.read_csv(csv_path)
    
#     # new_data.columns = new_data.columns.str.strip()

#     # 3. Clean the data
#     # IMPORTANT: The model expects the exact same features used during training.
#     # We must drop 'Label' or 'SourceFile' if they exist in the new CSV.
#     features = new_data.drop(['Label', 'SourceFile'], axis=1, errors='ignore')
    
#     # 4. Make Predictions
#     # This will give a prediction for EVERY row in the CSV
#     predictions = rf_model.predict(features)
    
#     # 5. Determine the overall result
#     # Often, sensor data is 'noisy', so we take the most frequent prediction (mode)
#     final_prediction = pd.Series(predictions).mode()[0]
    
#     # Get probabilities (confidence)
#     probs = rf_model.predict_proba(features)
#     avg_confidence = probs.max(axis=1).mean() * 100
    
#     return final_prediction, avg_confidence

# # --- Example Usage ---
# new_file = r"C:\Users\jcohe\OneDrive\Documents\Python_Projects\PhD_Research\Data_Review\Classifier\Test_Data\HardIce_01_26_60RPM_test1_u2.csv"
# new_file2 = r"C:\Users\jcohe\OneDrive\Documents\Python_Projects\PhD_Research\Data_Review\Classifier\Test_Data\SoftSnow_02_02_150RPM_test4_u5.csv"
# new_file3 = r"C:\Users\jcohe\OneDrive\Documents\Python_Projects\PhD_Research\Data_Review\Classifier\Test_Data\WetSand_1_Feb5_26_u2.csv"

# terrain2, confidence2 = predict_terrain(new_file3)

# print(f"Predicted Terrain: {terrain2}")
# print(f"Confidence Level: {confidence2:.2f}%")

import pandas as pd
import joblib

# 1. Load the saved model
model_path_1 = "terrain_dt_model.pkl"
model_path_2 = "terrain_rf_model.pkl"
model_path_3 = "terrain_gb_model.pkl"
model_path_4 = "terrain_svm_model.pkl"
model_path_5 = "terrain_knn_model.pkl"

rf_model = joblib.load(model_path_1)

def predict_terrain_windowed(csv_path, window_size):
    # 2. Load the data
    new_data = pd.read_csv(csv_path)
    
    new_data = new_data.fillna(0).replace([float('inf'), float('-inf')], 0)

    # 3. Align features (use only the columns the model expects)
    expected_features = rf_model.feature_names_in_
    features = new_data[expected_features]
    
    results = []
    
    # 4. Loop through the data in steps of window_size
    # range(start, stop, step)
    for i in range(0, len(features), window_size):
        # Extract the 2-second chunk (20 rows)
        window = features.iloc[i : i + window_size]
        
        # If the last chunk is too small (e.g., only 5 rows left), you can skip it
        if len(window) < window_size:
            break
            
        # Make predictions for every row in this 20-row window
        window_preds = rf_model.predict(window)
        
        # Take the "Majority Vote" (Mode) for this window
        final_pred = pd.Series(window_preds).mode()[0]
        
        # Calculate confidence for this window
        probs = rf_model.predict_proba(window)
        avg_confidence = probs.max(axis=1).mean() * 100
        
        timestamp = i / 10  # Assuming 10Hz, this is the start time in seconds
        results.append((timestamp, final_pred, avg_confidence))
        
        print(f"Time: {timestamp:>4}s | Terrain: {final_pred:<10} | Conf: {avg_confidence:.2f}%")

    return results

# --- Example Usage ---
new_file = r"C:\Users\jcohe\OneDrive\Documents\Python_Projects\PhD_Research\Data_Review\Classifier\Test_Data\HardIce_01_26_60RPM_test1_u1.csv"
new_file2 = r"C:\Users\jcohe\OneDrive\Documents\Python_Projects\PhD_Research\Data_Review\Classifier\Test_Data\HardIce_01_26_60RPM_test3_u1.csv"
new_file3 = r"C:\Users\jcohe\OneDrive\Documents\Python_Projects\PhD_Research\Data_Review\Classifier\Test_Data\SoftSnow_02_02_60RPM_test3_u2.csv"
new_file4 = r"C:\Users\jcohe\OneDrive\Documents\Python_Projects\PhD_Research\Data_Review\Classifier\Test_Data\SoftSnow_02_02_150RPM_test4_u4.csv"
new_file5 = r"C:\Users\jcohe\OneDrive\Documents\Python_Projects\PhD_Research\Data_Review\Classifier\Test_Data\WetSand_1_Feb5_26_u1.csv"
new_file6 = r"C:\Users\jcohe\OneDrive\Documents\Python_Projects\PhD_Research\Data_Review\Classifier\Test_Data\WetSand_2_Feb5_26_u3.csv"

file = new_file6
print(f"Processing {file} in 2-second windows...\n")
window_results = predict_terrain_windowed(file, 5)
