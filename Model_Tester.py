import pandas as pd
import joblib

# 1. Load the saved model
model_path = "terrain_rf_model.pkl"
rf_model = joblib.load(model_path)

def predict_terrain(csv_path):
    # 2. Load the new data
    new_data = pd.read_csv(csv_path)
    
    # new_data.columns = new_data.columns.str.strip()

    # 3. Clean the data
    # IMPORTANT: The model expects the exact same features used during training.
    # We must drop 'Label' or 'SourceFile' if they exist in the new CSV.
    features = new_data.drop(['Label', 'SourceFile'], axis=1, errors='ignore')
    
    # 4. Make Predictions
    # This will give a prediction for EVERY row in the CSV
    predictions = rf_model.predict(features)
    
    # 5. Determine the overall result
    # Often, sensor data is 'noisy', so we take the most frequent prediction (mode)
    final_prediction = pd.Series(predictions).mode()[0]
    
    # Get probabilities (confidence)
    probs = rf_model.predict_proba(features)
    avg_confidence = probs.max(axis=1).mean() * 100
    
    return final_prediction, avg_confidence

# --- Example Usage ---
new_file = r"C:\Users\jcohe\OneDrive\Documents\Python_Projects\PhD_Research\Data_Review\Classifier\Test_Data\HardIce_01_26_60RPM_test1_u2.csv"
new_file2 = r"C:\Users\jcohe\OneDrive\Documents\Python_Projects\PhD_Research\Data_Review\Classifier\Test_Data\SoftSnow_02_02_150RPM_test4_u5.csv"
new_file3 = r"C:\Users\jcohe\OneDrive\Documents\Python_Projects\PhD_Research\Data_Review\Classifier\Test_Data\WetSand_1_Feb5_26_u2.csv"

terrain2, confidence2 = predict_terrain(new_file3)

print(f"Predicted Terrain: {terrain2}")
print(f"Confidence Level: {confidence2:.2f}%")