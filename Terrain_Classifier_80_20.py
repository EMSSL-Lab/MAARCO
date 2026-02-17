
# ======================================80-20 Split============================================================ 
import pandas as pd
import glob
import os
import time
import joblib
import matplotlib.pyplot as plt

from sklearn.model_selection import train_test_split
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import classification_report, confusion_matrix, accuracy_score
from sklearn.pipeline import Pipeline
from sklearn.preprocessing import StandardScaler
from sklearn.svm import SVC
from sklearn.neighbors import KNeighborsClassifier
from sklearn.ensemble import GradientBoostingClassifier
from sklearn.tree import DecisionTreeClassifier
import numpy as np
from sklearn.model_selection import GroupKFold
from sklearn.metrics import accuracy_score


# 1. Load all CSV files
all_files = glob.glob(r"Normalized_Data\**\*.csv", recursive=True)
print(f"Found {len(all_files)} files.")
data_list = []

for f in all_files:
    temp_df = pd.read_csv(f)
    label = os.path.basename(f).split('_')[0] 
    temp_df['Label'] = label
    temp_df['SourceFile'] = f
    data_list.append(temp_df)

master_df = pd.concat(data_list, ignore_index=True)

# Replace all NaNs with 0
master_df = master_df.fillna(0)
# Also handle Infinity (which often occurs in FFT/PSD calculations)
master_df = master_df.replace([np.inf, -np.inf], 0)

# 1. Create a dataframe of just the numeric features
numeric_df = master_df.select_dtypes(include=[np.number])

# 2. Check for NaNs and Infs ONLY in numeric data
cols_with_nan = numeric_df.columns[numeric_df.isna().any()].tolist()
cols_with_inf = numeric_df.columns[np.isinf(numeric_df).any()].tolist()

print(f"Columns containing NaN: {cols_with_nan}")
print(f"Columns containing Infinity: {cols_with_inf}")

# 3. If you found any, let's see how many
if cols_with_inf or cols_with_nan:
    for col in set(cols_with_inf + cols_with_nan):
        nans = numeric_df[col].isna().sum()
        infs = np.isinf(numeric_df[col]).sum()
        print(f"--> Column '{col}': {nans} NaNs, {infs} Infs")


# 2. Split FILES (Leakage Protection)
unique_files = master_df['SourceFile'].unique()
file_labels = [os.path.basename(f).split('_')[0] for f in unique_files]

train_files, test_files = train_test_split(
    unique_files,
    test_size=0.2,
    random_state=42,
    stratify=file_labels
)

train_df = master_df[master_df['SourceFile'].isin(train_files)]
test_df  = master_df[master_df['SourceFile'].isin(test_files)]

# --- 3. DYNAMIC FEATURE SELECTION ---
# We grab columns by their prefix so you don't have to type all 150+ names
all_cols = master_df.columns

# COMMENT OUT any line below to exclude that entire sensor group
groups_to_include = [
    [c for c in all_cols if c.startswith(('TorqL', 'TorqR'))], # Torque Features
    [c for c in all_cols if c.startswith(('IavL', 'IavR'))],   # Current Features
    [c for c in all_cols if c.startswith(('AccX', 'AccY', 'AccZ'))], # Acceleration
    [c for c in all_cols if c.startswith('Sonar')],            # Sonar
    [c for c in all_cols if c.startswith('ToF')],              # Time of Flight
    [c for c in all_cols if c.startswith(('RPML', 'RPMR'))]    # RPM Features
]

# Flatten the list of lists into a single list of strings
selected_features = [item for sublist in groups_to_include for item in sublist]

X_train = train_df[selected_features]
y_train = train_df['Label']
X_test = test_df[selected_features]
y_test = test_df['Label']

print(f"Training shape: {X_train.shape}")
print(f"Testing shape: {X_test.shape}")

print(f"\nTraining on {len(selected_features)} features.")

# --- 4. TRAIN WITH TIMER ---
print("Starting training...")
start_time = time.time()

# --- 4. DEFINE MODELS ---
# We use make_pipeline(StandardScaler(), ...) for SVM and KNN 
# because they require scaled data to function correctly with raw inputs.
models = {
    "Decision Tree": DecisionTreeClassifier(random_state=42),
    "Random Forest": RandomForestClassifier(n_estimators=300, random_state=42, n_jobs=-1),
    "Gradient Boosting": GradientBoostingClassifier(n_estimators=100, random_state=42),
    "SVM (Scaled)": Pipeline([('scaler', StandardScaler()), ('svm', SVC(probability=True))]),
    "KNN (Scaled)": Pipeline([('scaler', StandardScaler()), ('knn', KNeighborsClassifier(n_neighbors=5))])
}

results = []

print(f"\nBenchmarking {len(models)} classifiers on {len(selected_features)} features...")

for name, clf in models.items():
    # Timer start
    start_time = time.time()
    
    # Train
    clf.fit(X_train, y_train)
    
    # Timer end
    elapsed = time.time() - start_time
    
    # Evaluate
    y_pred = clf.predict(X_test)
    acc = accuracy_score(y_test, y_pred)
    
    results.append({"Model": name, "Accuracy": acc, "Time (s)": round(elapsed, 4)})
    print(f"Finished {name:20} | Acc: {acc:.4f} | Time: {elapsed:.4f}s")

    print("\nConfusion Matrix:\n", confusion_matrix(y_test, y_pred))
    print("\nClassification Report:\n", classification_report(y_test, y_pred))


DT = models["Decision Tree"]
joblib.dump(DT, "terrain_dt_model_80_20.pkl")

RF = models["Random Forest"]
joblib.dump(RF, "terrain_rf_model_80_20.pkl")

GB = models["Gradient Boosting"]
joblib.dump(GB, "terrain_gb_model_80_20.pkl")

SVM = models["SVM (Scaled)"]
joblib.dump(SVM, "terrain_svm_model_80_20.pkl")

KNN = models["KNN (Scaled)"]
joblib.dump(KNN, "terrain_knn_model_80_20.pkl")

# --- 5. FINAL COMPARISON ---
df_results = pd.DataFrame(results).sort_values(by="Accuracy", ascending=False)
print("\n--- Final Model Rankings ---")
print(df_results)
