# import pandas as pd
# import glob
# import os
# from sklearn.model_selection import train_test_split
# from sklearn.ensemble import RandomForestClassifier
# from sklearn.metrics import classification_report, confusion_matrix, accuracy_score
# import joblib
# import time


# # 1. Load all CSV files
# # all_files = glob.glob("Normalized_Data\*.csv")
# all_files = glob.glob(r"Normalized_Data\**\*.csv", recursive=True)
# print(all_files)
# data_list = []

# for f in all_files:
#     temp_df = pd.read_csv(f)
    
#     label = os.path.basename(f).split('_')[0]   # HardIce, SoftSnow, etc
#     temp_df['Label'] = label
#     temp_df['SourceFile'] = f
    
#     data_list.append(temp_df)

# master_df = pd.concat(data_list, ignore_index=True)

# # 2. Get unique files
# unique_files = master_df['SourceFile'].unique()

# # 3. Split FILES with Stratification
# # Create a list of labels corresponding to each unique file
# file_labels = [os.path.basename(f).split('_')[0] for f in unique_files]

# train_files, test_files = train_test_split(
#     unique_files,
#     test_size=0.2,
#     random_state=42,
#     stratify=file_labels  # This ensures the 80/20 split happens per category
# )

# # 4. Create train/test datasets based on file membership
# train_df = master_df[master_df['SourceFile'].isin(train_files)]
# test_df  = master_df[master_df['SourceFile'].isin(test_files)]

# # 5. Separate features and labels
# X_train = train_df.drop(['Label','SourceFile'], axis=1)
# y_train = train_df['Label']

# X_test = test_df.drop(['Label','SourceFile'], axis=1)
# y_test = test_df['Label']

# # 6. Train Random Forest (no scaling needed)
# rf = RandomForestClassifier(
#     n_estimators=300,
#     random_state=42,
#     n_jobs=-1
# )

# rf.fit(X_train, y_train)

# # 7. Evaluate
# y_pred = rf.predict(X_test)

# print("Accuracy:", accuracy_score(y_test, y_pred))
# print("\nConfusion Matrix:\n", confusion_matrix(y_test, y_pred))
# print("\nClassification Report:\n", classification_report(y_test, y_pred))

# # 8. Save model
# joblib.dump(rf, "terrain_rf_model.pkl")

# print("\nModel saved successfully.")

# =======================================================KFOLD CROSS VALIDATION============================================================
import pandas as pd
import glob
import os
import time
import joblib
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

# --- 3. DYNAMIC FEATURE SELECTION ---
# We grab columns by their prefix so you don't have to type all 150+ names
all_cols = master_df.columns

# COMMENT OUT any line below to exclude that entire sensor group
groups_to_include = [
    [c for c in all_cols if c.startswith(('TorqL', 'TorqR'))], # Torque Features
    # [c for c in all_cols if c.startswith(('IavL', 'IavR'))],   # Current Features
    # [c for c in all_cols if c.startswith(('AccX', 'AccY', 'AccZ'))], # Acceleration
    # [c for c in all_cols if c.startswith('Sonar')],            # Sonar
    # [c for c in all_cols if c.startswith('ToF')],              # Time of Flight
    # [c for c in all_cols if c.startswith(('RPML', 'RPMR'))]    # RPM Features
]

# Flatten the list of lists into a single list of strings
selected_features = [item for sublist in groups_to_include for item in sublist]


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

#====================================KFOLD
X = master_df[selected_features]
y = master_df['Label']
groups = master_df['SourceFile']  # critical

gkf = GroupKFold(n_splits=5)

print(f"\nRunning 5-Fold Group Cross Validation...\n")

for name, clf in models.items():
    fold_accuracies = []
    
    # --- 4. TRAIN WITH TIMER ---
    print("Starting training...")
    start_time = time.time()

    all_y_test = []
    all_y_pred = []
    
    for fold, (train_idx, test_idx) in enumerate(gkf.split(X, y, groups)):
        X_train, X_test = X.iloc[train_idx], X.iloc[test_idx]
        y_train, y_test = y.iloc[train_idx], y.iloc[test_idx]
        
        clf.fit(X_train, y_train)
        y_pred = clf.predict(X_test)
        acc = accuracy_score(y_test, y_pred)
        
        all_y_test.extend(y_test)
        all_y_pred.extend(y_pred)
        
        fold_accuracies.append(acc)
        print(f"{name:20} | Fold {fold+1} Accuracy: {acc:.4f}")
    

    print(f"{name:20} | Mean Accuracy: {np.mean(fold_accuracies):.4f}")
    print("-" * 60)

    train_duration = time.time() - start_time
    print("-" * 30)
    print(f"TRAINING TIME: {train_duration:.4f} seconds")
    print("-" * 30)

    print("\nConfusion Matrix:\n", confusion_matrix(all_y_test, all_y_pred))
    print("\nClassification Report:\n", classification_report(all_y_test, all_y_pred))

DT = models["Decision Tree"]
joblib.dump(DT, "terrain_dt_model_KFOLD_trq.pkl")

RF = models["Random Forest"]
joblib.dump(RF, "terrain_rf_model_KFOLD_trq.pkl")

GB = models["Gradient Boosting"]
joblib.dump(GB, "terrain_gb_model_KFOLD_trq.pkl")

SVM = models["SVM (Scaled)"]
joblib.dump(SVM, "terrain_svm_model_KFOLD_trq.pkl")

KNN = models["KNN (Scaled)"]
joblib.dump(KNN, "terrain_knn_model_KFOLD_trq.pkl")