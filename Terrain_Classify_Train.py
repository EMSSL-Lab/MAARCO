import pandas as pd
import glob
import os
from sklearn.model_selection import train_test_split
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import classification_report, confusion_matrix, accuracy_score
import joblib

# 1. Load all CSV files
# all_files = glob.glob("Normalized_Data\*.csv")
all_files = glob.glob(r"Normalized_Data\**\*.csv", recursive=True)
print(all_files)
data_list = []

for f in all_files:
    temp_df = pd.read_csv(f)
    
    label = os.path.basename(f).split('_')[0]   # HardIce, SoftSnow, etc
    temp_df['Label'] = label
    temp_df['SourceFile'] = f
    
    data_list.append(temp_df)

master_df = pd.concat(data_list, ignore_index=True)

# 2. Get unique files
unique_files = master_df['SourceFile'].unique()

# # 3. Split FILES (not rows)
# train_files, test_files = train_test_split(
#     unique_files,
#     test_size=0.2,
#     random_state=42,
#     stratify=file_labels
# )

# 3. Split FILES with Stratification
# Create a list of labels corresponding to each unique file
file_labels = [os.path.basename(f).split('_')[0] for f in unique_files]

train_files, test_files = train_test_split(
    unique_files,
    test_size=0.2,
    random_state=42,
    stratify=file_labels  # This ensures the 80/20 split happens per category
)


# 4. Create train/test datasets based on file membership
train_df = master_df[master_df['SourceFile'].isin(train_files)]
test_df  = master_df[master_df['SourceFile'].isin(test_files)]

# 5. Separate features and labels
X_train = train_df.drop(['Label','SourceFile'], axis=1)
y_train = train_df['Label']

X_test = test_df.drop(['Label','SourceFile'], axis=1)
y_test = test_df['Label']

# 6. Train Random Forest (no scaling needed)
rf = RandomForestClassifier(
    n_estimators=300,
    random_state=42,
    n_jobs=-1
)

rf.fit(X_train, y_train)

# 7. Evaluate
y_pred = rf.predict(X_test)

print("Accuracy:", accuracy_score(y_test, y_pred))
print("\nConfusion Matrix:\n", confusion_matrix(y_test, y_pred))
print("\nClassification Report:\n", classification_report(y_test, y_pred))

# 8. Save model
joblib.dump(rf, "terrain_rf_model.pkl")

print("\nModel saved successfully.")
