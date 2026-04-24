# /// script
# requires-python = ">=3.13"
# dependencies = [
# "matplotlib",
# "numpy",
# "pandas",
# ]
# ///

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

# --- CONFIGURATION ---
GPS_FILE = Path("logs/2026-04-17_14-20-12_gps.csv")
EKF_FILE = Path("logs/2026-04-17_14-20-12_ekf.csv")
# ---------------------

GPS_FILE = Path("logs/2026-04-17_14-26-35_gps.csv")
EKF_FILE = Path("logs/2026-04-17_14-26-35_ekf.csv")



def plot_realigned():
    if not GPS_FILE.exists() or not EKF_FILE.exists(): return

    gps_df = pd.read_csv(GPS_FILE)
    ekf_df = pd.read_csv(EKF_FILE)

    # 1. GPS Reference
    ref_lat, ref_lon = gps_df['latitude'].iloc[0], gps_df['longitude'].iloc[0]
    cos_lat = np.cos(np.radians(ref_lat))
    gps_east = (gps_df['longitude'] - ref_lon) * (111320 * cos_lat)
    gps_north = (gps_df['latitude'] - ref_lat) * 111132

    # 2. RE-MAPPING FOR 90-DEGREE ROTATION
    # Based on your latest plot, we swap x and y back without inversions
    ekf_east = ekf_df['pos_x'] 
    ekf_north = ekf_df['pos_y']

    # 3. Plotting
    plt.figure(figsize=(10, 10))
    plt.plot(ekf_east, ekf_north, label='EKF', color='blue', marker='o', linestyle='-', markersize=4, alpha=0.8)
    plt.plot(gps_east, gps_north, label='GPS', color='red', marker='o', linestyle='--', markersize=4, alpha=0.6)
    
    gps_start = (gps_east.iloc[0], gps_north.iloc[0])
    gps_end = (gps_east.iloc[-1], gps_north.iloc[-1])
    ekf_start = (ekf_east.iloc[0], ekf_north.iloc[0])
    ekf_end = (ekf_east.iloc[-1], ekf_north.iloc[-1])

    plt.scatter(*ekf_start, color='green', edgecolor='black', s=140, marker='o')
    plt.scatter(*ekf_end, color='green', edgecolor='black', s=140, marker='X')

    plt.scatter(*gps_start, color='green', edgecolor='black', s=140, marker='o', label='Start/End')
    plt.scatter(*gps_end, color='green', edgecolor='black', s=140, marker='X')
    
    annotate_kwargs = dict(color='green', fontsize=8, bbox=dict(boxstyle='round,pad=0.2', fc='white', alpha=0.7))
    plt.annotate(f"GPS start: ({gps_start[0]:.3f}, {gps_start[1]:.3f})",
                 xy=gps_start, xytext=(5, 5), textcoords='offset points',
                 ha='left', va='bottom', **annotate_kwargs)
    plt.annotate(f"GPS end: ({gps_end[0]:.3f}, {gps_end[1]:.3f})",
                 xy=gps_end, xytext=(5, 5), textcoords='offset points',
                 ha='left', va='bottom', **annotate_kwargs)
    plt.annotate(f"EKF start: ({ekf_start[0]:.3f}, {ekf_start[1]:.3f})",
                 xy=ekf_start, xytext=(5, -5), textcoords='offset points',
                 ha='left', va='top', **annotate_kwargs)
    plt.annotate(f"EKF end: ({ekf_end[0]:.3f}, {ekf_end[1]:.3f})",
                 xy=ekf_end, xytext=(5, -5), textcoords='offset points',
                 ha='left', va='top', **annotate_kwargs)

    plt.gca().set_aspect('equal')
    plt.title("GPS vs EKF")
    plt.xlabel("East (meters)")
    plt.ylabel("North (meters)")
    plt.legend()
    plt.grid(True, alpha=0.3)
    ax = plt.gca() # Get current axes
    ax.set_aspect('equal')
    plt.show()

if __name__ == "__main__":
    plot_realigned()