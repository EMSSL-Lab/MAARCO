# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "matplotlib",
#     "numpy",
# ]
# ///
from pathlib import Path
from dataclasses import dataclass
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import csv
import argparse

@dataclass
class ParsedGpsData:
    lat: float
    long: float
    alt: float
    fix_quality: str

class CsvGpsParser:
    def __init__(self, file_path: Path):
        self.file_path = file_path
        self.data = []

    def parse_csv(self):
        try:
            with self.file_path.open('r', newline='') as csvfile:
                reader = csv.DictReader(csvfile)
                for row in reader:
                    # Check if we have valid lat/long/alt
                    if not row.get('latitude') or not row.get('longitude'):
                        continue
                    
                    try:
                        lat = float(row['latitude'])
                        long = float(row['longitude'])
                        alt = float(row.get('altitude', 0.0))
                        fix_quality = row.get('gga_fix_quality', 'Invalid')
                        
                        self.data.append(ParsedGpsData(lat, long, alt, fix_quality))
                    except ValueError:
                        continue
                        
        except Exception as e:
            print(f"Error parsing CSV: {e}")

    def plot_deviation_map(self):
        if not self.data:
            print("No data to plot")
            return
        
        lats = [d.lat for d in self.data]
        lons = [d.long for d in self.data]
        qualities = [d.fix_quality for d in self.data]
        
        mean_lat = np.mean(lats)
        mean_lon = np.mean(lons)
        cos_lat = math.cos(mean_lat * math.pi / 180)
        
        # Convert to cm relative
        easts = [(lon - mean_lon) * 111320 * cos_lat * 100 for lon in lons]  # cm
        norths = [(lat - mean_lat) * 111320 * 100 for lat in lats]  # cm
        
        # Colors
        # Mapping based on src/logging.rs
        color_map = {
            'GPS Fix': 'red',
            'DGPS': 'blue',
            'Fixed': 'green',
            'Float': 'yellow',
            'Invalid': 'black',
            'Simulation mode': 'magenta'
        }
        
        colors = [color_map.get(q, 'black') for q in qualities]
        
        # Plot
        plt.figure(figsize=(10, 10))
        plt.scatter(easts, norths, c=colors, alpha=0.7)
        plt.xlabel('East (cm)')
        plt.ylabel('North (cm)')
        plt.title(f'Deviation Map "{self.file_path.name}" above ground')
        plt.grid(True)
        
        # Legend
        legend_elements = [
            Line2D([0], [0], marker='o', color='w', label='GPS Fix', markerfacecolor='red', markersize=10),
            Line2D([0], [0], marker='o', color='w', label='DGPS', markerfacecolor='blue', markersize=10),
            Line2D([0], [0], marker='o', color='w', label='Fixed', markerfacecolor='green', markersize=10),
            Line2D([0], [0], marker='o', color='w', label='Float', markerfacecolor='yellow', markersize=10),
        ]
        plt.legend(handles=legend_elements, loc='upper right')
        plt.show()

    def plot_3d_deviation_map(self):
        if not self.data:
            print("No data to plot")
            return
            
        lats = [d.lat for d in self.data]
        lons = [d.long for d in self.data]
        alts = [d.alt for d in self.data]
        qualities = [d.fix_quality for d in self.data]
        
        mean_lat = np.mean(lats)
        mean_lon = np.mean(lons)
        mean_alt = np.mean(alts)
        cos_lat = math.cos(mean_lat * math.pi / 180)
        
        # Convert to cm relative
        easts = [(lon - mean_lon) * 111320 * cos_lat * 100 for lon in lons]  # cm
        norths = [(lat - mean_lat) * 111320 * 100 for lat in lats]  # cm
        ups = [(alt - mean_alt) * 100 for alt in alts]  # cm
        
        # Colors
        color_map = {
            'GPS Fix': 'red',
            'DGPS': 'blue',
            'Fixed': 'green',
            'Float': 'yellow',
            'Invalid': 'black',
            'Simulation mode': 'magenta'
        }
        
        colors = [color_map.get(q, 'black') for q in qualities]
        
        # Plot
        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(projection='3d')
        ax.scatter(easts, norths, ups, c=colors, alpha=0.7)
        ax.set_xlabel('East (cm)')
        ax.set_ylabel('North (cm)')
        ax.set_zlabel('Up (cm)')
        ax.set_title(f'3D Deviation Map of "{self.file_path.name}" above ground')
        
        # Legend
        legend_elements = [
            Line2D([0], [0], marker='o', color='w', label='GPS Fix', markerfacecolor='red', markersize=10),
            Line2D([0], [0], marker='o', color='w', label='DGPS', markerfacecolor='blue', markersize=10),
            Line2D([0], [0], marker='o', color='w', label='Fixed', markerfacecolor='green', markersize=10),
            Line2D([0], [0], marker='o', color='w', label='Float', markerfacecolor='yellow', markersize=10),
        ]
        ax.legend(handles=legend_elements, loc='upper right')
        plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Plot GPS log data')
    parser.add_argument('file', type=Path, help='Path to the GPS CSV log file')
    args = parser.parse_args()

    if not args.file.exists():
        print(f"File {args.file} does not exist")
        exit(1)

    parser = CsvGpsParser(args.file)
    parser.parse_csv()
    parser.plot_deviation_map()
    parser.plot_3d_deviation_map()
