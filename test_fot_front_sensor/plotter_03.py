# plotter_2nd.py
import pandas as pd
import matplotlib.pyplot as plt
import glob
import os
import sys

files = glob.glob("robot_log_*.csv")
if not files:
    print("Error: No log files found in current directory")
    print("Make sure logger_2nd.py was run and created CSV files")
    sys.exit(1)

latest_file = max(files, key=os.path.getctime)
print(f"Plotting data from: {latest_file}")

try:
    df = pd.read_csv(latest_file)
except pd.errors.EmptyDataError:
    print("Error: CSV file is empty")
    sys.exit(1)
except Exception as e:
    print(f"Error reading CSV file: {e}")
    sys.exit(1)

required_cols = ['time_sec', 'raw_dist_cm', 'lpf_dist_cm', 'error_cm',
                 'e_dot_lpf_cms', 's_value', 'target_distance_cm', 'state']

missing = [col for col in required_cols if col not in df.columns]
if missing:
    print(f"Error: Missing required columns: {missing}")
    sys.exit(1)

for col in required_cols[:-1]:
    df[col] = pd.to_numeric(df[col], errors='coerce')

df = df.dropna()

if len(df) == 0:
    print("Error: No valid data rows in CSV file")
    sys.exit(1)

print(f"Loaded {len(df)} data points")
print(f"Time range: {df['time_sec'].min():.2f}s to {df['time_sec'].max():.2f}s")

time = df['time_sec']
raw = df['raw_dist_cm']
lpf = df['lpf_dist_cm']
error = df['error_cm']
e_dot = df['e_dot_lpf_cms']
s_val = df['s_value']
target = df['target_distance_cm']
state = df['state']

plt.figure(figsize=(12,6))
plt.plot(time, raw, label='Raw Distance', alpha=0.5)
plt.plot(time, lpf, label='LPF Distance', linewidth=2)
plt.plot(time, target, '--', label='Target Distance', linewidth=2)
plt.xlabel('Time [s]')
plt.ylabel('Distance [cm]')
plt.title('Robot Distance vs Time')
plt.grid(True)
plt.legend()

unique_states = df['state'].unique()
colors = plt.cm.tab10.colors
for i, st in enumerate(unique_states):
    idx = df['state'] == st
    plt.scatter(time[idx].iloc[::5], lpf[idx].iloc[::5], 
                color=colors[i % len(colors)], 
                label=f'State: {st}', s=20, alpha=0.6)

plt.legend()

plt.figure(figsize=(12,8))

plt.subplot(3,1,1)
plt.plot(time, error, label='Error')
plt.ylabel('Error [cm]')
plt.grid(True)
plt.legend()

plt.subplot(3,1,2)
plt.plot(time, e_dot, label='Error Rate (LPF)', color='orange')
plt.ylabel('Error Rate [cm/s]')
plt.grid(True)
plt.legend()

plt.subplot(3,1,3)
plt.plot(time, s_val, label='Sliding Surface', color='green')
plt.xlabel('Time [s]')
plt.ylabel('s value')
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()

print("Plotting complete")
