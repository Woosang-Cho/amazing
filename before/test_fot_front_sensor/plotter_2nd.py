# plot_latest_smc.py
import pandas as pd
import matplotlib.pyplot as plt
import glob
import os

# 최근 CSV 파일 찾기
files = glob.glob("robot_log_*.csv")
if not files:
    print("No log files found.")
    exit()
latest_file = max(files, key=os.path.getctime)
print(f"Plotting {latest_file}")

# CSV 읽기
df = pd.read_csv(latest_file)

# 컬럼 추출
time = df['time_sec']
raw = df['raw_dist_cm']
lpf = df['lpf_dist_cm']
error = df['error_cm']
e_dot = df['e_dot_lpf_cms']
s_val = df['s_value']
target = df['target_distance_cm']
state = df['state']

# 1. 거리 및 목표 그래프
plt.figure(figsize=(12,6))
plt.plot(time, raw, label='Raw Distance')
plt.plot(time, lpf, label='LPF Distance')
plt.plot(time, target, '--', label='Target Distance')
plt.xlabel('Time [s]')
plt.ylabel('Distance [cm]')
plt.title('Robot Distance vs Time')
plt.grid(True)

# 상태별 표시
unique_states = df['state'].unique()
colors = plt.cm.tab10.colors
for i, st in enumerate(unique_states):
    idx = df['state'] == st
    plt.scatter(time[idx], target[idx], label=f"State: {st}", color=colors[i%10], s=10)

plt.legend()
plt.show()

# 2. 오차 및 제어 변수 그래프
plt.figure(figsize=(12,6))
plt.plot(time, error, label='Distance Error (Target - Measured)')
plt.plot(time, e_dot, label='Error Rate (cm/s)')
plt.plot(time, s_val, label='Sliding Variable s')
plt.xlabel('Time [s]')
plt.ylabel('Value')
plt.title('SMC Control Variables vs Time')
plt.grid(True)
plt.legend()
plt.show()
