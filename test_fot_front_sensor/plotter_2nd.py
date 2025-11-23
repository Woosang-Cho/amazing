import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys

CSV_FILE = sys.argv[1] if len(sys.argv) > 1 else None  # 커맨드라인 인자

if CSV_FILE is None:
    print("Usage: python plotter.py <csv_filename>")
    print("Example: python plotter.py robot_data_20251120_015800.csv")
    sys.exit(1)

# ==========================================
# CSV 데이터 로드
# ==========================================
try:
    df = pd.read_csv(CSV_FILE)
    print(f"   Loaded {len(df)} samples from {CSV_FILE}")
    print(f"   Duration: {df['time_sec'].iloc[-1]:.2f} seconds")
    print()
except FileNotFoundError:
    print(f"File not found: {CSV_FILE}")
    sys.exit(1)
except Exception as e:
    print(f"Error loading CSV: {e}")
    sys.exit(1)

# ==========================================
# 그래프 그리기
# ==========================================
fig, axes = plt.subplots(4, 1, figsize=(14, 12))
fig.suptitle(f'SMC Robot Data Analysis - {CSV_FILE}', fontsize=14, fontweight='bold')

# 1. 센서 거리
axes[0].plot(df['time_sec'], df['left_dist'], label='Left Distance', color='blue', linewidth=1.5)
axes[0].plot(df['time_sec'], df['right_dist'], label='Right Distance', color='green', linewidth=1.5)
axes[0].plot(df['time_sec'], df['front_dist'], label='Front Distance', color='red', linewidth=1.5)
axes[0].axhline(y=6.0, color='red', linestyle='--', alpha=0.5, label='Stop Threshold (6cm)')
axes[0].set_ylabel('Distance (cm)', fontsize=11)
axes[0].set_title('1. Ultrasonic Sensor Readings', fontsize=12, fontweight='bold')
axes[0].legend(loc='upper right')
axes[0].grid(True, alpha=0.3)
axes[0].set_xlim(df['time_sec'].min(), df['time_sec'].max())

# 2. 횡방향 에러 및 제어
ax2_1 = axes[1]
ax2_2 = ax2_1.twinx()

ax2_1.plot(df['time_sec'], df['error_lat'], label='Error (L-R)', color='purple', linewidth=1.5)
ax2_1.axhline(y=0, color='k', linestyle='--', alpha=0.3)
ax2_1.set_ylabel('Error (cm)', fontsize=11, color='purple')
ax2_1.tick_params(axis='y', labelcolor='purple')

ax2_2.plot(df['time_sec'], df['u_lat'], label='uLat (Control)', color='orange', linewidth=1.5)
ax2_2.set_ylabel('Control Output uLat', fontsize=11, color='orange')
ax2_2.tick_params(axis='y', labelcolor='orange')

axes[1].set_title('2. Lateral Error & Control Output', fontsize=12, fontweight='bold')
ax2_1.grid(True, alpha=0.3)
ax2_1.set_xlim(df['time_sec'].min(), df['time_sec'].max())

# 범례 합치기
lines1, labels1 = ax2_1.get_legend_handles_labels()
lines2, labels2 = ax2_2.get_legend_handles_labels()
ax2_1.legend(lines1 + lines2, labels1 + labels2, loc='upper right')

# 3. SMC 제어 출력
axes[2].plot(df['time_sec'], df['u_front'], label='uFront (Longitudinal)', color='red', linewidth=1.5)
axes[2].plot(df['time_sec'], df['u_lat'], label='uLat (Lateral)', color='orange', linewidth=1.5)
axes[2].axhline(y=0, color='k', linestyle='--', alpha=0.3)
axes[2].axhline(y=50, color='orange', linestyle=':', alpha=0.5, label='uLat Limit (±50)')
axes[2].axhline(y=-50, color='orange', linestyle=':', alpha=0.5)
axes[2].set_ylabel('Control Output', fontsize=11)
axes[2].set_title('3. SMC Control Outputs', fontsize=12, fontweight='bold')
axes[2].legend(loc='upper right')
axes[2].grid(True, alpha=0.3)
axes[2].set_xlim(df['time_sec'].min(), df['time_sec'].max())

# 4. PWM 출력
axes[3].plot(df['time_sec'], df['pwm_left'], label='Left Motor PWM', color='blue', linewidth=1.5)
axes[3].plot(df['time_sec'], df['pwm_right'], label='Right Motor PWM', color='green', linewidth=1.5)
axes[3].axhline(y=120, color='gray', linestyle='--', alpha=0.5, label='Base PWM (120)')
axes[3].set_xlabel('Time (seconds)', fontsize=11)
axes[3].set_ylabel('PWM Value', fontsize=11)
axes[3].set_title('4. Motor PWM Commands', fontsize=12, fontweight='bold')
axes[3].legend(loc='upper right')
axes[3].grid(True, alpha=0.3)
axes[3].set_xlim(df['time_sec'].min(), df['time_sec'].max())
axes[3].set_ylim(0, 255)

plt.tight_layout()
plt.show()

# ==========================================
# 통계 정보 출력
# ==========================================
print("\n" + "=" * 60)
print("Data Statistics")
print("=" * 60)
print(f"Sensor Distances:")
print(f"  Left:  min={df['left_dist'].min():.2f}, max={df['left_dist'].max():.2f}, avg={df['left_dist'].mean():.2f}")
print(f"  Right: min={df['right_dist'].min():.2f}, max={df['right_dist'].max():.2f}, avg={df['right_dist'].mean():.2f}")
print(f"  Front: min={df['front_dist'].min():.2f}, max={df['front_dist'].max():.2f}, avg={df['front_dist'].mean():.2f}")
print(f"\nLateral Error:")
print(f"  min={df['error_lat'].min():.2f}, max={df['error_lat'].max():.2f}, avg={df['error_lat'].mean():.2f}, std={df['error_lat'].std():.2f}")
print(f"\nControl Outputs:")
print(f"  uFront: min={df['u_front'].min():.2f}, max={df['u_front'].max():.2f}, avg={df['u_front'].mean():.2f}")
print(f"  uLat:   min={df['u_lat'].min():.2f}, max={df['u_lat'].max():.2f}, avg={df['u_lat'].mean():.2f}")
print(f"\nPWM Commands:")
print(f"  Left:  min={df['pwm_left'].min()}, max={df['pwm_left'].max()}, avg={df['pwm_left'].mean():.1f}")
print(f"  Right: min={df['pwm_right'].min()}, max={df['pwm_right'].max()}, avg={df['pwm_right'].mean():.1f}")
print("=" * 60)
