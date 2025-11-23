import pandas as pd
import matplotlib.pyplot as plt
import os

# ====================================================================
# [1] 설정 (로그 파일 이름)
# ====================================================================
# 로그 수집 후 생성된 CSV 파일 이름을 지정해야함
FILE_PATH = 'smc_log_YYYYMMDD_HHMMSS.csv' 
TS = 0.05 # 제어 루프 주기 (50ms) -> 시간 계산 기준 (PC Time이 없을 때 대체용)

# 참고용 SMC 파라미터 
LAMBDA_LAT = 'Arduino Setting' 
LAMBDA_FRONT = 'Arduino Setting' 

# ====================================================================
# [2] 데이터 불러오기 및 시간 계산 (수정됨)
# ====================================================================
if not os.path.exists(FILE_PATH):
    print(f"Error: File not found at {FILE_PATH}")
    exit()

df = pd.read_csv(FILE_PATH)

print(f"Loaded and processed {len(df)} data points from {FILE_PATH}")

# --- 핵심 수정: Time 컬럼 처리 ---
# 1. 'Time' 컬럼을 datetime 객체로 변환
df['Time'] = pd.to_datetime(df['Time'])

# 2. 첫 번째 행의 시간을 기준으로 상대적인 시간(초) 컬럼 생성
start_time = df['Time'].iloc[0]
df['Time (s)'] = (df['Time'] - start_time).dt.total_seconds()
# ------------------------------------


# ====================================================================
# [3] 그래프 시각화 (3개 서브플롯) - X축을 'Time (s)'로 유지
# ====================================================================
fig, axes = plt.subplots(3, 1, figsize=(14, 12), sharex=True)
plt.subplots_adjust(hspace=0.3) 

# --- A. 횡방향 오차 및 제어 출력 (Subplot 1)
# X축: df['Time (s)'] 사용
axes[0].plot(df['Time (s)'], df['ErrorLat'], label='Lateral Error ($e_{lat}$)', color='blue')
axes[0].plot(df['Time (s)'], df['uLat'], label='uLat (Control Output)', color='red', linestyle='--')
axes[0].axhline(0, color='gray', linestyle=':')
axes[0].set_title('Lateral Control Analysis (Error and Control Signal)')
axes[0].set_ylabel('Value')
axes[0].legend()
axes[0].grid(True)

# --- B. 슬라이딩 변수 s (Subplot 2)
# X축: df['Time (s)'] 사용
axes[1].plot(df['Time (s)'], df['s_lat'], label='Sliding Variable ($s_{lat}$)', color='darkgreen')
axes[1].plot(df['Time (s)'], df['s_front'], label='Sliding Variable ($s_{front}$)', color='purple')
axes[1].axhline(0, color='black', linestyle='-')
axes[1].set_title(f'SMC Sliding Variables ($s$) - $\lambda_{{lat}}: {LAMBDA_LAT}$, $\lambda_{{front}}: {LAMBDA_FRONT}$')
axes[1].set_ylabel('$s$ Value')
axes[1].legend()
axes[1].grid(True)


# --- C. 최종 PWM 출력 (Subplot 3)
# X축: df['Time (s)'] 사용
axes[2].plot(df['Time (s)'], df['PWM_L'], label='PWM_L', color='C1')
axes[2].plot(df['Time (s)'], df['PWM_R'], label='PWM_R', color='C2')
axes[2].axhline(255, color='red', linestyle=':')
axes[2].axhline(0, color='red', linestyle=':')
axes[2].set_title('Final Motor PWM Output')
axes[2].set_xlabel('Time (s)') # X축 라벨을 'Time (s)'로 유지
axes[2].set_ylabel('PWM (0-255)')
axes[2].legend()
axes[2].grid(True)

plt.show()