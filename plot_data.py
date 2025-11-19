import pandas as pd
import matplotlib.pyplot as plt
import os

# ====================================================================
# [1] 설정 (로그 파일 이름)
# ====================================================================
# <<< 분석할 CSV 파일 경로를 지정하세요. (예: smc_log_20251119_180000.csv)
FILE_PATH = 'smc_log_YYYYMMDD_HHMMSS.csv' 

# 참고용 SMC 파라미터 (그래프 제목 표기에 사용)
LAMBDA_LAT = 'Arduino Setting' 
LAMBDA_FRONT = 'Arduino Setting' 

# ====================================================================
# [2] 데이터 불러오기 및 SMC 분석 (계산 없이 바로 사용)
# ====================================================================
if not os.path.exists(FILE_PATH):
    print(f"Error: File not found at {FILE_PATH}")
    exit()

df = pd.read_csv(FILE_PATH)

# df.dropna()를 사용하여 미분 계산 시 발생한 첫 행의 NaN을 제거할 필요가 없습니다.
# 데이터가 이미 깨끗하게 로깅되었다고 가정합니다.

print(f"Loaded and processed {len(df)} data points from {FILE_PATH}")

# ====================================================================
# [3] 그래프 시각화 (3개 서브플롯)
# ====================================================================
fig, axes = plt.subplots(3, 1, figsize=(14, 12), sharex=True)
plt.subplots_adjust(hspace=0.3) 

# --- A. 횡방향 오차 및 제어 출력 (Subplot 1)
axes[0].plot(df.index, df['ErrorLat'], label='Lateral Error ($e_{lat}$)', color='blue')
axes[0].plot(df.index, df['uLat'], label='uLat (Control Output)', color='red', linestyle='--')
axes[0].axhline(0, color='gray', linestyle=':')
axes[0].set_title('Lateral Control Analysis (Error and Control Signal)')
axes[0].set_ylabel('Value')
axes[0].legend()
axes[0].grid(True)

# --- B. 슬라이딩 변수 s (Subplot 2) - Arduino에서 계산된 값을 사용
# 
axes[1].plot(df.index, df['s_lat'], label='Sliding Variable ($s_{lat}$)', color='darkgreen')
axes[1].plot(df.index, df['s_front'], label='Sliding Variable ($s_{front}$)', color='purple')
axes[1].axhline(0, color='black', linestyle='-')
axes[1].set_title(f'SMC Sliding Variables ($s$) - $\lambda_{{lat}}: {LAMBDA_LAT}$, $\lambda_{{front}}: {LAMBDA_FRONT}$')
axes[1].set_ylabel('$s$ Value')
axes[1].legend()
axes[1].grid(True)

# --- C. 최종 PWM 출력 (Subplot 3)
axes[2].plot(df.index, df['PWM_L'], label='PWM_L', color='C1')
axes[2].plot(df.index, df['PWM_R'], label='PWM_R', color='C2')
axes[2].axhline(255, color='red', linestyle=':')
axes[2].axhline(0, color='red', linestyle=':')
axes[2].set_title('Final Motor PWM Output')
axes[2].set_xlabel('Time Step (k)')
axes[2].set_ylabel('PWM (0-255)')
axes[2].legend()
axes[2].grid(True)

plt.show()