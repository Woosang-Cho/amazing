import serial
import datetime
import csv
import os
import sys
import time

# ====================================================================
# [1] 설정 파라미터 변경 (총 12개 변수 수신)
# ====================================================================
SERIAL_PORT = 'COM3'  # <<< 현재 아두이노가 연결된 포트로 반드시 변경하세요.
BAUD_RATE = 115200
NUM_VARIABLES = 12 

# CSV 파일 헤더
VARIABLE_NAMES = [
    "L_Dist", "R_Dist", "F_Dist", "ErrorLat", 
    "uFront", "uLat", "PWM_L", "PWM_R", 
    "ErrorLat_dot", "s_lat", "ErrorFront_dot", "s_front"
]

# 로그 파일 이름 설정 (실행 시간으로 파일 이름 자동 생성)
LOG_FILENAME = f"smc_log_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

# ====================================================================
# [2] 초기화 및 연결
# ====================================================================
try:
    # 타임아웃을 설정하여 ser.readline()이 무한정 대기하지 않도록 합니다.
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5) 
    print("Waiting for Arduino to reboot...")
    # 아두이노 리셋 대기
    time.sleep(2) 
    print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud. Starting log to {LOG_FILENAME}")
except serial.SerialException as e:
    print(f"Error connecting to serial port: {e}")
    print("Port not found or already in use. Check port name and ensure the Arduino Serial Monitor is closed.")
    sys.exit(1)


# ====================================================================
# [3] 메인 루프 (데이터 수집 및 로깅 전용)
# ====================================================================
try:
    with open(LOG_FILENAME, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        
        # CSV 파일 헤더 작성
        csv_writer.writerow(VARIABLE_NAMES)
        csvfile.flush()
        
        # 시리얼 버퍼에 남아있는 이전 데이터 비우기
        ser.reset_input_buffer() 
        
        print("Logging started. Press Ctrl+C to stop.")
        
        while True:
            # 데이터가 있을 때만 읽음
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                
                try:
                    # 쉼표로 분리하여 숫자 값으로 변환 시도
                    values = [float(v) for v in line.split(',')]
                    
                    if len(values) == NUM_VARIABLES:
                        
                        # 1. 파일 로깅
                        csv_writer.writerow(values)
                        csvfile.flush() # 즉시 파일에 쓰기

                except ValueError:
                    # 데이터 변환 오류(깨진 데이터)는 무시하고 다음 줄로 이동
                    pass

except KeyboardInterrupt:
    print("\nScript terminated by user (Ctrl+C).")
except Exception as e:
    print(f"\nAn error occurred: {e}")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Serial port closed.")
    print(f"Data successfully logged to: {LOG_FILENAME}")