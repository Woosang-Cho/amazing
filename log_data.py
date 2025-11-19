import serial
import datetime
import csv
import os
import sys

# ====================================================================
# [1] 설정 파라미터 변경
# ====================================================================
SERIAL_PORT = 'COM3'  # <<< 현재 아두이노가 연결된 포트로 반드시 변경하세요.
BAUD_RATE = 115200
NUM_VARIABLES = 8 

# CSV 파일 헤더 (아두이노 출력 순서와 동일)
VARIABLE_NAMES = ["L_Dist", "R_Dist", "F_Dist", "uFront", "uLat", "PWM_L", "PWM_R", "ErrorLat"]

# 로그 파일 이름 설정 (실행 시간으로 파일 이름 자동 생성)
LOG_FILENAME = f"smc_log_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

# ====================================================================
# [2] 초기화 및 연결
# ====================================================================
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    # 아두이노가 재시작되도록 잠깐 대기 (우노 연결 시 필수)
    print("Waiting for Arduino to reboot...")
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
        
        while True:
            # 1. 데이터 읽기 (줄바꿈 문자가 올 때까지 대기)
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                
                try:
                    # 2. 쉼표로 분리하여 리스트로 변환 (float 변환 시도)
                    values = [float(v) for v in line.split(',')]
                    
                    if len(values) == NUM_VARIABLES:
                        
                        # 3. 파일 로깅: CSV 파일에 기록
                        csv_writer.writerow(values)
                        csvfile.flush() # 데이터 손실 방지를 위해 즉시 파일에 쓰기

                except ValueError:
                    # 불완전하거나 비정상적인 데이터 무시
                    # print(f"Skipping bad data: {line}")
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