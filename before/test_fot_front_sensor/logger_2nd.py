# robot_logger.py
import serial
import time
import datetime
import csv

# 시리얼 포트 설정 
SERIAL_PORT = "COM3" 
BAUD_RATE = 115200

# CSV 파일 이름 생성 (자동 날짜/시간)
timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
filename = f"robot_log_{timestamp}.csv"

# 시리얼 연결
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)  # 시리얼 안정화

print(f"Logging to {filename}")
with open(filename, mode='w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    # 헤더 작성
    writer.writerow(["time_sec","raw_dist_cm","lpf_dist_cm","error_cm","e_dot_lpf_cms","s_value","target_distance_cm","state"])
    
    try:
        while True:
            line = ser.readline().decode('utf-8').strip()
            if line:
                # 아두이노가 보내는 CSV 그대로 저장
                writer.writerow(line.split(','))
                print(line)
    except KeyboardInterrupt:
        print("Logging stopped by user.")
    finally:
        ser.close()
