import serial
import time
import re
import csv
from datetime import datetime

# ==========================================
# 설정
# ==========================================
PORT = 'dev/cu.usbmodem01'  # Windows: COM3, Mac: /dev/cu.usbmodem*, Linux: /dev/ttyUSB0
BAUD = 115200
DURATION_SEC = 60  # 로깅 시간 (초)

# CSV 파일명 (타임스탬프 포함)
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
CSV_FILENAME = f"robot_data_{timestamp}.csv"

# ==========================================
# CSV 헤더
# ==========================================
HEADER = [
    'time_sec',
    'left_dist', 'right_dist', 'front_dist',
    'error_lat', 'u_front', 'u_lat',
    'pwm_left', 'pwm_right',
    'e_lat_dot', 's_lat',
    'e_front_dot', 's_front'
]

# ==========================================
# 메인 로깅 함수
# ==========================================
def log_to_csv():
    """시리얼 데이터를 실시간으로 CSV에 저장"""
    
    try:
        # 시리얼 포트 연결
        ser = serial.Serial(PORT, BAUD, timeout=1)
        time.sleep(2)
        
        print("=" * 60)
        print(f"Connected to {PORT} at {BAUD} baud")
        print(f"Logging to: {CSV_FILENAME}")
        print(f"Duration: {DURATION_SEC} seconds")
        print("=" * 60)
        print()
        
        # CSV 파일 생성
        with open(CSV_FILENAME, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(HEADER)
            
            start_time = time.time()
            sample_count = 0
            
            while (time.time() - start_time) < DURATION_SEC:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    # 데이터 라인 파싱 (< > 패턴만)
                    match = re.search(r'<([\d.,-]+)>', line)
                    if match:
                        try:
                            values = match.group(1).split(',')
                            if len(values) == 12:
                                # 현재 시간 계산
                                current_time = time.time() - start_time
                                
                                # CSV 행 작성
                                row = [current_time] + values
                                writer.writerow(row)
                                
                                sample_count += 1
                                
                                # 진행 상황 출력 (10개마다)
                                if sample_count % 10 == 0:
                                    print(f"[{current_time:.2f}s] Sample {sample_count}: "
                                          f"L={values[0]}, R={values[1]}, F={values[2]}")
                        except Exception as e:
                            print(f"Parse error: {e}")
                    
                    # 타이밍 정보는 화면에만 출력
                    elif any(keyword in line for keyword in 
                            ["TIMING", "ms", "===", "├", "└", "WARNING", "OK"]):
                        print(line)
            
            elapsed = time.time() - start_time
            
        ser.close()
        
        print()
        print("=" * 60)
        print(f"   Logging completed!")
        print(f"   Total samples: {sample_count}")
        print(f"   Elapsed time: {elapsed:.2f} seconds")
        print(f"   Sample rate: {sample_count/elapsed:.2f} Hz")
        print(f"   Saved to: {CSV_FILENAME}")
        print("=" * 60)
        
    except serial.SerialException as e:
        print(f"Serial Error: {e}")
        print("   Check your port name and connection.")
    except KeyboardInterrupt:
        print("\n⚠️  Logging interrupted by user.")
        ser.close()
    except Exception as e:
        print(f"Error: {e}")

# ==========================================
# 실행
# ==========================================
if __name__ == "__main__":
    print("Arduino Data Logger")
    print("Press Ctrl+C to stop early\n")
    log_to_csv()
