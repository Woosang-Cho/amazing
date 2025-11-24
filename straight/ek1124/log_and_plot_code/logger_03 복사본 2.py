import serial
import time
import datetime
import csv


SERIAL_PORT = "/dev/tty.usbmodem1101"
BAUD_RATE = 115200


timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
filename = f"robot_log_{timestamp}.csv"


try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
except serial.SerialException as e:
    print(f"Error: Cannot open serial port {SERIAL_PORT}")
    print(f"Details: {e}")
    exit(1)


print(f"Logging to {filename}")
print(f"Serial port: {SERIAL_PORT} at {BAUD_RATE} baud")
print("Press Ctrl+C to stop logging")
print("-" * 50)


with open(filename, mode='w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(["time_sec","raw_dist_cm","lpf_dist_cm","error_cm",
                     "e_dot_lpf_cms","s_value","target_distance_cm","state"])
    
    line_count = 0
    
    try:
        while True:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                
                if not line:
                    continue
                
                if line.startswith("time_sec"):
                    continue
                
                parts = line.split(',')
                if len(parts) != 8:
                    print(f"Warning: Invalid data (expected 8 fields, got {len(parts)})")
                    continue
                
                writer.writerow(parts)
                csvfile.flush()
                
                line_count += 1
                print(line)
            except Exception as e:
                print(f"Error during logging: {e}")
                csvfile.flush()
                # 필요하면 여기서 재연결 로직 등 추가 가능
                
    except KeyboardInterrupt:
        print()
        print("-" * 50)
        print(f"Logging stopped")
        print(f"Total lines logged: {line_count}")
        print(f"Data saved to: {filename}")
    finally:
        ser.close()
