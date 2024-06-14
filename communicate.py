import serial
import time
import json
import cv2
import numpy as np
from rplidar import RPLidar

# Inisialisasi koneksi serial
ser = serial.Serial('COM3', 115200)  # Ganti '/dev/ttyUSB0' dengan port serial yang sesuai
lidar = RPLidar('COM5')
lidar.start_motor()
height, width = 400, 600  # Size of the display window
img = np.zeros((height, width, 3), dtype=np.uint8)

def send_command(linear_vel_x, linear_vel_y, angular_vel_z):
    # Kirim pesan ke ESP32 dalam format yang diharapkan
    command = f"linear_x:{linear_vel_x},linear_y:{linear_vel_y},angular_z:{angular_vel_z}\n"
    ser.write(command.encode())

def receive_response():
    # Baca balasan dari ESP32
    response = ser.readline().decode().strip()
    try:
        response_json = json.loads(response)
        print("Response:", response_json)
    except json.JSONDecodeError:
        print("Failed to decode JSON response:", response)


def all():
    try:
        for scan in lidar.iter_scans():
            for quality, angle, distance in scan:
                if 85 <= angle <= 95 and distance > 160:
                    send_command(0.2, 0, 0)
                    time.sleep(0.01)
                    receive_response()
                elif 85 <= angle <= 95 and distance < 160:
                    send_command(0, 0, 0)
                    time.sleep(0.01)
                    receive_response()

    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        lidar.stop()
        lidar.disconnect()
        ser.close()

if __name__ == '__main__':
    all()
