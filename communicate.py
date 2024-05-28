import serial
import time

# Inisialisasi koneksi serial
ser = serial.Serial('/dev/ttyUSB0', 9600)  # Ganti '/dev/ttyUSB0' dengan port serial yang sesuai

def send_command(linear_vel_x, linear_vel_y, angular_vel_z):
    # Kirim pesan ke ESP32 dalam format yang diharapkan
    command = f"linear_x:{linear_vel_x},linear_y:{linear_vel_y},angular_z:{angular_vel_z}\n"
    ser.write(command.encode())

def receive_response():
    # Baca balasan dari ESP32
    response = ser.readline().decode().strip()
    print("Response:", response)

# Contoh penggunaan
send_command(0.5, 0, 0.8)  # Kirim perintah ke ESP32
time.sleep(1)  # Tunggu sebentar
receive_response()  # Baca dan cetak balasan dari ESP32
