import os
import time
import serial
import cv2
import numpy as np
from rplidar import RPLidar

# Replace 'COM3' with the port where your RPLIDAR is connected
PORT_NAME = '/dev/ttyUSB0' 

def run():
    '''Main function'''
    lidar = RPLidar(PORT_NAME)
    lidar.start_motor()

    height, width = 400, 600  # Size of the display window
    img = np.zeros((height, width, 3), dtype=np.uint8)

    try:
        print('Starting RPLidar...')
        while True:
            print("masuk ini")
            for scan in lidar.iter_scans():
                img.fill(0)  # Clear the image for each scan

                for quality, angle, distance in scan:
               
                    # print(quality, angle, distance)
                    # Convert the angle and distance to coordinates
                    (angle)
                    if(angle == 270): #and (angle == 360) and (angle == 90) and (angle == 180):
                        print(distance)

                    angle_rad = np.deg2rad(angle)
                    x = int(distance * 0.1 * np.cos(angle_rad)) + width // 2
                    y = int(distance * 0.1 * np.sin(angle_rad)) + height // 2

                    # Draw the dot
                    cv2.circle(img, (x, y), 1, (255, 255, 255), -1)  # The '-1' argument fills the circle

                # Display the image
                cv2.imshow("RPLIDAR", img)
                if cv2.waitKey(1) == 27:  # Exit on ESC key
                    break

    except KeyboardInterrupt:
        print('Stopping.')

    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    run()
