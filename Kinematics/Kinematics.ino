#include <Arduino.h>
#include "Kinematics.h"

// Variables to store the received commands
float linear_vel_x = 0;
float linear_vel_y = 0;
float angular_vel_z = 0;
void parseCommand(String command);

#define MOTOR_MAX_RPM 90        // motor's maximum rpm
#define WHEEL_DIAMETER 0.06      // robot's wheel diameter expressed in meters
#define FR_WHEEL_DISTANCE 0.135   // distance between front wheel and rear wheel
#define LR_WHEEL_DISTANCE 0.115   // distance between left wheel and right wheel
#define PWM_BITS 8              // microcontroller's PWM pin resolution. Arduino Uno/Mega Teensy is using 8 bits(0-255)

Kinematics kinematics(MOTOR_MAX_RPM, WHEEL_DIAMETER, FR_WHEEL_DISTANCE, LR_WHEEL_DISTANCE, PWM_BITS);

// motor 1 settings
#define IN1 17
#define IN2 16
#define ENA 4 // this pin must be PWM enabled pin

// motor 2 settings
#define IN3 19
#define IN4 18
#define ENB 5 // this pin must be PWM enabled pin

void driveMotor(int speed, bool direction, int enPin, int inPin1, int inPin2) {
    digitalWrite(inPin1, direction);
    digitalWrite(inPin2, !direction);
    analogWrite(enPin, speed);
}

void stopMotor(int enPin, int inPin1, int inPin2) {
    digitalWrite(inPin1, LOW);
    digitalWrite(inPin2, LOW);
    analogWrite(enPin, 0);
    Serial.println("Stop motor");
}

void setup()
{
    // Initialize serial communication at 9600 baud rate
    Serial.begin(115200);

    // Setup motor a  (left or right side?)
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    
    // Setup motor B
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
}

void loop()
{
    Kinematics::output rpm;
    Kinematics::output pwm;

    rpm = kinematics.getRPM(linear_vel_x, linear_vel_y, angular_vel_z);


    // Check if data is available on the serial port
    if (Serial.available() > 0)
    {
    // Read the incoming serial data
    String command = Serial.readStringUntil('\n');
    parseCommand(command);

    // Respond back with the received command in JSON format
    Serial.print("{\"linear_x\":");
    Serial.print(linear_vel_x);
    Serial.print(",\"linear_y\":");
    Serial.print(linear_vel_y);
    Serial.print(",\"angular_z\":");
    Serial.print(angular_vel_z);
    Serial.println("}");

    pwm = kinematics.getPWM(linear_vel_x, linear_vel_y, angular_vel_z);
    int speed_A = abs(pwm.motor1);
    int speed_B = abs(pwm.motor2);

    driveMotor(speed_A, rpm.motor1 >= 0, ENA, IN1, IN2);
    driveMotor(speed_B, rpm.motor2 >= 0, ENB, IN3, IN4);


    if (speed_A == 0) {
        stopMotor(ENA, IN1, IN2);
    } else {
        driveMotor(speed_A, rpm.motor1 >= 0, ENA, IN1, IN2);
    }

    if (speed_B == 0) {
        stopMotor(ENB, IN3, IN4);
    } else {
        driveMotor(speed_B, rpm.motor2 >= 0, ENB, IN3, IN4);

    Kinematics::velocities vel;
    }
  }
}

void parseCommand(String command)
{
  // Split the command string based on the delimiter ','
  int index1 = command.indexOf(',');
  int index2 = command.indexOf(',', index1 + 1);

  // Extract and convert the values from the command string
  String linear_x_str = command.substring(command.indexOf(':') + 1, index1);
  String linear_y_str = command.substring(command.indexOf(':', index1) + 1, index2);
  String angular_z_str = command.substring(command.indexOf(':', index2) + 1);

  // Convert strings to float
  linear_vel_x = linear_x_str.toFloat();
  linear_vel_y = linear_y_str.toFloat();
  angular_vel_z = angular_z_str.toFloat();
}
