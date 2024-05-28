#include "Kinematics.h"

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


void setup() 
{
    Serial.begin(9600);

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

    //simulated required velocities
    //float linear_vel_x = 0;  // 1: 1m/s kedepan, -1: 1m/s kebelakang
    //float linear_vel_y = 0;  // 1: kanan, -1: kiri
    //float angular_vel_z = 0; // 1: roda kanan berputar maju, -1: roda kanan berputer mundur

    // Parse pesan dan ekstrak nilai linear_vel_x, linear_vel_y, dan angular_vel_z
    float linear_vel_x, linear_vel_y, angular_vel_z;
    // Contoh parsing pesan dengan format "linear_x:0.5,linear_y:0,angular_z:0.8"

    //given the required velocities for the robot, you can calculate the rpm required for each motor
    rpm = kinematics.getRPM(linear_vel_x, linear_vel_y, angular_vel_z);
    pwm = kinematics.getPWM(linear_vel_x, linear_vel_y, angular_vel_z);

    if (Serial.available() > 0) {
        // Jika ada pesan yang tersedia di koneksi serial
        String message = Serial.readStringUntil('\n'); // Baca pesan dari serial hingga karakter newline (\n)
        

        scanf(message.c_str(), "linear_x:%f,linear_y:%f,angular_z:%f", &linear_vel_x, &linear_vel_y, &angular_vel_z);

        // Lakukan sesuatu dengan nilai yang diterima, misalnya mengatur kecepatan motor
        // ...
        
        // Kirim balasan jika diperlukan
        Serial.println("Received command"); // Kirim balasan ke Python
    }

    Serial.print(" FRONT LEFT MOTOR: ");
    Serial.print(rpm.motor1);

    Serial.print(" FRONT RIGHT MOTOR: ");
    Serial.print(rpm.motor2);

    Serial.print(" REAR LEFT MOTOR: ");
    Serial.print(rpm.motor3);

    Serial.print(" REAR RIGHT MOTOR: ");
    Serial.println(rpm.motor4);

    /*
    karena mobil kita bagian Front Left dan Rear Left, Front Right dan Left RIght adalah masing2 paralel
    maka linear_y harus 0
    srhingga kecepatan motor depan dan belakang adalah sama untuk masing2 bagian kiri dan kanan
    */
    
    // This is a simulated feedback from each motor. We'll just pass the calculated rpm above for demo's sake.
    // In a live robot, these should be replaced with real RPM values derived from encoder.
    int motor1_feedback = rpm.motor1; //in rpm
    int motor2_feedback = rpm.motor2; //in rpm
    int motor3_feedback = rpm.motor3; //in rpm
    int motor4_feedback = rpm.motor4; //in rpm

    pwm = kinematics.getPWM(linear_vel_x, linear_vel_y, angular_vel_z);
    int speed_A = abs(pwm.motor1);
    int speed_B = abs(pwm.motor2);

    driveMotor(speed_A, rpm.motor1 >= 0, ENA, IN1, IN2);
    driveMotor(speed_B, rpm.motor2 >= 0, ENB, IN3, IN4);
//    delay(5000);


    Kinematics::velocities vel;

    // Now given the RPM from each wheel, you can calculate the linear and angular velocity of the robot.
    // This is useful if you want to create an odometry data (dead reckoning)
    vel = kinematics.getVelocities(motor1_feedback, motor2_feedback, motor3_feedback, motor4_feedback);
    Serial.print(" VEL X: ");
    Serial.print(vel.linear_x, 4);

    Serial.print(" VEL_Y: ");
    Serial.print(vel.linear_y, 4);

    Serial.print(" ANGULAR_Z: ");
    Serial.println(vel.angular_z, 4);
    Serial.println("");
    
}
