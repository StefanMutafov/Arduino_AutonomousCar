#include <Arduino.h>
#include <QTRSensors.h>
#include "test_functions.h"
#include "functions.h"

QTRSensors qtr;
Servo myServo;
Adafruit_MPU6050 mpu;

void setup() {
    Serial.begin(9600);
    // setupMPU(mpu);
    for (unsigned char sensorPin: sensorPins) {
        pinMode(sensorPin, INPUT);
    }
    myServo.write(SERVO_MID);
    myServo.attach(SERVO_PIN);
    pinMode(LPWM_Output, OUTPUT);
    pinMode(RPWM_Output, OUTPUT);
    pinMode(UlTRASONIC_TRIG_PIN, OUTPUT);
    pinMode(ULTRASONIC_ECHO_PIN, INPUT);

//setupArray_manual(qtr);
    setup_array(qtr);
    setupMPU(mpu);
}


void loop() {
// write your code here
    static bool finishDetected = false;
    static int speed = DEFAULT_SPEED;
    int position = getPosition(qtr, finishDetected);
   // int incline = detectHill(mpu);
  //  double distance = getUSValues();
    //double currentSpeed = getCurrentSpeed(distance);

    if (finishDetected) {
        stopCar(myServo);
        delay(200000);
        //TODO:Make some kind of a restart mechanism, like a button or smth;
    }
    double correction = getTurnDeg(position);
//    if (obstacleDetected(distance)) {
//        avoidObstacle(myServo, currentSpeed, distance);
//    }
//    if(incline == 1) {
//        speed = DEFAULT_SPEED * HILL_ACCELERATION;
//        while(incline != -1){
//            incline = detectHill(mpu);
//            drive(myServo, speed, 0);
//        }
//        position = getPosition(qtr, finishDetected);
//        speed = DEFAULT_SPEED-15;
//        drive(myServo, speed, 0);
//        delay(950);
//        position = getPosition(qtr, finishDetected);
//        speed = DEFAULT_SPEED;
//    }

    drive(myServo, speed, correction);

}