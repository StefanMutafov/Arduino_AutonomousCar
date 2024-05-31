#include <Arduino.h>
#include <QTRSensors.h>
#include "test_functions.h"
#include "functions.h"

QTRSensors qtr;
Servo myServo;
Adafruit_MPU6050 mpu;

void setup() {
    Serial.begin(9600);
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
    static double inclination = 0;
    static bool finishDetected = false;
    static int speed = DEFAULT_SPEED;
    int position = getPosition(qtr, finishDetected);
    //int incline = detectHill(mpu);
    double distance = getUSValues();
    double currentSpeed = DEFAULT_SPEED / 23.63;
    inclination += getInclination(mpu);
    Serial.println(inclination);

    if (finishDetected) {
        stopCar(myServo);
        while (true) {
            delay(200000);
        }
    }
    double correction = getTurnDeg(position);
    if(inclination > 11) {
        avoidHill(qtr, myServo, mpu, inclination);
        position = getPosition(qtr, finishDetected);
        correction = getTurnDeg(position);
        inclination += getInclination(mpu);
    }

//        while (inclination > 15) {
//            drive(myServo, DEFAULT_SPEED/2, 0);
//            inclination += getInclination(mpu);
//            Serial.println(inclination);
//
//          }
//    if (obstacleDetected(distance)) {
//        avoidObstacle(myServo, currentSpeed);
//        position = getPosition(qtr, finishDetected);
//        correction = getTurnDeg(position);
//    }

    drive(myServo, speed, correction);

}