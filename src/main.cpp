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
    static bool finishDetected = false;
    static int speed = DEFAULT_SPEED;
    int position = getPosition(qtr, finishDetected);
    int incline = detectHill(mpu);
    double distance = getUSValues();
    double currentSpeed = 1.1;



    if (finishDetected) {
        stopCar(myServo);
        while (true) {
            delay(200000);
        }
        //TODO:Make some kind of a restart mechanism, like a button or smth;
    }
    double correction = getTurnDeg(position);
    if (obstacleDetected(distance)) {
        avoidObstacle(myServo, currentSpeed);
        position = getPosition(qtr, finishDetected);
        correction = getTurnDeg(position);
    }
    if(incline == 1) {
        avoidHill(qtr, myServo);
    }

    drive(myServo, speed, correction);

}