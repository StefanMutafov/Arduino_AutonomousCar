#include <Arduino.h>
#include <QTRSensors.h>
#include "test_functions.h"
#include "functions.h"
QTRSensors qtr;
Servo myServo;
Adafruit_MPU6050 mpu;
void setup() {
    //Serial.begin(9600);
    for(unsigned char sensorPin : sensorPins){
        pinMode(sensorPin, INPUT);
    }
    myServo.write(SERVO_MID);
    myServo.attach(SERVO_PIN);
    pinMode(LPWM_Output, OUTPUT);
    pinMode(RPWM_Output, OUTPUT);
    pinMode(UlTRASONIC1_TRIG_PIN, OUTPUT);
    pinMode(ULTRASONIC1_ECHO_PIN, INPUT);

//setupArray_manual(qtr);
    setup_array(qtr);
    //setupMPU(mpu);
}




void loop() {
// write your code here
static bool finishDetected = false;
    static int speed = DEFAULT_SPEED;
    int position = getPosition(qtr, finishDetected);
    double US1 = 0, US2 = 0;
    getUSValues(US1, US2);
    if(finishDetected){
        stopCar(myServo);
        delay(200000);
        //TODO:Make some kind of a restart mechanism, like a button or smth;
    }
    double correction = getTurnDeg(position);
//    if(detectHill(mpu) == 1){
//        speed = DEFAULT_SPEED * HILL_ACCELERATION;
//    }else{
//        speed = DEFAULT_SPEED;
//    }
    if(obstacleDetected(US1, US2)){
        avoidObstacle(myServo, speed);
    }
    drive(myServo, DEFAULT_SPEED, correction);

//    test_servo(myServo, correction);
//    analogWrite(LPWM_Output, 35.2);
//    analogWrite(RPWM_Output, 0);

   // delay(100);
   // test_array(qtr);
}