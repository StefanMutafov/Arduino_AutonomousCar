#include <Arduino.h>
#include <QTRSensors.h>
#include "test_functions.h"
#include "functions.h"
QTRSensors qtr;
Servo myServo;
void setup() {
    Serial.begin(9600);
    for(unsigned char sensorPin : sensorPins){
        pinMode(sensorPin, INPUT);
    }
    myServo.write(57);
    myServo.attach(SERVO_PIN);
//    pinMode(LPWM_Output, OUTPUT);
//    pinMode(RPWM_Output, OUTPUT);
//    pinMode(UlTRASONIC_TRIG_PIN, OUTPUT);
//    pinMode(ULTRASONIC_ECHO_PIN, INPUT);
    setup_array(qtr);
}




void loop() {
// write your code here
    int position = getPosition(qtr);
    int correction = getTurnDeg(position);

    test_servo(myServo, correction);
   // delay(100);
   // test_array(qtr);
}