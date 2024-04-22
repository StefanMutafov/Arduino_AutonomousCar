#include <Arduino.h>
#include "../test/test_functions.h"
#include <QTRSensors.h>
QTRSensors qtr;
Servo myServo;
void setup() {
    Serial.begin(9600);
    for(unsigned char sensorPin : sensorPins){
        pinMode(sensorPin, INPUT);
    }
    pinMode(LPWM_Output, OUTPUT);
    pinMode(RPWM_Output, OUTPUT);
    myServo.attach(SERVO_PIN);
    pinMode(UlTRASONIC_TRIG_PIN, OUTPUT);
    pinMode(ULTRASONIC_ECHO_PIN, INPUT);
    setup_array(qtr);



}




void loop() {
// write your code here

}