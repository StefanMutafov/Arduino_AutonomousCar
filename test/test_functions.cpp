//
// Created by Stefan on 13/04/2024.
//

#include "test_functions.h"

void test_servo(Servo& myServo,const int degrees){
    if(abs(degrees) <= SERVO_DEG_MAX){
        myServo.write(degrees);
        delay(10);
    }else if(degrees > 0){
        myServo.write(SERVO_DEG_MAX);
        delay(10);
    }else{
        myServo.write(-SERVO_DEG_MAX);
        delay(10);
    }
    Serial.print("Current Servo degrees: ");
    Serial.println(myServo.read());
}

void test_ultraSonic(){
    //Could be done with NewPing library, but will probably get faulty 0 values
    unsigned long duration;
    unsigned int distance;
    digitalWrite(UlTRASONIC_TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(UlTRASONIC_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(UlTRASONIC_TRIG_PIN, LOW);
    duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH);
    distance = duration * 0.034 / 2;
    Serial.print("Distance in cm: ");
    Serial.println(distance);
}

void test_array(QTRSensors& qtr){
    uint16_t array[SENSOR_COUNT];
    uint16_t position = qtr.readLineWhite(array);
    for (uint8_t i = 0; i < SENSOR_COUNT; i++)
    {
        Serial.print("Sensor");
        Serial.print(i);
        Serial.print(":");
        Serial.print(array[i]);
        Serial.print('\t');
    }
    Serial.print("Position: ");
    Serial.println(position);
}

