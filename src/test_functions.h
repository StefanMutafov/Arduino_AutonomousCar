//
// Created by Stefan on 13/04/2024.
//

#ifndef AUTONOMOUSCAR_TEST_FUNCTIONS_H
#define AUTONOMOUSCAR_TEST_FUNCTIONS_H
#include <Arduino.h>
#include "Servo.h"
#include "QTRSensors.h"
#include "config.h"
///Tests the servo by rotating it to certain degrees
///Could be used with potentiometer
///@param myServo Reference to the Servo object
///@param degrees Rotation in degrees max - SERVO_DEG_MAX, min - SERVO_DEG_MIN
void test_servo(Servo& myServo,int degrees);

///Tests the ultrasonic sensor by printing the sensed distance
void test_ultraSonic();

/// Tests IR array by printing the readings from the sensors and the position of the line

///@param qtr Reference to the array object
void test_array(QTRSensors& qtr);
void setup_array(QTRSensors& qtr);

///Tests the motor and motor shield by setting the motor speed
///Could be used with potentiometer
///@param level the speed of the motor(>512 forward, <512 reverce)
void test_motor(int level);
//Motor
//
#endif //AUTONOMOUSCAR_TEST_FUNCTIONS_H
