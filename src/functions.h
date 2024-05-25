//
// Created by Stefan on 19/05/2024.
//

#ifndef AUTONOMOUSCAR_FUNCTIONS_H
#define AUTONOMOUSCAR_FUNCTIONS_H
#include <Arduino.h>
#include "config.h"
#include "Servo.h"
#include "QTRSensors.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
int getTurnDeg(const int position);
void drive(Servo& myServo, int speed, int turnAngle);
int getPosition(QTRSensors& qtr, bool& detectFinish);
void setupMPU(Adafruit_MPU6050& mpu);
void setupArray_manual(QTRSensors& qtr);
bool obstacleDetected(int US_trig, int US_echo);
void avoidObstacle(Servo& myServo, int& speed);
int detectHill(Adafruit_MPU6050& mpu);
void stopCar(Servo& myServo);

//void setup_array(QTRSensors& qtr);

#endif //AUTONOMOUSCAR_FUNCTIONS_H
