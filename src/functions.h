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
///Setup function for gyroscope
///@param mpu Reference to the gyroscope
void setupMPU(Adafruit_MPU6050& mpu);

///Manual setup of IR_Sensor Array
///@param qtr Reference to the IR_Sensor Array
void setupArray_manual(QTRSensors& qtr);

///Setup IR_Sensor Array
///@param qtr reference to the IR_Sensor Array
void setup_array(QTRSensors& qtr);


///Get the value of the correction of the turn angle
///@param position the current position of the IR_Sensor Array
///@return The value of the correction of the turn angle
double getTurnDeg(const int position);

///Set the drive mode of the car
///@param myServo The servo responsible for turning
///@param speed The required speed
///@param turnAngle The correction to the turn angle
void drive(Servo& myServo, int speed, int turnAngle);

///Get the values from the US_Sensors
///@param distance1 A reference to the variable recording the distance of the first sensor
///@param distance1 A reference to the variable recording the distance of the second sensor
void getUSValues(double& distance1, double& distance2);

///Get the position from the  IR_Sensor Array
///@param qtr Reference to the IR_Sensor Array
///@param detectFinish Reference to a boolean checking if the end of the track has been reached
///@return Value of the position
int getPosition(QTRSensors& qtr, bool& detectFinish);


///Detecting an obstacle
///@param distance1 The distance from the first US
///@param distance1 The distance from the second US
///@return TRUE if detected FALSE otherwise
bool obstacleDetected(double distance1, double distance2);

///Calculate current speed based on US position
///@param distance Distance from one of the US
double getCurrentSpeed(double distance);
//bool obstacleDetected(int US_trig, int US_echo);

///Avoid an obstacle
///@param myServo  The servo responsible for turning
///@param Reference to the current speed
void avoidObstacle(Servo& myServo, int& speed);

///Detect a hill
///@param mpu Reference to the gyroscope
int detectHill(Adafruit_MPU6050& mpu);

///Stop the car
///@param myServo Reference to the servo responsible for turning
void stopCar(Servo& myServo);

#endif //AUTONOMOUSCAR_FUNCTIONS_H
