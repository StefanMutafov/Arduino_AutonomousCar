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

//bool detectHill(double distance1, double distance2);
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

///Get the values from the US_Sensor
///@return the reading from the sensor
double getUSValues();

///Get the position from the  IR_Sensor Array
///@param qtr Reference to the IR_Sensor Array
///@param detectFinish Reference to a boolean checking if the end of the track has been reached
///@return Value of the position
int getPosition(QTRSensors& qtr, bool& detectFinish);


///Detecting an obstacle
///@param distance1 The distance from the US
///@return TRUE if detected FALSE otherwise
bool obstacleDetected(double distance);


///Avoid an obstacle
///@param myServo  The servo responsible for turning
///@param Reference to the current speed
void avoidObstacle(Servo& myServo, double currentSpeed,  Adafruit_MPU6050& mpu, double& inclination);

///Detect a hill
///@param mpu Reference to the gyroscope
int detectHill(Adafruit_MPU6050& mpu);

double getInclination(Adafruit_MPU6050& mpu);

///Avoid the obstacle
///@param qtr Reference to the IR-Array
///@param myServo Reference to the servo
void avoidHill(QTRSensors& qtr, Servo& myServo, Adafruit_MPU6050& mpu, double& inclination);

///Stop the car
///@param myServo Reference to the servo responsible for turning
void stopCar(Servo& myServo);

#endif //AUTONOMOUSCAR_FUNCTIONS_H
