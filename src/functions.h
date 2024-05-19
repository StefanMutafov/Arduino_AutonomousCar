//
// Created by Stefan on 19/05/2024.
//

#ifndef AUTONOMOUSCAR_FUNCTIONS_H
#define AUTONOMOUSCAR_FUNCTIONS_H
#include <Arduino.h>
#include "config.h"
#include "Servo.h"
#include "QTRSensors.h"

int getTurnDeg(const int position);
int getPosition(QTRSensors& qtr);

#endif //AUTONOMOUSCAR_FUNCTIONS_H
