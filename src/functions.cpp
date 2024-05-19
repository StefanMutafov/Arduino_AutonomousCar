//
// Created by Stefan on 19/05/2024.
//

#include "functions.h"

int getTurnDeg(const int position) {
    const double Kp = 1.0;
    const double Ki = 0.0;
    const double Kd = 0.0;
    const int midPosition = 3500;

    static double previous_error = 0;
    static double integral = 0;
    double derivative = 0;
    double correction = 0;
    int error = 0;
    error = position - midPosition;

    integral += error;
    derivative = error - previous_error;
    correction = Kp * error + Ki * integral + Kd * derivative;
    //TODO:When using integral change the KP*3500 to the maximum possible correction value
    correction =(SERVO_DEG_MIN + (SERVO_DEG_MAX-SERVO_DEG_MIN)/2) + ((SERVO_DEG_MAX-SERVO_DEG_MIN)/(Kp*midPosition*2))*correction;
    previous_error = error;
    Serial.print("Correction: ");
    Serial.println(correction);
    return correction;
}

int getPosition(QTRSensors& qtr){
    uint16_t array[SENSOR_COUNT];
    uint16_t position = qtr.readLineWhite(array);
    for (uint8_t i = 0; i < SENSOR_COUNT; i++)
    {
        Serial.print("Sensor ");
        Serial.print(i);
        Serial.print(":");
        Serial.print(array[i]);
        Serial.print('\t');
    }
    Serial.print("Position: ");
    Serial.println(position);
    return position;
}