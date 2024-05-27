//
// Created by Stefan on 13/04/2024.
//

#ifndef AUTONOMOUSCAR_CONFIG_H
#define AUTONOMOUSCAR_CONFIG_H
#define SERVO_DEG_MAX  90
#define SERVO_DEG_MIN  35
#define SERVO_MID 63
#define MAX_DETECT_DISTANCE 65
#define US_DETECTION_DIFFERENCE 1000
#define HILL_ACCELERATION 1.25
#define DEFAULT_SPEED 25
#define SENSOR_COUNT  8
const uint8_t sensorPins[SENSOR_COUNT]{12,11,10,9,8,7,4,2};//Digital
#define RPWM_Output 5 // Arduino PWM output pin
#define LPWM_Output  6 // Arduino PWM output pin
#define SERVO_PIN  3//Analog
#define UlTRASONIC1_TRIG_PIN A0//Digital
#define ULTRASONIC1_ECHO_PIN A1//Digital
#define UlTRASONIC2_TRIG_PIN A2//Digital
#define ULTRASONIC2_ECHO_PIN A3//Digital



#endif //AUTONOMOUSCAR_CONFIG_H
