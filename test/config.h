//
// Created by Stefan on 13/04/2024.
//

#ifndef AUTONOMOUSCAR_CONFIG_H
#define AUTONOMOUSCAR_CONFIG_H
#define SERVO_DEG_MAX  0
# define SENSOR_COUNT  8
const uint8_t sensorPins[SENSOR_COUNT]{0,0,0,0,0,0,0,0};//Digital
#define RPWM_Output 0 // Arduino PWM output pin
#define LPWM_Output  0 // Arduino PWM output pin
#define SERVO_PIN  0//Analog
#define UlTRASONIC_TRIG_PIN 0//Digital
#define ULTRASONIC_ECHO_PIN 0//Digital


#endif //AUTONOMOUSCAR_CONFIG_H
