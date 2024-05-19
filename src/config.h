//
// Created by Stefan on 13/04/2024.
//

#ifndef AUTONOMOUSCAR_CONFIG_H
#define AUTONOMOUSCAR_CONFIG_H
#define SERVO_DEG_MAX  80
#define SERVO_DEG_MIN  40
# define SENSOR_COUNT  8
const uint8_t sensorPins[SENSOR_COUNT]{12,11,10,9,8,7,4,2};//Digital
#define RPWM_Output 5 // Arduino PWM output pin
#define LPWM_Output  6 // Arduino PWM output pin
#define SERVO_PIN  3//Analog
#define UlTRASONIC_TRIG_PIN 0//Digital
#define ULTRASONIC_ECHO_PIN 0//Digital



#endif //AUTONOMOUSCAR_CONFIG_H
