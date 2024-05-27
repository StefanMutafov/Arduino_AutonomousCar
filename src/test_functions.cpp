//
// Created by Stefan on 13/04/2024.
//

#include "test_functions.h"

void test_servo(Servo& myServo,const int degrees){
    if(degrees <= SERVO_DEG_MAX && degrees >= SERVO_DEG_MIN){
        myServo.write(degrees);
       // delay(10);
    }else if(degrees > SERVO_DEG_MAX){
        myServo.write(SERVO_DEG_MAX);
       // delay(10);
    }else{
        myServo.write(SERVO_DEG_MIN);
     //   delay(10);
    }
    //Serial.print("Current Servo degrees: ");
    //Serial.println(myServo.read());
}

void test_ultraSonic(){
    //Could be done with NewPing library, but will probably get faulty 0 values
    unsigned long duration1;
    unsigned int distance1;
    unsigned long duration2;
    unsigned int distance2;
    digitalWrite(UlTRASONIC1_TRIG_PIN, LOW);
    digitalWrite(UlTRASONIC2_TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(UlTRASONIC1_TRIG_PIN, HIGH);
    digitalWrite(UlTRASONIC2_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(UlTRASONIC1_TRIG_PIN, LOW);
    digitalWrite(UlTRASONIC2_TRIG_PIN, LOW);
    duration1 = pulseIn(ULTRASONIC1_ECHO_PIN, HIGH);
    distance1 = duration1 * 0.034 / 2;
    duration2 = pulseIn(ULTRASONIC2_ECHO_PIN, HIGH);
    distance2 = duration2 * 0.034 / 2;
    Serial.print("Distance 1 in cm: ");
    Serial.println(distance1);
    Serial.print("Distance 2 in cm: ");
    Serial.println(distance2);
}
//void setup_array(QTRSensors& qtr){
//    qtr.setTypeRC();
//    qtr.setSensorPins(sensorPins, SENSOR_COUNT);
//    /*I think the calibration has to be done only once when using the array for the first time, but am not sure*/
//    // For the calibration all the IR sensors need to be moved across the line, so that they can get their max an min values
//
//
//    // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
//    // = ~25 ms per calibrate() call.
//    // Call calibrate() 400 times to make calibration take about 10 seconds.
//    for (uint16_t i = 0; i < 400; i++)
//    {
//        qtr.calibrate();
//    }
//
//    // print the calibration minimum values measured when emitters were on
//
//    for (uint8_t i = 0; i < SENSOR_COUNT; i++)
//    {
//        Serial.print(qtr.calibrationOn.minimum[i]);
//        Serial.print(' ');
//    }
//    Serial.println();
//
//    // print the calibration maximum values measured when emitters were on
//    for (uint8_t i = 0; i < SENSOR_COUNT; i++)
//    {
//        Serial.print(qtr.calibrationOn.maximum[i]);
//        Serial.print(' ');
//    }
//    Serial.println();
//
//}
void test_array(QTRSensors& qtr){
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
}



void test_motor(int level){
// sensor value is in the range 0 to 1023
    // the lower half of it we use for reverse rotation; the upper half for forward
   // rotation
    if (level < 512)
    {
        // reverse rotation
        int reversePWM = -(level - 511) / 2;
        analogWrite(LPWM_Output, 0);
        analogWrite(RPWM_Output, reversePWM);
    }
    else
    {
        // forward rotation
        int forwardPWM = (level - 512) / 2;
        analogWrite(RPWM_Output, 0);
        analogWrite(LPWM_Output, forwardPWM);
    }

}

