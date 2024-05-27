//
// Created by Stefan on 19/05/2024.
//

#include "functions.h"
void setup_array(QTRSensors& qtr){
    qtr.setTypeRC();
    qtr.setSensorPins(sensorPins, SENSOR_COUNT);

    // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
    // = ~25 ms per calibrate() call.
    // Call calibrate() 400 times to make calibration take about 10 seconds.
    for (uint16_t i = 0; i < 400; i++)
    {
        qtr.calibrate();
    }
    Serial.begin(9600);
    for (uint8_t i = 0; i < SENSOR_COUNT; i++)
    {
        Serial.print(qtr.calibrationOn.minimum[i]);
        Serial.print(' ');
    }
    Serial.println();
    for (uint8_t i = 0; i < SENSOR_COUNT; i++)
    {
        Serial.print(qtr.calibrationOn.maximum[i]);
        Serial.print(' ');
    }
    Serial.println();

}

void setupArray_manual(QTRSensors& qtr){
    qtr.setTypeRC();
    qtr.setSensorPins(sensorPins, SENSOR_COUNT);
    for (uint16_t i = 0; i < 10; i++)
    {
        qtr.calibrate();
    }
    qtr.calibrationOn.minimum[0] = 1508;
    qtr.calibrationOn.minimum[1] = 1184;
    qtr.calibrationOn.minimum[2] = 1170;
    qtr.calibrationOn.minimum[3] = 1086;
    qtr.calibrationOn.minimum[4] = 1057;
    qtr.calibrationOn.minimum[5] = 990;
    qtr.calibrationOn.minimum[6] = 1073;
    qtr.calibrationOn.minimum[7] = 1122;


    for (uint8_t i = 0; i < SENSOR_COUNT; i++)
    {
        qtr.calibrationOn.maximum[i] = 2500;
        //Serial.print(qtr.calibrationOn.maximum[i]);
        //Serial.print(' ');
    }
//    Serial.println();
//
//    for (uint8_t i = 0; i < SENSOR_COUNT; i++)
//    {
//        Serial.print(qtr.calibrationOn.minimum[i]);
//        Serial.print(' ');
//    }
//    Serial.println();

}
void setupMPU(Adafruit_MPU6050& mpu){
    mpu.begin();
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}
void drive(Servo& myServo,int speed, int turnAngle){
    if(speed > 255) speed = 255;
    if(speed < -255) speed = -255;
    if(speed >=0){
        analogWrite(LPWM_Output, speed);
        analogWrite(RPWM_Output, 0);
    }else{
        analogWrite(LPWM_Output, 0);
        analogWrite(RPWM_Output, -1*speed);
    }
    if(SERVO_MID+turnAngle <= SERVO_DEG_MAX && SERVO_MID+turnAngle >= SERVO_DEG_MIN){
        myServo.write(SERVO_MID+turnAngle);
    }else if(SERVO_MID+turnAngle > SERVO_DEG_MAX){
        myServo.write(SERVO_DEG_MAX);
    }else{
        myServo.write(SERVO_DEG_MIN);
    }
}

double getTurnDeg(const int position) {
    const double Kp = 0.00399;
   // const double Kp = 0.4;
    const double Ki = 0.00;
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
   // double divisions = Kp*midPosition*2 + Ki*integral + Kd*(midPosition*2-previous_error);
    //correction = map(correction, 0, Kp*midPosition* 2, 35, 90);
  //  correction = ((SERVO_DEG_MAX-SERVO_DEG_MIN)/divisions)*correction;
    previous_error = error;
   // Serial.print("Correction: ");
    //Serial.println(correction);
    return correction;
}

int getPosition(QTRSensors& qtr, bool& atFinish){
    uint16_t array[SENSOR_COUNT];
    uint16_t position = qtr.readLineWhite(array);
    if(array[0] == 0 && array[7] == 0){
        atFinish = true;
    }
//    for (uint8_t i = 0; i < SENSOR_COUNT; i++)
//    {
//        Serial.print("Sensor ");
//        Serial.print(i);
//        Serial.print(":");
//        Serial.print(array[i]);
//        Serial.print('\t');
//    }
//    Serial.print("Position: ");
//    Serial.println(position);
    return position;
}

void getUSValues(double& distance1, double& distance2){
    unsigned long duration1;
    unsigned long duration2;
    digitalWrite(UlTRASONIC1_TRIG_PIN, LOW);
    digitalWrite(UlTRASONIC2_TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(UlTRASONIC1_TRIG_PIN, HIGH);
    digitalWrite(UlTRASONIC2_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(UlTRASONIC1_TRIG_PIN, LOW);
    digitalWrite(UlTRASONIC2_TRIG_PIN, LOW);
    duration1 = pulseIn(ULTRASONIC1_ECHO_PIN, HIGH);
    duration2 = pulseIn(ULTRASONIC2_ECHO_PIN, HIGH);
    distance1 = duration1 * 0.034 / 2;
    distance2 = duration2 * 0.034 / 2;
}

bool obstacleDetected(double distance1, double distance2){
    if(distance2 <= MAX_DETECT_DISTANCE && distance1-distance2 < US_DETECTION_DIFFERENCE)
    {
        return true;
    }
    return false;
}

//bool obstacleDetected(int US_trig, int US_echo){
//    //Could be done with NewPing library, but will probably get faulty 0 values
//    unsigned long duration;
//    unsigned int distance;
//    digitalWrite(US_trig, LOW);
//    delayMicroseconds(2);
//    digitalWrite(US_trig, HIGH);
//    delayMicroseconds(10);
//    digitalWrite(US_trig, LOW);
//    duration = pulseIn(US_echo, HIGH);
//    //distance is measured from the board
//    distance = duration * 0.034 / 2;
//
//    if (distance <= MAX_DETECT_DISTANCE)
//    {
//        // Serial.print("TRUE Distance of US 1 in cm: ");
//        // Serial.println(distance);
//        return true;
//    }
//
//    // Serial.print("FALSE");
//    return false;
//    //Serial.print("Distance of US 1 in cm: ");
//    //Serial.println(distance);
//}
void avoidObstacle(Servo& myServo, int& speed){

    drive(myServo, speed, SERVO_DEG_MAX);
    delay(1000);
//    drive(myServo, speed, SERVO_DEG_MAX);
//    delay(3000);
}

//bool detectHill(double distance1, double distance2){
//    if(distance2 <= MAX_DETECT_DISTANCE && distance2-distance1 > US_DETECTION_DIFFERENCE)
//    {
//        return true;
//    }
//    return false;
//}

int detectHill(Adafruit_MPU6050& mpu){
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    //this starts detecting the up properly but detects the down too quickly (we can solve it by delaying the change of speed from the time it detects the down or up)
    if(abs(g.gyro.y) > 0.5)
    {
        if (g.gyro.y > 0) return 1;
        else return -1;
    }

    delay(500);

    return 0;
}
double getCurrentSpeed(double currentDistance){
    static unsigned long current_time = 0;
    static double lastDistance = 0;
    if (currentDistance <= 80 && currentDistance > 60)
    {
        lastDistance = currentDistance;
        current_time = millis();
        return 0;
    }
    if (currentDistance <= 60 && currentDistance > 0)
    {
        lastDistance = lastDistance - currentDistance;
        current_time = millis() - current_time;
    }

    if (current_time > 0) return (lastDistance/100)/(current_time/1000);

    return 0;


}

void stopCar(Servo& myServo){
    drive(myServo, -20, 0);
    delay(10);
    drive(myServo, 0, 0);
}