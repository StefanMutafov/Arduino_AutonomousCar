#include "functions.h"

void setup_array(QTRSensors &qtr) {
    qtr.setTypeRC();
    qtr.setSensorPins(sensorPins, SENSOR_COUNT);

    // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
    // = ~25 ms per calibrate() call.
    // Call calibrate() 400 times to make calibration take about 10 seconds.
    for (uint16_t i = 0; i < 400; i++) {
        qtr.calibrate();
    }
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        Serial.print(qtr.calibrationOn.minimum[i]);
        Serial.print(' ');
    }
    Serial.println();
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        Serial.print(qtr.calibrationOn.maximum[i]);
        Serial.print(' ');
    }
    Serial.println();

}

void setupArray_manual(QTRSensors &qtr) {
    qtr.setTypeRC();
    qtr.setSensorPins(sensorPins, SENSOR_COUNT);
    for (uint16_t i = 0; i < 10; i++) {
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


    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        qtr.calibrationOn.maximum[i] = 2500;

    }

}

void setupMPU(Adafruit_MPU6050 &mpu) {
    mpu.begin();
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void drive(Servo &myServo, int speed, int turnAngle) {
    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;


    if (SERVO_MID + turnAngle <= SERVO_DEG_MAX && SERVO_MID + turnAngle >= SERVO_DEG_MIN) {
        myServo.write(SERVO_MID + turnAngle);
    } else if (SERVO_MID + turnAngle > SERVO_DEG_MAX) {
        myServo.write(SERVO_DEG_MAX);
    } else {
        myServo.write(SERVO_DEG_MIN);
    }

    if (speed >= 0) {
        analogWrite(LPWM_Output, speed);
        analogWrite(RPWM_Output, 0);
    } else {
        analogWrite(LPWM_Output, 0);
        analogWrite(RPWM_Output, -1 * speed);
    }
}

double getTurnDeg(const int position) {
    const double Kp = 0.0158;
    // const double Kp = 0.65;
    const double Ki = 0.0000000;
//    const double Ki = 0.000007;
    const double Kd = 0.085;
    const int midPosition = 4000;

    static double previous_error = 0;
    static double integral = 0;
    double derivative = 0;
    double correction = 0;
    int error = 0;
    error = position - midPosition;

    integral += error;
    derivative = error - previous_error;
    correction = Kp * error + Ki * integral + Kd * derivative;
    previous_error = error;
    return correction;
}

int getPosition(QTRSensors &qtr, bool &atFinish) {
    uint16_t array[SENSOR_COUNT];
    uint16_t position = qtr.readLineWhite(array);
    if (array[0] == 0 && array[7] == 0) {
        atFinish = true;
    }
    return position;
}

double getUSValues() {
    unsigned long duration;
    digitalWrite(UlTRASONIC_TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(UlTRASONIC_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(UlTRASONIC_TRIG_PIN, LOW);
    duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH);
    return duration * 0.034 / 2;
}

bool obstacleDetected(double distance) {
    if (distance <= MAX_DETECT_DISTANCE) {
        return true;
    }
    return false;
}


void avoidObstacle(Servo &myServo, double currentSpeed,  Adafruit_MPU6050& mpu, double& inclination) {
    unsigned long time = millis();
    while(millis() < time +currentSpeed*1100){
        drive(myServo, -DEFAULT_SPEED, 0);
        inclination += getInclination(mpu);
    }
    time = millis();
    while(millis() < time +currentSpeed*1000){
        drive(myServo, DEFAULT_SPEED, SERVO_DEG_MIN - SERVO_MID);
        inclination += getInclination(mpu);
    }
    time = millis();
    while(millis() < time +currentSpeed*900){
         drive(myServo, DEFAULT_SPEED, SERVO_DEG_MAX);
        inclination += getInclination(mpu);
    }
    time = millis();
    while(millis() < time +currentSpeed*950){
        drive(myServo, -DEFAULT_SPEED, SERVO_DEG_MIN - SERVO_MID);
        inclination += getInclination(mpu);
    }

    time = millis();
    while(millis() < time +currentSpeed*300){
        drive(myServo, DEFAULT_SPEED, SERVO_DEG_MAX);
        inclination += getInclination(mpu);
    }

}


double getInclination(Adafruit_MPU6050 &mpu) {
    static unsigned long prev_time = millis();
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    unsigned long delta_time = millis() - prev_time;
    prev_time = millis();
    if(abs(g.gyro.y)>0.12) {
        return -1 * 57.29578 * g.gyro.y * delta_time / 1000;
    }
    return 0;
}


void avoidHill(QTRSensors &qtr, Servo &myServo, Adafruit_MPU6050 &mpu,  double& inclination) {
    bool finishDetected = false;
    int position = getPosition(qtr, finishDetected);
    double correction = getTurnDeg(position);
    inclination += getInclination(mpu);
    //Serial.println(inclination);
   // unsigned long time = millis();
    while (inclination < 20){
        position = getPosition(qtr, finishDetected);
        correction = getTurnDeg(position);
        drive(myServo, DEFAULT_SPEED * HILL_ACCELERATION, correction);
        inclination += getInclination(mpu);
       // Serial.println(inclination);
    }
    while (inclination >= 8){
        drive(myServo, DEFAULT_SPEED * HILL_ACCELERATION, 0);
        inclination += getInclination(mpu);
       // Serial.println(inclination);
    }
    while (inclination >= -15){
        drive(myServo, DEFAULT_SPEED / 5, 0);
        inclination += getInclination(mpu);
        //Serial.println(inclination);
    }
    inclination += getInclination(mpu);
    while (inclination < -10){
        position = getPosition(qtr, finishDetected);
        correction = getTurnDeg(position);
        drive(myServo, DEFAULT_SPEED / 5, correction);
        inclination += getInclination(mpu);
      //  Serial.println(inclination);
    }



}

void stopCar(Servo &myServo) {
    drive(myServo, -20, 0);
    delay(10);
    drive(myServo, 0, 0);
}