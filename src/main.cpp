#include <Arduino.h>
#include "../test/test_functions.h"
#include <QTRSensors.h>
QTRSensors qtr;
void setup() {
    Serial.begin(9600);
    qtr.setTypeRC();
    qtr.setSensorPins(sensorPins, SENSOR_COUNT);
    qtr.setEmitterPin(EMMITER_PIN);


    /*I think the calibration has to be done only once when using the array for the first time, but am not sure*/

    // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
    // = ~25 ms per calibrate() call.
    // Call calibrate() 400 times to make calibration take about 10 seconds.
    for (uint16_t i = 0; i < 400; i++)
    {
        qtr.calibrate();
    }
    digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

    // print the calibration minimum values measured when emitters were on
    Serial.begin(9600);
    for (uint8_t i = 0; i < SENSOR_COUNT; i++)
    {
        Serial.print(qtr.calibrationOn.minimum[i]);
        Serial.print(' ');
    }
    Serial.println();

    // print the calibration maximum values measured when emitters were on
    for (uint8_t i = 0; i < SENSOR_COUNT; i++)
    {
        Serial.print(qtr.calibrationOn.maximum[i]);
        Serial.print(' ');
    }
    Serial.println();
    Serial.println();
    delay(1000);

}




void loop() {
// write your code here

}