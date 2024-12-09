#include "tof_sensors.h"
#include <Arduino.h>
// #include "Adafruit_VL53L0X.h"s
#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();

VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
VL53L0X_RangingMeasurementData_t measure3;

int setup_tof()
{
    // while (! Serial)
    Serial.begin(115200);
    while (!Serial)
    {
        delay(1);
    }

    pinMode(X_SHT1, OUTPUT);
    pinMode(X_SHT2, OUTPUT);
    pinMode(X_SHT3, OUTPUT);

    // digitalWrite(X_SHT1, LOW);
    // digitalWrite(X_SHT2, LOW);
    // digitalWrite(X_SHT3, LOW);

    setID();
}

void setID()
{

    digitalWrite(X_SHT1, LOW);
    digitalWrite(X_SHT2, LOW);
    digitalWrite(X_SHT3, LOW);
    delay(10);

    digitalWrite(X_SHT1, HIGH);
    digitalWrite(X_SHT2, HIGH);
    digitalWrite(X_SHT3, HIGH);
    delay(10);

    digitalWrite(X_SHT1, HIGH);
    digitalWrite(X_SHT2, LOW);
    digitalWrite(X_SHT3, LOW);
    // delay(10);

    if (!lox1.begin(TOF1_ADDRESS))
    {
        while (1)
            ;
    }
    delay(10);

    // activatate TOF2
    digitalWrite(X_SHT2, HIGH);
    delay(10);
    if (!lox2.begin(TOF2_ADDRESS))
    {
        while (1)
            ;
    }
    delay(10);

    // activatate TOF3
    digitalWrite(X_SHT3, HIGH);
    delay(10);
    if (!lox3.begin(TOF3_ADDRESS))
    {
        while (1)
            ;
    }
    delay(10);
}

void read_sensors()
{

    lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
    lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!
    lox3.rangingTest(&measure3, false); // pass in 'true' to get debug data printout!

    Serial.print(F("1: "));
    if (measure1.RangeStatus != 4)
    { // if not out of range
        Serial.print(measure1.RangeMilliMeter);
    }
    else
    {
        Serial.print(F("Out of range"));
    }

    Serial.print(F(" "));

    // print sensor two reading
    Serial.print(F("2: "));
    if (measure2.RangeStatus != 4)
    {
        Serial.print(measure2.RangeMilliMeter);
    }
    else
    {
        Serial.print(F("Out of range"));
    }

    Serial.println();
}