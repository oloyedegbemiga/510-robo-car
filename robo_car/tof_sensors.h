#ifndef TOFSENSOR
#define TOFSENSOR

// #include "Adafruit_VL53L0X.h"

#define TOF1_ADDRESS 0x30
#define TOF2_ADDRESS 0x31
#define TOF3_ADDRESS 0x32

#define X_SHT1 7
#define X_SHT2 6
#define X_SHT3 5

// Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
// Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
// Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();

// VL53L0X_RangingMeasurementData_t measure1;
// VL53L0X_RangingMeasurementData_t measure2;
// VL53L0X_RangingMeasurementData_t measure3;

int setup_tof();
void setID();
void read_sensors();

#endif