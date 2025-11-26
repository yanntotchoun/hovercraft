#ifndef IMU_H
#define IMU_H

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include "i2c.h"


#define IMU_ADDR 0x68
#define REG_SMPLRT_DIV 0x19 //sample rate configuration
#define REG_GYRO_CFG 0x1B
#define REG_ACCEL_CFG 0x1C
#define REG_PWR_MGMT_1 0x6B //Power Management register
#define REG_ACCEL_X_H 0x3B
#define REG_WHO_AM_I 0x75
#define REG_DIG_LOW_PASS 0x1A // digital low pass filter

void imu_init();
uint16_t getImuAccX();
uint16_t getImuAccY();
uint16_t getImuAccZ();
uint16_t getImuTemp();
uint16_t readImuGyroX();
uint16_t readImuGyroY();
uint16_t readImuGyroZ();



#endif