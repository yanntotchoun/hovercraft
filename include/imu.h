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


void imu_lowlevel_init(void);

int16_t readImuGyroZ();



#endif