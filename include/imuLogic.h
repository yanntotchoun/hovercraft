#ifndef IMULOGIC_H
#define IMULOGIC_H

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include "imu.h"
#include "tim.h"


#define MAX_YAW 84
#define GYRO_SENS_Z 65.5f// LSB per deg/s

extern const uint16_t Servo_angle[256];


extern volatile struct IMU_data imu;

void imu_calibration();


void getValuesImu();

 uint8_t yaw_to_idx(float y);

void drift_algorithm();


#endif