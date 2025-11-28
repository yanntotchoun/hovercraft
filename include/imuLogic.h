#ifndef IMULOGIC_H
#define IMULOGIC_H

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include "imu.h"
#include "tim.h"


#define MAX_YAW        180.0f//software limit(soft correction limit)
#define GYRO_SENS_Z 65.5f// LSB per deg/s

extern const uint16_t Servo_angle[256];

 struct IMU_data {
    //RAW DATA FROM IMU
    int16_t gyroZraw;
    //CONVERTED VALUES

    float gyroZ;
    //BIAS
   float gyroZbias;

    //INTEGRATED VALUES
    float yaw;
    float timeSinceLast;//dt in yaw =yaw +angular velocity *dt
    
} ;

extern struct IMU_data imu;

void imu_calibration();
       
uint8_t yaw_to_idx(float y);

void drift_algorithm(volatile uint8_t *flag);


#endif