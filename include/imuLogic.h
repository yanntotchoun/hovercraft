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
extern volatile uint32_t imuTime;
extern struct IMU_data imu;

void imu_init(void);
void imu_calibration(uint16_t samples);
void imu_update(void);
float imu_get_yaw(void);
void imu_reset_yaw(void);
uint16_t yaw_to_pwm(float yaw);
uint8_t yaw_to_idx(float y);

void drift_algorithm();


#endif