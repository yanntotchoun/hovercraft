#ifndef IMU_H
#define IMU_H

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

void getImuAcc();

void getImuTemp();

void getImuGyro();



#endif