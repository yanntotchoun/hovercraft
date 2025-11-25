#include <avr/io.h>
#include <avr/boot.h>
#include <util/delay.h>



void getImuAccX(){return imu_read_word(0x3B);}
void getImuAccY(){return imu_read_word(0x3D);}
void getImuAccZ(){return imu_read_word(0x3F);}


void getImuTemp(){return imu_read_word(0x41);}

void readImuGyroX(){return imu_read_word(0x43);}
void readImuGyroY(){return imu_read_word(0x45);}
void readImuGyroZ(){return imu_read_word(0x47);}



