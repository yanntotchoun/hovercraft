#include "imu.h"

void imu_lowlevel_init(void){
    Write_Reg(IMU_ADDR, REG_PWR_MGMT_1, 0x01);
    _delay_ms(10);
    Write_Reg(IMU_ADDR, REG_DIG_LOW_PASS, 0x03);
    Write_Reg(IMU_ADDR, REG_SMPLRT_DIV, 0x04);
    Write_Reg(IMU_ADDR, REG_GYRO_CFG, 0x08);
    Write_Reg(IMU_ADDR, REG_ACCEL_CFG, 0x00);
}

int16_t imu_read_word(uint8_t reg) {
    uint8_t hi, lo;

    TWI_start(IMU_ADDR, TW_WRITE);
    TWI_write(reg);
    TWI_start(IMU_ADDR, TW_READ);

    hi = TWI_ack_read();
    lo = TWI_nack_read();
    TWI_stop();

    return (int16_t)((hi << 8) | lo);
}

int16_t readImuGyroZ(void){ return imu_read_word(0x47); }
