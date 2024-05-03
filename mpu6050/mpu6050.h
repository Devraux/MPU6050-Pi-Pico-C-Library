#ifndef _mpu6050_
#define _mpu6050_

#include <stdbool.h>
#include <stdio.h>
#include <assert.h>
#include "hardware/uart.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"


//I2C INTERFACE//
#define SDA_Pin 26
#define SCL_Pin 27

typedef struct MPU6050_REG
{
    const uint8_t address; //device address
    const uint8_t who_i_am_add;
    const uint8_t reset_add; //reset address
    const uint8_t accel_add; //accelerator data address register
    const uint8_t gyro_add; //gryoscope data address register
    const uint8_t temp_add; //temperature data address register
}MPU6050_REG;


typedef struct MPU6050
{
    int16_t acceleration[3];
    int16_t gyro[3];
    int16_t temp;
}MPU6050;

void mpu_init(); //mpu I2C init
void mpu_cal(); // mpu6050 sensor self test to get accuraate results
void mpu_reset(); // reset
void who_i_am(uint8_t* mpu_address); //get I2C address
void mpu_read(MPU6050* mpu6050); //read data from mpu6050


#endif