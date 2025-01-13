#ifndef _mpu6050_
#define _mpu6050_

#include <stdbool.h>
#include "pico/stdio.h"
#include <pico/stdlib.h>
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/irq.h"
#include "../kalman_filter/kalman_filter.h"
#include "../ringbuffer/ringbuffer.h"
#include <math.h>
#include <string.h>

//I2C INTERFACE//
#define SDA_Pin 26          // Exemplary PI PICO SDA PIN
#define SCL_Pin 27          // Exemplary PI PICO SCL PIN
#define I2C_Instance i2c1   // Exemplary PI PICO I2C INSTANCE

// SOME OF UNNECESSARY(USED IN THIS PROJECT) REGISTERS BASED ON MPU6050 DOCUMENTATION // 
#define MPU6050_Address          0x68 // device address
#define MPU6050_Who_I_am_add     0x75 // Who I am address
#define MPU6050_Reset_add        0x6B // reset address
#define MPU6050_Accel_add        0x3B // accelerator data address register
#define MPU6050_Gyro_add         0x43 // gyroscope data address register
#define MPU6050_Temp_add         0x41 // temperature data address register
#define MPU6050_Acc_config       0x1C // accelerometer resolution config register and calibration
#define MPU6050_Gyro_config      0x1B // gyroscope resolution config register and calibration
#define MPU6050_gyro_res         0x1B // gyroscope resolution config register and calibration
#define MPU6050_XA_TEST          0x0D // XA_TEST and XG_test register
#define MPU6050_YA_TEST          0x0E // YA_TEST and YG_test register
#define MPU6050_ZA_TEST          0x0F // ZA_TEST and ZG_test register
#define MPU6050_A_TEST           0x10 // second accelerometer test register XA_TEST[1:0]
#define MPU6050_Config           0x1A // gyroscope DLPF_CFG set register
#define MPU6050_SMPLRT_DIV       0x19 // sample rate divider
#define MPU6050_FIFO_EN          0x23 // fifo enable
#define MPU6050_FIFO_COUNTER_H   0x72 // fifo counter high[15 : 8]
#define MPU6050_FIFO_COUNTER_L   0x73 // fifo counter high[7 : 0]
#define MPU6050_INT_ENABLE       0x38 // interrupt register
#define MPU6050_USER_CTRL        0x6A // user control

typedef struct MPU6050_STATE
{
    uint8_t accel_res; uint16_t accel_res_val;                      // 0=> 16384, 1=>8192, 2=>4096, 3=>2048  
    uint8_t gyro_res;                                               // 0=> 131,   1=>65.5, 2=>32.8, 3=>16.4 

    int16_t accel_x_deviation, accel_y_deviation, accel_z_deviation;// accelerometer standard deviation
    int16_t gyro_x_deviation, gyro_y_deviation, gyro_z_deviation;   // gyroscope standard deviation

    int16_t accel_x_offset, accel_y_offset, accel_z_offset;         // accelerometer offset
    int16_t gyro_x_offset, gyro_y_offset, gyro_z_offset;            // gyroscope offset
        
}MPU6050_STATE;

typedef struct MPU6050_DATA
{
    int16_t accel_raw[3];               // RAW X - Y - Z Acceleration
    int16_t gyro_raw[3];                // RAW X - Y - Z Gyroscope Data
    int16_t temp_raw;                   // RAW Temperature

    int16_t accel_no_offset[3];
    int16_t gyro_no_offset[3];

    float accel_mod_no_gravity;         // accelerometer vector module sqrt(X^2 + Y^2 + Z^2) without gravity constant
    uint16_t gyro_mod;                  // gyroscope vector module sqrt(X^2 + Y^2 + Z^2)

    RINGBUFFER accelbuffer;
    RINGBUFFER gyrobuffer;

    float accel_convert[3];             // converted acceleration measures
    float gyro_convert[3];              // converted gyroscope measures 

    float accel_no_gravity[3];          //user's data without offset and gravity constant

    float distance;                     // computed distance
    float theta_roll, theta_pitch, theta_yaw;      // theta nagle
}MPU6050_DATA;

typedef struct MPU6050
{
    struct MPU6050_STATE mpu6050_state;
    struct MPU6050_DATA mpu6050_data;
}MPU6050;

typedef struct MPU6050_SELFTEST
{
    uint8_t STR_X, STR_Y, STR_Z;            //STR => SELFT-TEST-RESPONSE
    float FT_X, FT_Y, FT_Z;                 //FT => FACTORY TRIMMER
    uint8_t X_TEST, Y_TEST, Z_TEST, A_TEST; // TEST REGISTER
    float X_ERROR, Y_ERROR, Z_ERROR;        //Errors given in %
}MPU6050_SELFTEST;

/// i2c_write_reg 
/// @param i2c_address  => device address
/// @param reg          => device register(place to write data)
/// @param data         => data to write in register  
void i2c_write_reg(uint8_t i2c_address, uint8_t reg, uint8_t data);

/// mpu I2C init
/// @param MPU6050 =>MPU6050 data structure 
void mpu_init(MPU6050* mpu6050); 

/// who_i_am
///@brief save MPU6050 I2C address in mpu_address param
uint8_t who_i_am(void); 

/// device reset
void mpu_reset(void); 

/// mpu_set resolution
/// @param gyro_res => gyrometer resolution <=> ±250 ±500g ±1000g ±2000g  <=user enter=> 0, 1, 2, 3
/// @param acc_res  => acceleroeter resolution <=> ±2g, ±4g, ±8g, or ±16g  <=user enter=> 0, 1, 2, 3 
/// @param MPU6050  => MPU6050 data structure 
void mpu_set_resolution(uint8_t gyro_res, uint8_t acc_res, MPU6050* mpu6050); 

/// mpu_read
/// @brief data from sensor
/// @param MPU6050 => MPU6050 data structure 
void mpu_read_raw(MPU6050* mpu6050); // read data from mpu6050

/// mpu_accel_st
/// @brief mpu6050 accelerometer sensor self test, return true if selftest goes properly and false otherwise//
/// @param MPU6050          => MPU6050 data structure 
/// @param mpu6050_accel_st => mpu6050 accel self test data structure
bool mpu_accel_st(MPU6050* mpu6050, MPU6050_SELFTEST* mpu6050_accel_st); 

/// mpu_gyro_st 
/// @brief mpu6050 gyrometer sensor self test, return true if selftest goes properly and false otherwise//
/// @param MPU6050          => MPU6050 data structure 
/// @param mpu6050_gyro_st  => mpu6050 gyro self test data structure
bool mpu_gyro_st(MPU6050* mpu6050, MPU6050_SELFTEST* mpu6050_gyro_st); 

/// mpu_convert
/// @brief convert data measured as ADC value into acceleration in [m/s2] and gyroscope [*]
/// @param MPU6050 => MPU6050 data structure 
void mpu_convert(MPU6050* mpu6050);

/// mpu_set_sample_rate
/// @brief set sample rate of gyroscope and accelerometer
/// @param divider => divide sample rate by: 1, 2, 4, 8, 16, 32, 64, 128 
void mpu_set_sample_rate(uint8_t divider);

/// mpu_statistic
/// @brief get mpu6050 statistic's information set
/// @param MPU6050 => MPU6050 data structure 
/// @return print statistics data like variance and standard deviation(sqrt(variance))
/// @details => while mpu_statistic is enabled don't move sensor 
void mpu_get_statistic(MPU6050* mpu6050);

/// get_variance
/// @brief get mpu6050 variance information
/// @param data      => input data => sensor data
/// @param data_size => data size
int16_t get_variance(int16_t* data, uint8_t data_size);

/// remove offset
/// @param MPU6050 => MPU6050 data structure 
void mpu_get_offset(MPU6050* mpu6050); 

// mpu_read
/// @param MPU6050 => mpu6050 output data
void mpu_read(MPU6050* mpu6050);

/// mpu_remove_gravity
/// @brief remove gravity from sensor output and save data in mpu6050 structure 
/// @param MPU6050 => mpu6050 output data
void mpu_remove_gravity(MPU6050* mpu6050);

/// mpu_get_distance 
/// @param MPU6050 => mpu6050 output data
void mpu_get_distance(MPU6050* mpu6050);

/// mpu_CALLBACK
/// @brief build in pi pico sdk callback 
/// @brief read data from sensor every 10 ms <> sensor measure new data after 19 ms(look mpu_set_resolution) 
bool mpu_callback(struct repeating_timer* timer);

/// mpu_get_theta 
/// @brief compute pitch and roll from accelerometer and gyroscope 
/// @param MPU6050 => mpu6050 output data
void mpu_get_theta(MPU6050* mu6050);

#endif