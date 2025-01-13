#include "mpu6050.h"

struct repeating_timer timer;

void i2c_write_reg(uint8_t i2c_address, uint8_t reg, uint8_t data)
{
    uint8_t tab[] = {reg, data};
    i2c_write_blocking(I2C_Instance, i2c_address, tab, sizeof(tab)/sizeof(tab[0]), false);
}
 
uint8_t who_i_am(void)
{
    uint8_t who_i_am_t;
    i2c_write_blocking(I2C_Instance, MPU6050_Address, (uint8_t*)MPU6050_Who_I_am_add, 2, true); 
    i2c_read_blocking(I2C_Instance, MPU6050_Address, &who_i_am_t, 2, false); 
    
    // YOU CAN COMMENT THIS LINE //
    printf("Device address: %x", who_i_am);

    return who_i_am_t;
}

void mpu_reset(void)
{
    i2c_write_reg(MPU6050_Address, MPU6050_Reset_add, 0x80);
    sleep_ms(50); //Necessary sensor delay
    i2c_write_reg(MPU6050_Address, MPU6050_Reset_add, 0x00);
    sleep_ms(50); //Necessary sensor delay
}

void mpu_init(MPU6050* mpu6050)
{
    // I2C INIT //
    i2c_init(I2C_Instance, 400000);
    gpio_set_function(SDA_Pin, GPIO_FUNC_I2C); //27 and 26
    gpio_set_function(SCL_Pin, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_Pin);
    gpio_pull_up(SCL_Pin);

    //RING BUFFER INIT //
    Ring_buffer_init(&mpu6050->mpu6050_data.accelbuffer, 10);
    Ring_buffer_init(&mpu6050->mpu6050_data.gyrobuffer, 10);

    // MPU6050 SENSOR INIT //
    mpu_reset();
    mpu_set_sample_rate(1);
    mpu_set_resolution(0, 0, mpu6050);
    mpu_get_offset(mpu6050);
    //mpu_get_statistic(mpu6050);

    //MPU6050 INTERRUPT INIT
    add_repeating_timer_ms(-200, mpu_callback, (void*)mpu6050, &timer); // 5 times per second
}

void mpu_read_raw(MPU6050* mpu6050)
{
    uint8_t buffer[6];
    int16_t temperature = 0;

    //ACCELERATION
    i2c_write_blocking(I2C_Instance, MPU6050_Address, (uint8_t*)MPU6050_Accel_add, 1, true); 
    i2c_read_blocking(I2C_Instance, MPU6050_Address, buffer, 6, false);

    mpu6050->mpu6050_data.accel_raw[0] = (buffer[0] << 8) | buffer[1];
    mpu6050->mpu6050_data.accel_raw[1] = (buffer[2] << 8) | buffer[3];
    mpu6050->mpu6050_data.accel_raw[2] = (buffer[4] << 8) | buffer[5];

    //GYROSCOPE
    i2c_write_blocking(I2C_Instance,MPU6050_Address, (uint8_t*)MPU6050_Gyro_add, 1, true);
    i2c_read_blocking(I2C_Instance, MPU6050_Address, buffer, 6, false);  

    mpu6050->mpu6050_data.gyro_raw[0] = (buffer[0] << 8) | buffer[1];
    mpu6050->mpu6050_data.gyro_raw[1] = (buffer[2] << 8) | buffer[3];
    mpu6050->mpu6050_data.gyro_raw[2] = (buffer[4] << 8) | buffer[5];


    //TEMPERATURE
    i2c_write_blocking(I2C_Instance,MPU6050_Address, (uint8_t*)MPU6050_Temp_add, 1, true);
    i2c_read_blocking(I2C_Instance, MPU6050_Address, buffer, 2, false);  

    temperature = buffer[0] << 8 | buffer[1];
    //mpu6050->mpu6050_raw.temp = (temperature / 340.f) + 36.53;
    mpu6050->mpu6050_data.temp_raw = temperature; 
}

void mpu_set_resolution(uint8_t gyro_res, uint8_t acc_res, MPU6050* mpu6050)
{
    uint8_t check, resolution = 0;
    uint8_t res_index = 0;
    uint16_t res_value = 0;
 
    //GYROSCOPE RESOLUTION
    switch(gyro_res)
    {
        case 0: //+- 250
            resolution = 0b00000000;
            res_index = 0;
            break;

        case 1: //+- 500
            resolution = 0b00001000;
            res_index = 1;
            break;

        case 2: //+- 1000
            resolution = 0b00010000;
            res_index = 2;
            break;

        case 3: //+- 2000
            resolution = 0b00011000;
            res_index = 3;
            break;
    }

    i2c_write_reg(MPU6050_Address, MPU6050_gyro_res, resolution);
    mpu6050->mpu6050_state.gyro_res = res_index;


    //ACCELEROMETER RESOLUTION
    switch(acc_res)
    {
        case 0: //+- 2
            resolution = 0b00000000;
            res_index = 0;
            res_value = 16384;
            break;

        case 1: //+- 4
            resolution = 0b00001000;
            res_index = 1;
            res_value = 8192;
            break;

        case 2: //+- 8
            resolution = 0b00010000;
            res_index = 2;
            res_value = 4096;
            break;

        case 3: //+- 16
            resolution = 0b00011000;
            res_index = 3;
            res_value = 2048;
            break;
    }
    
    i2c_write_reg(MPU6050_Address, MPU6050_Acc_config ,resolution);
    mpu6050->mpu6050_state.accel_res = res_index;
    mpu6050->mpu6050_state.accel_res_val = res_value;
}

bool mpu_accel_st(MPU6050* mpu6050, MPU6050_SELFTEST* mpu6050_accel_st)
{
    uint8_t gyro_res_mem = mpu6050->mpu6050_state.gyro_res;
    uint8_t accel_res_mem = mpu6050->mpu6050_state.accel_res;
    uint8_t mask = 0b00000011;

    mpu_read_raw(mpu6050); // read data before selF test
    int16_t accel_x = mpu6050->mpu6050_data.accel_raw[0];
    int16_t accel_y = mpu6050->mpu6050_data.accel_raw[1];
    int16_t accel_z = mpu6050->mpu6050_data.accel_raw[2];

    i2c_write_reg(MPU6050_Address, MPU6050_Acc_config, 0b11110000); // enable selft test | set +-8g

    i2c_write_blocking(I2C_Instance, MPU6050_Address, (uint8_t*)MPU6050_XA_TEST, 1, true); 
    i2c_read_blocking(I2C_Instance, MPU6050_Address, &mpu6050_accel_st->X_TEST, 1, false);
    
    i2c_write_blocking(I2C_Instance, MPU6050_Address, (uint8_t*)MPU6050_YA_TEST, 1, true); 
    i2c_read_blocking(I2C_Instance, MPU6050_Address, &mpu6050_accel_st->Y_TEST, 1, false);
    
    i2c_write_blocking(I2C_Instance, MPU6050_Address, (uint8_t*)MPU6050_ZA_TEST, 1, true); 
    i2c_read_blocking(I2C_Instance, MPU6050_Address, &mpu6050_accel_st->Z_TEST, 1, false);
    
    i2c_write_blocking(I2C_Instance, MPU6050_Address, (uint8_t*)MPU6050_A_TEST, 1, true); 
    i2c_read_blocking(I2C_Instance, MPU6050_Address, &mpu6050_accel_st->A_TEST, 1, false);
    
    mpu6050_accel_st->X_TEST = (mpu6050_accel_st->X_TEST >> 3); mpu6050_accel_st->X_TEST |= ((mpu6050_accel_st->A_TEST >> 4) & mask);
    mpu6050_accel_st->Y_TEST = (mpu6050_accel_st->Y_TEST >> 3); mpu6050_accel_st->Y_TEST |= ((mpu6050_accel_st->A_TEST >> 2) & mask);
    mpu6050_accel_st->Z_TEST = (mpu6050_accel_st->Z_TEST >> 3); mpu6050_accel_st->Z_TEST |= (mpu6050_accel_st->A_TEST & mask);
    
    mpu6050_accel_st->FT_X = 4096 * pow(0.92f, (mpu6050_accel_st->X_TEST - 1.0f) / 30.0f);
    mpu6050_accel_st->FT_Y = 4096 * pow(0.92f, (mpu6050_accel_st->Y_TEST - 1.0f) / 30.0f);
    mpu6050_accel_st->FT_Z = 4096 * pow(0.92f, (mpu6050_accel_st->Z_TEST - 1.0f) / 30.0f);

    mpu_read_raw(mpu6050); // read data while self test is enabled
    mpu6050_accel_st->STR_X = mpu6050->mpu6050_data.accel_raw[0] - accel_x;
    mpu6050_accel_st->STR_Y = mpu6050->mpu6050_data.accel_raw[1] - accel_y;
    mpu6050_accel_st->STR_Z = mpu6050->mpu6050_data.accel_raw[2] - accel_z;

    mpu6050_accel_st->X_ERROR = (mpu6050_accel_st->STR_X - mpu6050_accel_st->FT_X) / mpu6050_accel_st->FT_X;
    mpu6050_accel_st->Y_ERROR = (mpu6050_accel_st->STR_Y - mpu6050_accel_st->FT_Y) / mpu6050_accel_st->FT_Y;
    mpu6050_accel_st->Z_ERROR = (mpu6050_accel_st->STR_Z - mpu6050_accel_st->FT_Z) / mpu6050_accel_st->FT_Z;
    
    mpu_set_resolution(gyro_res_mem, accel_res_mem, mpu6050);// After self test come back to old config values and disable self test mode 
    return true;
}

bool mpu_gyro_st(MPU6050* mpu6050, MPU6050_SELFTEST* mpu6050_gyro_st)
{
    uint8_t gyro_res_mem = mpu6050->mpu6050_state.gyro_res;
    uint8_t accel_res_mem = mpu6050->mpu6050_state.accel_res;
    uint8_t mask = 0b00011111;

    mpu_read_raw(mpu6050); // read data before selF test
    uint8_t gyro_x = mpu6050->mpu6050_data.gyro_raw[0];
    uint8_t gyro_y = mpu6050->mpu6050_data.gyro_raw[1];
    uint8_t gyro_z = mpu6050->mpu6050_data.gyro_raw[2];

    i2c_write_reg(MPU6050_Address, MPU6050_Gyro_config, 0b11100000); // enable selft test | set +-250*/s

    i2c_write_blocking(I2C_Instance, MPU6050_Address, (uint8_t*)MPU6050_XA_TEST, 1, true); 
    i2c_read_blocking(I2C_Instance, MPU6050_Address, &mpu6050_gyro_st->X_TEST, 1, false);
    
    i2c_write_blocking(I2C_Instance, MPU6050_Address, (uint8_t*)MPU6050_YA_TEST, 1, true); 
    i2c_read_blocking(I2C_Instance, MPU6050_Address, &mpu6050_gyro_st->Y_TEST, 1, false);
    
    i2c_write_blocking(I2C_Instance, MPU6050_Address, (uint8_t*)MPU6050_ZA_TEST, 1, true); 
    i2c_read_blocking(I2C_Instance, MPU6050_Address, &mpu6050_gyro_st->Z_TEST, 1, false);
    
    mpu6050_gyro_st->X_TEST = (mpu6050_gyro_st->X_TEST & mask);
    mpu6050_gyro_st->Y_TEST = (mpu6050_gyro_st->Y_TEST & mask);
    mpu6050_gyro_st->Z_TEST = (mpu6050_gyro_st->Z_TEST & mask);
    
    mpu6050_gyro_st->FT_X = 25 * 131 * pow(1.046f, (mpu6050_gyro_st->X_TEST - 1.0f));
    mpu6050_gyro_st->FT_Y = -25 * 131 * pow(1.046f, (mpu6050_gyro_st->Y_TEST - 1.0f));
    mpu6050_gyro_st->FT_Z = 25 * 131 * pow(1.046f, (mpu6050_gyro_st->Z_TEST - 1.0f));

    mpu_read_raw(mpu6050); // read data while self test is enabled
    mpu6050_gyro_st->STR_X = mpu6050->mpu6050_data.gyro_raw[0] - gyro_x;
    mpu6050_gyro_st->STR_Y = mpu6050->mpu6050_data.gyro_raw[1] - gyro_y;
    mpu6050_gyro_st->STR_Z = mpu6050->mpu6050_data.gyro_raw[2] - gyro_z;

    mpu6050_gyro_st->X_ERROR = (mpu6050_gyro_st->STR_X - mpu6050_gyro_st->FT_X) / mpu6050_gyro_st->FT_X;
    mpu6050_gyro_st->Y_ERROR = (mpu6050_gyro_st->STR_Y - mpu6050_gyro_st->FT_Y) / mpu6050_gyro_st->FT_Y;
    mpu6050_gyro_st->Z_ERROR = (mpu6050_gyro_st->STR_Z - mpu6050_gyro_st->FT_Z) / mpu6050_gyro_st->FT_Z;
    
    mpu_set_resolution(gyro_res_mem, accel_res_mem, mpu6050);// After self test come back to old config values and disable self test mode 
    return true;
}

void mpu_convert(MPU6050* mpu6050)
{
    switch(mpu6050->mpu6050_state.accel_res)
    {
        case 0:
            mpu6050->mpu6050_data.accel_convert[0] = mpu6050->mpu6050_data.accel_no_offset[0] / 16384.0f;
            mpu6050->mpu6050_data.accel_convert[1] = mpu6050->mpu6050_data.accel_no_offset[1] / 16384.0f;
            mpu6050->mpu6050_data.accel_convert[2] = mpu6050->mpu6050_data.accel_no_offset[2] / 16384.0f;
        break;

        case 1:
            mpu6050->mpu6050_data.accel_convert[0] = mpu6050->mpu6050_data.accel_no_offset[0] / 8192.0f;
            mpu6050->mpu6050_data.accel_convert[1] = mpu6050->mpu6050_data.accel_no_offset[1] / 8192.0f;
            mpu6050->mpu6050_data.accel_convert[2] = mpu6050->mpu6050_data.accel_no_offset[2] / 8192.0f;
        break;

        case 2:
            mpu6050->mpu6050_data.accel_convert[0] = mpu6050->mpu6050_data.accel_no_offset[0] / 4096.0f;
            mpu6050->mpu6050_data.accel_convert[1] = mpu6050->mpu6050_data.accel_no_offset[1] / 4096.0f;
            mpu6050->mpu6050_data.accel_convert[2] = mpu6050->mpu6050_data.accel_no_offset[2] / 4096.0f;
        break;

        case 3:
            mpu6050->mpu6050_data.accel_convert[0] = mpu6050->mpu6050_data.accel_no_offset[0] / 2048.0f;
            mpu6050->mpu6050_data.accel_convert[1] = mpu6050->mpu6050_data.accel_no_offset[1] / 2048.0f;
            mpu6050->mpu6050_data.accel_convert[2] = mpu6050->mpu6050_data.accel_no_offset[2] / 2048.0f;
        break;
    
    }

    switch(mpu6050->mpu6050_state.gyro_res)
    {
        case 0:
            mpu6050->mpu6050_data.gyro_convert[0] = mpu6050->mpu6050_data.gyro_no_offset[0] / 131.0f;
            mpu6050->mpu6050_data.gyro_convert[1] = mpu6050->mpu6050_data.gyro_no_offset[1] / 131.0f;
            mpu6050->mpu6050_data.gyro_convert[2] = mpu6050->mpu6050_data.gyro_no_offset[2] / 131.0f;
        break;

        case 1:
            mpu6050->mpu6050_data.gyro_convert[0] = mpu6050->mpu6050_data.gyro_no_offset[0] / 65.5f;
            mpu6050->mpu6050_data.gyro_convert[1] = mpu6050->mpu6050_data.gyro_no_offset[1] / 65.5f;
            mpu6050->mpu6050_data.gyro_convert[2] = mpu6050->mpu6050_data.gyro_no_offset[2] / 65.5f;
        break;

        case 2:
            mpu6050->mpu6050_data.gyro_convert[0] = mpu6050->mpu6050_data.gyro_no_offset[0] / 32.8f;
            mpu6050->mpu6050_data.gyro_convert[1] = mpu6050->mpu6050_data.gyro_no_offset[1] / 32.8f;
            mpu6050->mpu6050_data.gyro_convert[2] = mpu6050->mpu6050_data.gyro_no_offset[2] / 32.8f;
        break;

        case 3:
            mpu6050->mpu6050_data.gyro_convert[0] = mpu6050->mpu6050_data.gyro_no_offset[0] / 16.4f;
            mpu6050->mpu6050_data.gyro_convert[1] = mpu6050->mpu6050_data.gyro_no_offset[1] / 16.4f;
            mpu6050->mpu6050_data.gyro_convert[2] = mpu6050->mpu6050_data.gyro_no_offset[2] / 16.4f;
        break;
    }
}

void mpu_set_sample_rate(uint8_t divider)
{
    i2c_write_reg(MPU6050_Address, MPU6050_Config, 0b00000110); // => set DLPF as 1kHz

    switch(divider)
    {
        case 1:
            i2c_write_reg(MPU6050_Address, MPU6050_SMPLRT_DIV, 0b00000001);
        break;

        case 2:
            i2c_write_reg(MPU6050_Address, MPU6050_SMPLRT_DIV, 0b00000010);
        break;

        case 4:
            i2c_write_reg(MPU6050_Address, MPU6050_SMPLRT_DIV, 0b00000100);
        break;

        case 8:
            i2c_write_reg(MPU6050_Address, MPU6050_SMPLRT_DIV, 0b00001000);
        break;

        case 16:
            i2c_write_reg(MPU6050_Address, MPU6050_SMPLRT_DIV, 0b00010000);
        break;
        
        case 32:
            i2c_write_reg(MPU6050_Address, MPU6050_SMPLRT_DIV, 0b00100000);
        break;

        case 64:
            i2c_write_reg(MPU6050_Address, MPU6050_SMPLRT_DIV, 0b01000000);
        break;

        case 128:
            i2c_write_reg(MPU6050_Address, MPU6050_SMPLRT_DIV, 0b10000000);
        break;
    }  
}

void mpu_get_statistic(MPU6050* mpu6050)
{
    int16_t acc_x[200], acc_y[200], acc_z[200];
    int16_t gyro_x[200], gyro_y[200], gyro_z[200];
    int16_t var_acc_x, var_acc_y, var_acc_z, var_gyro_x, var_gyro_y, var_gyro_z;


    for(uint8_t i; i < 200; i++) // measure output 50 times
    {
        mpu_read(mpu6050);
        acc_x[i] = mpu6050->mpu6050_data.accel_no_offset[0];  acc_y[i] = mpu6050->mpu6050_data.accel_no_offset[1];  acc_z[i] = mpu6050->mpu6050_data.accel_no_offset[2];
        gyro_x[i] = mpu6050->mpu6050_data.gyro_no_offset[0];  gyro_y[i] = mpu6050->mpu6050_data.gyro_no_offset[1];  gyro_z[i] = mpu6050->mpu6050_data.gyro_no_offset[2];
        sleep_ms(20); // wait fornext measurement
    }

    var_acc_x = get_variance(acc_x, 200); var_acc_y = get_variance(acc_y, 200); var_acc_z = get_variance(acc_z, 200);
    var_gyro_x = get_variance(gyro_x, 200); var_gyro_y= get_variance(gyro_y, 200); var_gyro_z = get_variance(gyro_z, 200);

    mpu6050->mpu6050_state.accel_x_deviation = var_acc_x; mpu6050->mpu6050_state.accel_y_deviation = var_acc_y; mpu6050->mpu6050_state.accel_z_deviation = var_acc_z; 
    mpu6050->mpu6050_state.gyro_x_deviation = var_gyro_x; mpu6050->mpu6050_state.gyro_y_deviation = var_gyro_y; mpu6050->mpu6050_state.gyro_z_deviation = var_gyro_z;
}

void mpu_read(MPU6050* mpu6050)
{
    mpu_read_raw(mpu6050);

    mpu6050->mpu6050_data.accel_no_offset[0] = mpu6050->mpu6050_data.accel_raw[0] - mpu6050->mpu6050_state.accel_x_offset;
    mpu6050->mpu6050_data.accel_no_offset[1] = mpu6050->mpu6050_data.accel_raw[1] - mpu6050->mpu6050_state.accel_y_offset;
    mpu6050->mpu6050_data.accel_no_offset[2] = mpu6050->mpu6050_data.accel_raw[2] - mpu6050->mpu6050_state.accel_z_offset; 

    mpu6050->mpu6050_data.gyro_no_offset[0] = mpu6050->mpu6050_data.gyro_raw[0] - mpu6050->mpu6050_state.gyro_x_offset;
    mpu6050->mpu6050_data.gyro_no_offset[1] = mpu6050->mpu6050_data.gyro_raw[1] - mpu6050->mpu6050_state.gyro_y_offset;
    mpu6050->mpu6050_data.gyro_no_offset[2] = mpu6050->mpu6050_data.gyro_raw[2] - mpu6050->mpu6050_state.gyro_z_offset;    
    
    mpu_convert(mpu6050); 
    mpu_remove_gravity(mpu6050);
    mpu_get_theta(mpu6050);

    Ring_buffer_push(&mpu6050->mpu6050_data.accelbuffer, mpu6050->mpu6050_data.accel_no_gravity[0], mpu6050->mpu6050_data.accel_no_gravity[1], mpu6050->mpu6050_data.accel_no_gravity[2]);
    Ring_buffer_push(&mpu6050->mpu6050_data.gyrobuffer, mpu6050->mpu6050_data.gyro_convert[0], mpu6050->mpu6050_data.gyro_convert[1], mpu6050->mpu6050_data.gyro_convert[2]); 
}

void mpu_get_offset(MPU6050* mpu6050)
{
    int32_t accel_X_offset, accel_Y_offset, accel_Z_offset = 0;
    int32_t gyro_X_offset, gyro_Y_offset, gyro_Z_offset = 0;
    
    for(uint8_t i = 0; i < 200; i++) // make 200 meaesurements and get average
    {
        mpu_read_raw(mpu6050);
        accel_X_offset += mpu6050->mpu6050_data.accel_raw[0];
        accel_Y_offset += mpu6050->mpu6050_data.accel_raw[1];
        accel_Z_offset += mpu6050->mpu6050_data.accel_raw[2] - mpu6050->mpu6050_state.accel_res_val;

        gyro_X_offset += mpu6050->mpu6050_data.gyro_raw[0];
        gyro_Y_offset += mpu6050->mpu6050_data.gyro_raw[1];
        gyro_Z_offset += mpu6050->mpu6050_data.gyro_raw[2];

        sleep_ms(20); // wait for next measurement from mpu
    }

    accel_X_offset /= 200; accel_Y_offset /= 200; accel_Z_offset /= 200;
    gyro_X_offset /= 200; gyro_Y_offset /= 200; gyro_Z_offset /= 200;

    mpu6050->mpu6050_state.accel_x_offset = accel_X_offset; 
    mpu6050->mpu6050_state.accel_y_offset = accel_Y_offset;
    mpu6050->mpu6050_state.accel_z_offset = accel_Z_offset;

    mpu6050->mpu6050_state.gyro_x_offset = gyro_X_offset;
    mpu6050->mpu6050_state.gyro_y_offset = gyro_Y_offset;
    mpu6050->mpu6050_state.gyro_z_offset = gyro_Z_offset;
}

void mpu_remove_gravity(MPU6050* mpu6050)
{
    static float gravity[3] = {0, 0, 0};
    float alpha = 0.8;

    gravity[0] = alpha * gravity[0] + (1 - alpha) * mpu6050->mpu6050_data.accel_convert[0];
    gravity[1] = alpha * gravity[1] + (1 - alpha) * mpu6050->mpu6050_data.accel_convert[1];
    gravity[2] = alpha * gravity[2] + (1 - alpha) * mpu6050->mpu6050_data.accel_convert[2];

    mpu6050->mpu6050_data.accel_no_gravity[0] = mpu6050->mpu6050_data.accel_convert[0] - gravity[0];
    mpu6050->mpu6050_data.accel_no_gravity[1] = mpu6050->mpu6050_data.accel_convert[1] - gravity[1];
    mpu6050->mpu6050_data.accel_no_gravity[2] = mpu6050->mpu6050_data.accel_convert[2] - gravity[2];

    mpu6050->mpu6050_data.accel_mod_no_gravity = sqrtf(powf(mpu6050->mpu6050_data.accel_no_gravity[0], 2) +
    powf(mpu6050->mpu6050_data.accel_no_gravity[1], 2) + powf(mpu6050->mpu6050_data.accel_no_gravity[2], 2)); 
}

void mpu_get_distance(MPU6050* mpu6050)
{
    float distance = 0;
    float accelX[mpu6050->mpu6050_data.accelbuffer.Buffer_Size], accelY[mpu6050->mpu6050_data.accelbuffer.Buffer_Size], accelZ[mpu6050->mpu6050_data.accelbuffer.Buffer_Size];
    float mod_buf[mpu6050->mpu6050_data.accelbuffer.Buffer_Size];

    for(uint16_t i = 0; i < mpu6050->mpu6050_data.accelbuffer.Buffer_Size; i++)
    {
        if(mpu6050->mpu6050_data.accelbuffer.DataX[i] >= 0.001f)
            accelX[i] = mpu6050->mpu6050_data.accelbuffer.DataX[i];
        else
            accelX[i] = 0;

        if(mpu6050->mpu6050_data.accelbuffer.DataX[i] >= 0.001f)
            accelY[i] = mpu6050->mpu6050_data.accelbuffer.DataY[i];
        else
            accelY[i] = 0;

        if(mpu6050->mpu6050_data.accelbuffer.DataX[i] >= 0.001f)
            accelZ[i] = mpu6050->mpu6050_data.accelbuffer.DataZ[i];    
        else
            accelZ[i] = 0;    
    }

    for(uint16_t i = 0; i < mpu6050->mpu6050_data.accelbuffer.Buffer_Size; i++)
        mod_buf[i] = sqrtf(powf(accelX[i], 2) + powf(accelY[i], 2) + powf(accelZ[i], 2));
    
    float vel[mpu6050->mpu6050_data.accelbuffer.Buffer_Size]; // velocity
    float dist[mpu6050->mpu6050_data.accelbuffer.Buffer_Size - 1]; // distance
    float dist_sum = 0;

    for(uint16_t i = 0; i < mpu6050->mpu6050_data.accelbuffer.Buffer_Size - 1; i++)
        vel[i] = 0.5 * 0.2 * 9.81 * (mod_buf[i] + mod_buf[i + 1]);
    
    for(uint16_t i = 0; i < mpu6050->mpu6050_data.accelbuffer.Buffer_Size - 2; i++)
        dist[i] = 0.5 * 0.2 * (vel[i] + vel[i + 1]);

    for(uint16_t i = 0; i < mpu6050->mpu6050_data.accelbuffer.Buffer_Size - 2; i++)
        dist_sum += dist[i];


    mpu6050->mpu6050_data.distance += dist_sum;
    Ring_buffer_clear(&mpu6050->mpu6050_data.accelbuffer);


   mpu6050->mpu6050_data.distance += distance;
   Ring_buffer_clear(&mpu6050->mpu6050_data.accelbuffer);
}

void mpu_get_theta(MPU6050* mpu6050)
{
    mpu6050->mpu6050_data.theta_roll  = (mpu6050->mpu6050_data.theta_roll  + mpu6050->mpu6050_data.gyro_convert[1] * 0.2f) + (0.02f * mpu6050->mpu6050_data.accel_no_gravity[1] * (9.81f));
    mpu6050->mpu6050_data.theta_pitch = (mpu6050->mpu6050_data.theta_pitch + mpu6050->mpu6050_data.gyro_convert[0] * 0.2f) + (0.02f * mpu6050->mpu6050_data.accel_no_gravity[0] * (9.81f));
    mpu6050->mpu6050_data.theta_yaw   = (mpu6050->mpu6050_data.theta_yaw   + mpu6050->mpu6050_data.gyro_convert[2] * 0.2f) + (0.02f * mpu6050->mpu6050_data.accel_no_gravity[2] * (9.81f));
}

bool mpu_callback(struct repeating_timer *timer)
{
    MPU6050* mpu6050 = (MPU6050*)timer->user_data;   // void* casting 
    mpu_read(mpu6050);
    
    if(mpu6050->mpu6050_data.accelbuffer.Counter == mpu6050->mpu6050_data.accelbuffer.Buffer_Size) // compute data every 1 sec
        mpu_get_distance(mpu6050);

    return true;
}

void mpu_fifo_en(bool temp_en, bool acc_en, bool gyro_en)
{

    i2c_write_reg(MPU6050_Address, MPU6050_USER_CTRL, 0b01000000);
    uint8_t current_set = 0;
    uint8_t mask = 0b00000000;

    i2c_write_blocking(I2C_Instance, MPU6050_Address, (uint8_t*)MPU6050_FIFO_EN, 1, true); 
    i2c_read_blocking(I2C_Instance, MPU6050_Address, &current_set, 1, false); 

    switch(temp_en)
    {
        case 0:
            mask = current_set & 0b01111111;
            i2c_write_reg(MPU6050_Address, MPU6050_FIFO_EN, mask);
        break;

        case 1:
            mask = current_set | 0b1000000;
            i2c_write_reg(MPU6050_Address, MPU6050_FIFO_EN, mask);
        break;
    } 

    switch(acc_en)
    {
        case 0:
            mask = current_set & 0b11110111;
            i2c_write_reg(MPU6050_Address, MPU6050_FIFO_EN, mask);
        break;

        case 1:
            mask = current_set | 0b00001000;
            i2c_write_reg(MPU6050_Address, MPU6050_FIFO_EN, mask);
        break;
    } 

    switch(gyro_en)
    {
        case 0:
            mask = current_set & 0b10001111;
            i2c_write_reg(MPU6050_Address, MPU6050_FIFO_EN, mask);
        break;

        case 1:
            mask = current_set | 0b01110000;
            i2c_write_reg(MPU6050_Address, MPU6050_FIFO_EN, mask);
        break;
    } 

    // ENABLE FIFO INTERRUPT //

    uint8_t int_current_set = 0;
    uint8_t int_mask = 0b00010000;  

    i2c_write_blocking(I2C_Instance, MPU6050_Address, (uint8_t*)MPU6050_INT_ENABLE, 1, true); 
    i2c_read_blocking(I2C_Instance, MPU6050_Address, &int_current_set, 1, false); 

    i2c_write_reg(MPU6050_Address, MPU6050_INT_ENABLE, int_mask | int_current_set);
}

int16_t get_variance(int16_t* data, uint8_t data_size)
{
    int16_t variance = 0;
    int32_t sum = 0;
    int16_t mean = 0;
    int16_t x[data_size];

    for(uint8_t i = 0; i < data_size; i++)
        sum += data[i];
        
    mean = sum / data_size;
    sum = 0;

    for(uint8_t i = 0; i < data_size; i++)
        x[i] = data[i] - mean;

    for(uint8_t i = 0; i < data_size; i++)
        sum += pow(x[i], 2);
    
    variance = sum / data_size;

    return variance;
}