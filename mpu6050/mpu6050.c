#include "mpu6050.h"

struct repeating_timer timer;

MPU6050_REG mpu6050_reg = {
    .address = 0x68,        //device address
    .who_i_am_add = 0x75,
    .reset_add = 0x6B,      //reset address
    .accel_add = 0x3B,      //accelerator data address register
    .gyro_add = 0x43,       //gryoscope data address register
    .temp_add = 0x41,       //temperature data address register
    .acc_config = 0x1C,     //accelerometer resolution config register and calibration
    .gyro_config = 0x1B,    // gyroscope resolution config register and calibration
    .gyro_res = 0x1B,       // gyroscope resolution config register and calibration
    .XA_TEST = 0x0D,        //XA_TEST and XG_test register
    .YA_TEST = 0x0E,        //YA_TEST and YG_test register
    .ZA_TEST = 0x0F,        //ZA_TEST and ZG_test register
    .A_TEST = 0x10,         //second accelerometer test register XA_TEST[1:0]
    .config = 0x1A,         //gyroscope DLPF_CFG set register
    .SMPLRT_DIV = 0x19      //sample rate divider
};

void i2c_write_reg(uint8_t i2c_address, uint8_t reg, uint8_t data)
{
    uint8_t tab[] = {reg, data};
    i2c_write_blocking(i2c1, i2c_address, tab, sizeof(tab), false);
}

void who_i_am(uint8_t* mpu_address)
{
    uint8_t who_i_am;
    i2c_write_blocking(i2c1, mpu6050_reg.address, &mpu6050_reg.who_i_am_add, 2, true); 
    i2c_read_blocking(i2c1, mpu6050_reg.address, &who_i_am, 2, false); 
    *mpu_address = who_i_am;

    if (who_i_am != 0x68) 
        printf("Address is not 0x68:, addres is: %d", who_i_am);
    else 
        printf("Address is 0x68, adress is : %d", who_i_am);
}

void mpu_reset()
{
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c1, mpu6050_reg.address, buf, 2, false);
    sleep_ms(50);
    uint8_t buf2[] = {0x6B, 0x80}; //0x80 => 1000 0000
    i2c_write_blocking(i2c1, mpu6050_reg.address, &buf2[0], 1, true);
    i2c_write_blocking(i2c1, mpu6050_reg.address, &buf2[1], 1, false);
    sleep_ms(50);
 }

void mpu_init(MPU6050* mpu6050)
{
    // I2C INIT //
    i2c_init(i2c1, 400000);
    gpio_set_function(SDA_Pin, GPIO_FUNC_I2C); //27 and 26
    gpio_set_function(SCL_Pin, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_Pin);
    gpio_pull_up(SCL_Pin);

    //RING BUFFER INIT //
    Ring_buffer_init(&mpu6050->mpu6050_data.accelbuffer, 50);
    Ring_buffer_init(&mpu6050->mpu6050_data.gyrobuffer, 50);

    // MPU6050 SENSOR INIT //
    mpu_reset();
    mpu_set_sample_rate(1);
    mpu_setresolution(0, 0, mpu6050);
    mpu_get_offset(mpu6050);

    //MPU6050 INTERRUPT INIT
    add_repeating_timer_ms(-10, mpu_callback, (void*)mpu6050, &timer);
}

void mpu_read_raw(MPU6050* mpu6050)
{
    uint8_t buffer[6];
    int16_t temperature = 0;

    //ACCELERATION
    i2c_write_blocking(i2c1, mpu6050_reg.address, &mpu6050_reg.accel_add, 1, true); 
    i2c_read_blocking(i2c1, mpu6050_reg.address, buffer, 6, false);

    mpu6050->mpu6050_raw.acceleration[0] = (buffer[0] << 8) | buffer[1];
    mpu6050->mpu6050_raw.acceleration[1] = (buffer[2] << 8) | buffer[3];
    mpu6050->mpu6050_raw.acceleration[2] = (buffer[4] << 8) | buffer[5];

    //GYROSCOPE
    i2c_write_blocking(i2c1,mpu6050_reg.address, &mpu6050_reg.gyro_add, 1, true);
    i2c_read_blocking(i2c1, mpu6050_reg.address, buffer, 6, false);  

    mpu6050->mpu6050_raw.gyro[0] = (buffer[0] << 8) | buffer[1];
    mpu6050->mpu6050_raw.gyro[1] = (buffer[2] << 8) | buffer[3];
    mpu6050->mpu6050_raw.gyro[2] = (buffer[4] << 8) | buffer[5];


    //TEMPERATURE
    i2c_write_blocking(i2c1,mpu6050_reg.address, &mpu6050_reg.temp_add, 1, true);
    i2c_read_blocking(i2c1, mpu6050_reg.address, buffer, 2, false);  

    temperature = buffer[0] << 8 | buffer[1];
    mpu6050->mpu6050_raw.temp = (temperature / 340.f) + 36.53;

    mpu_convert(mpu6050); // data conversion into [m/s] and [deg/s] 
}

void mpu_setresolution(uint8_t gyro_res, uint8_t acc_res, MPU6050* mpu6050)
{
    uint8_t check, resolution = 0;
    uint8_t res_value = 0;

    //GYROSCOPE RESOLUTION
    switch(gyro_res)
    {
        case 0: //+- 250
            resolution = 0b00000000;
            res_value = 0;
            break;

        case 1: //+- 500
            resolution = 0b00001000;
            res_value = 1;
            break;

        case 2: //+- 1000
            resolution = 0b00010000;
            res_value = 2;
            break;

        case 3: //+- 2000
            resolution = 0b00011000;
            res_value = 3;
            break;
    }

    i2c_write_reg(mpu6050_reg.address, mpu6050_reg.gyro_res, resolution);
    mpu6050->mpu6050_raw.gyro_res = res_value;


    //ACCELEROMETER RESOLUTION
    switch(acc_res)
    {
        case 0: //+- 2
            resolution = 0b00000000;
            res_value = 0;
            break;

        case 1: //+- 4
            resolution = 0b00001000;
            res_value = 1;
            break;

        case 2: //+- 8
            resolution = 0b00010000;
            res_value = 2;
            break;

        case 3: //+- 16
            resolution = 0b00011000;
            res_value = 3;
            break;
    }
    
    i2c_write_reg(mpu6050_reg.address,mpu6050_reg.acc_config ,resolution);
    mpu6050->mpu6050_raw.accel_res = res_value; 
}

bool mpu_accel_st(MPU6050* mpu6050, MPU6050_SELFTEST* mpu6050_accel_st)
{
    uint8_t gyro_res_mem = mpu6050->mpu6050_raw.gyro_res;
    uint8_t accel_res_mem = mpu6050->mpu6050_raw.accel_res;
    uint8_t mask = 0b00000011;

    mpu_read_raw(mpu6050); // read data before selF test
    int16_t accel_x = mpu6050->mpu6050_raw.acceleration[0];
    int16_t accel_y = mpu6050->mpu6050_raw.acceleration[1];
    int16_t accel_z = mpu6050->mpu6050_raw.acceleration[2];

    i2c_write_reg(mpu6050_reg.address, mpu6050_reg.acc_config, 0b11110000); // enable selft test | set +-8g

    i2c_write_blocking(i2c1, mpu6050_reg.address, &mpu6050_reg.XA_TEST, 1, true); 
    i2c_read_blocking(i2c1, mpu6050_reg.address, &mpu6050_accel_st->X_TEST, 1, false);
    
    i2c_write_blocking(i2c1, mpu6050_reg.address, &mpu6050_reg.YA_TEST, 1, true); 
    i2c_read_blocking(i2c1, mpu6050_reg.address, &mpu6050_accel_st->Y_TEST, 1, false);
    
    i2c_write_blocking(i2c1, mpu6050_reg.address, &mpu6050_reg.ZA_TEST, 1, true); 
    i2c_read_blocking(i2c1, mpu6050_reg.address, &mpu6050_accel_st->Z_TEST, 1, false);
    
    i2c_write_blocking(i2c1, mpu6050_reg.address, &mpu6050_reg.A_TEST, 1, true); 
    i2c_read_blocking(i2c1, mpu6050_reg.address, &mpu6050_accel_st->A_TEST, 1, false);
    
    mpu6050_accel_st->X_TEST = (mpu6050_accel_st->X_TEST >> 3); mpu6050_accel_st->X_TEST |= ((mpu6050_accel_st->A_TEST >> 4) & mask);
    mpu6050_accel_st->Y_TEST = (mpu6050_accel_st->Y_TEST >> 3); mpu6050_accel_st->Y_TEST |= ((mpu6050_accel_st->A_TEST >> 2) & mask);
    mpu6050_accel_st->Z_TEST = (mpu6050_accel_st->Z_TEST >> 3); mpu6050_accel_st->Z_TEST |= (mpu6050_accel_st->A_TEST & mask);
    
    mpu6050_accel_st->FT_X = 4096 * pow(0.92, (mpu6050_accel_st->X_TEST - 1.0) / 30.0);
    mpu6050_accel_st->FT_Y = 4096 * pow(0.92, (mpu6050_accel_st->Y_TEST - 1.0) / 30.0);
    mpu6050_accel_st->FT_Z = 4096 * pow(0.92, (mpu6050_accel_st->Z_TEST - 1.0) / 30.0);

    mpu_read_raw(mpu6050); // read data while self test is enabled
    mpu6050_accel_st->STR_X = mpu6050->mpu6050_raw.acceleration[0] - accel_x;
    mpu6050_accel_st->STR_Y = mpu6050->mpu6050_raw.acceleration[1] - accel_y;
    mpu6050_accel_st->STR_Z = mpu6050->mpu6050_raw.acceleration[2] - accel_z;

    mpu6050_accel_st->X_ERROR = (mpu6050_accel_st->STR_X - mpu6050_accel_st->FT_X) / mpu6050_accel_st->FT_X;
    mpu6050_accel_st->Y_ERROR = (mpu6050_accel_st->STR_Y - mpu6050_accel_st->FT_Y) / mpu6050_accel_st->FT_Y;
    mpu6050_accel_st->Z_ERROR = (mpu6050_accel_st->STR_Z - mpu6050_accel_st->FT_Z) / mpu6050_accel_st->FT_Z;
    
    mpu_setresolution(gyro_res_mem, accel_res_mem, mpu6050);// After self test come back to old config values and disable self test mode 
    return true;
}

bool mpu_gyro_st(MPU6050* mpu6050, MPU6050_SELFTEST* mpu6050_gyro_st)
{
    uint8_t gyro_res_mem = mpu6050->mpu6050_raw.gyro_res;
    uint8_t accel_res_mem = mpu6050->mpu6050_raw.accel_res;
    uint8_t mask = 0b00011111;

    mpu_read_raw(mpu6050); // read data before selF test
    uint8_t gyro_x = mpu6050->mpu6050_raw.acceleration[0];
    uint8_t gyro_y = mpu6050->mpu6050_raw.acceleration[1];
    uint8_t gyro_z = mpu6050->mpu6050_raw.acceleration[2];

    i2c_write_reg(mpu6050_reg.address, mpu6050_reg.gyro_config, 0b11100000); // enable selft test | set +-250*/s

    i2c_write_blocking(i2c1, mpu6050_reg.address, &mpu6050_reg.XA_TEST, 1, true); 
    i2c_read_blocking(i2c1, mpu6050_reg.address, &mpu6050_gyro_st->X_TEST, 1, false);
    
    i2c_write_blocking(i2c1, mpu6050_reg.address, &mpu6050_reg.YA_TEST, 1, true); 
    i2c_read_blocking(i2c1, mpu6050_reg.address, &mpu6050_gyro_st->Y_TEST, 1, false);
    
    i2c_write_blocking(i2c1, mpu6050_reg.address, &mpu6050_reg.ZA_TEST, 1, true); 
    i2c_read_blocking(i2c1, mpu6050_reg.address, &mpu6050_gyro_st->Z_TEST, 1, false);
    
    mpu6050_gyro_st->X_TEST = (mpu6050_gyro_st->X_TEST & mask);
    mpu6050_gyro_st->Y_TEST = (mpu6050_gyro_st->Y_TEST & mask);
    mpu6050_gyro_st->Z_TEST = (mpu6050_gyro_st->Z_TEST & mask);
    
    mpu6050_gyro_st->FT_X = 25 * 131 * pow(1.046, (mpu6050_gyro_st->X_TEST - 1.0));
    mpu6050_gyro_st->FT_Y = -25 * 131 * pow(1.046, (mpu6050_gyro_st->Y_TEST - 1.0));
    mpu6050_gyro_st->FT_Z = 25 * 131 * pow(1.046, (mpu6050_gyro_st->Z_TEST - 1.0));

    mpu_read_raw(mpu6050); // read data while self test is enabled
    mpu6050_gyro_st->STR_X = mpu6050->mpu6050_raw.acceleration[0] - gyro_x;
    mpu6050_gyro_st->STR_Y = mpu6050->mpu6050_raw.acceleration[1] - gyro_y;
    mpu6050_gyro_st->STR_Z = mpu6050->mpu6050_raw.acceleration[2] - gyro_z;

    mpu6050_gyro_st->X_ERROR = (mpu6050_gyro_st->STR_X - mpu6050_gyro_st->FT_X) / mpu6050_gyro_st->FT_X;
    mpu6050_gyro_st->Y_ERROR = (mpu6050_gyro_st->STR_Y - mpu6050_gyro_st->FT_Y) / mpu6050_gyro_st->FT_Y;
    mpu6050_gyro_st->Z_ERROR = (mpu6050_gyro_st->STR_Z - mpu6050_gyro_st->FT_Z) / mpu6050_gyro_st->FT_Z;
    
    mpu_setresolution(gyro_res_mem, accel_res_mem, mpu6050);// After self test come back to old config values and disable self test mode 
    return true;
}

void mpu_convert(MPU6050* mpu6050)
{
    switch(mpu6050->mpu6050_raw.accel_res)
    {
        case 0:
            mpu6050->mpu6050_raw.accel_convert[0] = mpu6050->mpu6050_raw.acceleration[0] / 16384.0;
            mpu6050->mpu6050_raw.accel_convert[1] = mpu6050->mpu6050_raw.acceleration[1] / 16384.0;
            mpu6050->mpu6050_raw.accel_convert[2] = mpu6050->mpu6050_raw.acceleration[2] / 16384.0;
        break;

        case 1:
            mpu6050->mpu6050_raw.accel_convert[0] = mpu6050->mpu6050_raw.acceleration[0] / 8192.0;
            mpu6050->mpu6050_raw.accel_convert[1] = mpu6050->mpu6050_raw.acceleration[1] / 8192.0;
            mpu6050->mpu6050_raw.accel_convert[2] = mpu6050->mpu6050_raw.acceleration[2] / 8192.0;
        break;

        case 2:
            mpu6050->mpu6050_raw.accel_convert[0] = mpu6050->mpu6050_raw.acceleration[0] / 4096.0;
            mpu6050->mpu6050_raw.accel_convert[1] = mpu6050->mpu6050_raw.acceleration[1] / 4096.0;
            mpu6050->mpu6050_raw.accel_convert[2] = mpu6050->mpu6050_raw.acceleration[2] / 4096.0;
        break;

        case 3:
            mpu6050->mpu6050_raw.accel_convert[0] = mpu6050->mpu6050_raw.acceleration[0] / 2048.0;
            mpu6050->mpu6050_raw.accel_convert[1] = mpu6050->mpu6050_raw.acceleration[1] / 2048.0;
            mpu6050->mpu6050_raw.accel_convert[2] = mpu6050->mpu6050_raw.acceleration[2] / 2048.0;
        break;
    
    }

    switch(mpu6050->mpu6050_raw.gyro_res)
    {
        case 0:
            mpu6050->mpu6050_raw.gyro_convert[0] = mpu6050->mpu6050_raw.gyro[0] / 131.0;
            mpu6050->mpu6050_raw.gyro_convert[1] = mpu6050->mpu6050_raw.gyro[1] / 131.0;
            mpu6050->mpu6050_raw.gyro_convert[2] = mpu6050->mpu6050_raw.gyro[2] / 131.0;
        break;

        case 1:
            mpu6050->mpu6050_raw.gyro_convert[0] = mpu6050->mpu6050_raw.gyro[0] / 65.5;
            mpu6050->mpu6050_raw.gyro_convert[1] = mpu6050->mpu6050_raw.gyro[1] / 65.5;
            mpu6050->mpu6050_raw.gyro_convert[2] = mpu6050->mpu6050_raw.gyro[2] / 65.5;
        break;

        case 2:
            mpu6050->mpu6050_raw.gyro_convert[0] = mpu6050->mpu6050_raw.gyro[0] / 32.8;
            mpu6050->mpu6050_raw.gyro_convert[1] = mpu6050->mpu6050_raw.gyro[1] / 32.8;
            mpu6050->mpu6050_raw.gyro_convert[2] = mpu6050->mpu6050_raw.gyro[2] / 32.8;
        break;

        case 3:
            mpu6050->mpu6050_raw.gyro_convert[0] = mpu6050->mpu6050_raw.gyro[0] / 16.4;
            mpu6050->mpu6050_raw.gyro_convert[1] = mpu6050->mpu6050_raw.gyro[1] / 16.4;
            mpu6050->mpu6050_raw.gyro_convert[2] = mpu6050->mpu6050_raw.gyro[2] / 16.4;
        break;
    }

}

void mpu_set_sample_rate(uint8_t divider)
{
    i2c_write_reg(mpu6050_reg.address, mpu6050_reg.config, 0b00000110); // => set DLPF as 1kHz

    switch(divider)
    {
        case 1:
            i2c_write_reg(mpu6050_reg.address, mpu6050_reg.SMPLRT_DIV, 0b00000001);
        break;

        case 2:
            i2c_write_reg(mpu6050_reg.address, mpu6050_reg.SMPLRT_DIV, 0b00000010);
        break;

        case 4:
            i2c_write_reg(mpu6050_reg.address, mpu6050_reg.SMPLRT_DIV, 0b00000100);
        break;

        case 8:
            i2c_write_reg(mpu6050_reg.address, mpu6050_reg.SMPLRT_DIV, 0b00001000);
        break;

        case 16:
            i2c_write_reg(mpu6050_reg.address, mpu6050_reg.SMPLRT_DIV, 0b00010000);
        break;
        
        case 32:
            i2c_write_reg(mpu6050_reg.address, mpu6050_reg.SMPLRT_DIV, 0b00100000);
        break;

        case 64:
            i2c_write_reg(mpu6050_reg.address, mpu6050_reg.SMPLRT_DIV, 0b01000000);
        break;

        case 128:
            i2c_write_reg(mpu6050_reg.address, mpu6050_reg.SMPLRT_DIV, 0b10000000);
        break;
    }  
}

void mpu_statistic(MPU6050* mpu6050)
{
    float acc_x[244], acc_y[244], acc_z[244];
    float gyro_x[244], gyro_y[244], gyro_z[244];
    float var_acc_x, var_acc_y, var_acc_z, var_gyro_x, var_gyro_y, var_gyro_z;
    float sigma_acc_x, sigma_acc_y, sigma_acc_z, sigma_gyro_x, sigma_gyro_y, sgima_gyro_z;


    for(uint8_t i; i < 244; i++) // measure output 50 times
    {
        mpu_read_raw(mpu6050);
        acc_x[i] = mpu6050->mpu6050_raw.accel_convert[0]; acc_y[i] = mpu6050->mpu6050_raw.accel_convert[1]; acc_z[i] = mpu6050->mpu6050_raw.accel_convert[2];
        gyro_x[i] = mpu6050->mpu6050_raw.gyro_convert[0];  gyro_y[i] = mpu6050->mpu6050_raw.gyro_convert[1];  gyro_z[i] = mpu6050->mpu6050_raw.gyro_convert[2];
    }

    var_acc_x = get_variance(acc_x, 244); var_acc_y = get_variance(acc_y, 244); var_acc_z = get_variance(acc_z, 244);
    var_gyro_x = get_variance(gyro_x, 244); var_gyro_y= get_variance(gyro_y, 244); var_gyro_z = get_variance(gyro_z, 244);

    //printf("%f,%f,%f\n", sqrt(var_acc_x), sqrt(var_acc_y), sqrt(var_acc_z));
    //printf("%f,%f,%f\n", sqrt(var_gyro_x), sqrt(var_gyro_y), sqrt(var_gyro_z));
}

void mpu_read(MPU6050* mpu6050)
{
    mpu_read_raw(mpu6050);
    mpu6050->mpu6050_data.accel[0] = mpu6050->mpu6050_raw.accel_convert[0] - mpu6050->mpu6050_raw.accel_x_offset;
    mpu6050->mpu6050_data.accel[1] = mpu6050->mpu6050_raw.accel_convert[1] - mpu6050->mpu6050_raw.accel_y_offset;
    mpu6050->mpu6050_data.accel[2] = mpu6050->mpu6050_raw.accel_convert[2] - mpu6050->mpu6050_raw.accel_z_offset; 

    mpu6050->mpu6050_data.gyro[0] = mpu6050->mpu6050_raw.gyro_convert[0] - mpu6050->mpu6050_raw.gyro_x_offset;
    mpu6050->mpu6050_data.gyro[1] = mpu6050->mpu6050_raw.gyro_convert[1] - mpu6050->mpu6050_raw.gyro_y_offset;
    mpu6050->mpu6050_data.gyro[2] = mpu6050->mpu6050_raw.gyro_convert[2] - mpu6050->mpu6050_raw.gyro_z_offset;
    
    mpu_remove_gravity(mpu6050);
    Ring_buffer_push(&mpu6050->mpu6050_data.accelbuffer, mpu6050->mpu6050_data.accel[0]); //TO DO !!!
    Ring_buffer_push(&mpu6050->mpu6050_data.gyrobuffer, mpu6050->mpu6050_data.gyro[0]); 
}

void mpu_get_offset(MPU6050* mpu6050)
{
    for(uint8_t i = 0; i < 244; i++)
    {
        mpu_read_raw(mpu6050);
        mpu6050->mpu6050_raw.accel_x_offset += mpu6050->mpu6050_raw.accel_convert[0]; 
        mpu6050->mpu6050_raw.accel_x_offset += mpu6050->mpu6050_raw.accel_convert[1];
        mpu6050->mpu6050_raw.accel_x_offset += mpu6050->mpu6050_raw.accel_convert[2] - 1.0;

        mpu6050->mpu6050_raw.gyro_x_offset += mpu6050->mpu6050_raw.gyro_convert[0];
        mpu6050->mpu6050_raw.gyro_x_offset += mpu6050->mpu6050_raw.gyro_convert[1];
        mpu6050->mpu6050_raw.gyro_x_offset += mpu6050->mpu6050_raw.gyro_convert[2];
    }

    mpu6050->mpu6050_raw.accel_x_offset /= 250.0f;  mpu6050->mpu6050_raw.accel_y_offset /= 250.0f;  mpu6050->mpu6050_raw.accel_z_offset /= 250.0f;
    mpu6050->mpu6050_raw.gyro_x_offset /= 250.0f; mpu6050->mpu6050_raw.gyro_y_offset /= 250.0f; mpu6050->mpu6050_raw.gyro_z_offset /= 250.0f;
}

void mpu_remove_gravity(MPU6050* mpu6050)
{
    static float gravity[3] = {0, 0, 0};
    float alpha = 0.8;

    gravity[0] = alpha * gravity[0] + (1 - alpha) * mpu6050->mpu6050_data.accel[0];
    gravity[1] = alpha * gravity[1] + (1 - alpha) * mpu6050->mpu6050_data.accel[1];
    gravity[2] = alpha * gravity[2] + (1 - alpha) * mpu6050->mpu6050_data.accel[2];

    mpu6050->mpu6050_data.accelwithoutgravity[0] = mpu6050->mpu6050_data.accel[0] - gravity[0];
    mpu6050->mpu6050_data.accelwithoutgravity[1] = mpu6050->mpu6050_data.accel[1] - gravity[1];
    mpu6050->mpu6050_data.accelwithoutgravity[2] = mpu6050->mpu6050_data.accel[2] - gravity[2]; 
}

void mpu_get_distance(MPU6050* mpu6050)
{
    //mpu_read(mpu6050_raw, mpu6050);
    //float velocity = fabs(mpu6050->accelwithoutgravity[0] * 9.81) * 0.001;
    //mpu6050->distance += velocity * 0.001;

    //mpu6050->distance += 0.5 * fabs(mpu6050->accelwithoutgravity[0] * 9.81) * 0.0001;

    //printf("%f\n", mpu6050->distance);
   
}

bool mpu_callback(struct repeating_timer *timer)
{
    MPU6050* mpu6050 = (MPU6050*)timer->user_data;   // void* casting 
    mpu_read(mpu6050);
}

float get_variance(float* data, uint8_t data_size)
{
    float variance;
    float sum = 0;
    float mean;
    float x[data_size];

    for(uint8_t i = 0; i < data_size; i++)
        sum = sum + data[i];
        
    mean = sum / data_size;
    sum = 0;

    for(uint8_t i = 0; i < data_size; i++)
        x[i] = data[i] - mean;

    for(uint8_t i = 0; i < data_size; i++)
        sum = sum + pow(x[i], 2);
    
    variance = sum / data_size;

    return variance;
}