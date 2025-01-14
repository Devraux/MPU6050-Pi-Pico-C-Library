#include "main.h"
//#include "mylib/mylib.h"
#include <pico/double.h>
int main()
{
    stdio_init_all();
    sleep_ms(5000);
    MPU6050_Data mpu6050 = {0};
    mpu_init(&mpu6050);


    while(1)
    {
        mpu_read_raw(&mpu6050);
        printf("GX: %d GY:%d GZ:%d\n", mpu6050.gyro_no_offset[0], mpu6050.gyro_no_offset[1], mpu6050.gyro_no_offset[2]);
        //printf("%f,%f,%f\n", mpu6050.mpu6050_data.accel_no_gravity[0], mpu6050.mpu6050_data.accel_no_gravity[1], mpu6050.mpu6050_data.accel_no_gravity[2]);
        //printf("%f\n", mpu6050.mpu6050_data.distance);
        //buffer_print(&mpu6050.mpu6050_data.accelbuffer);
        //printf("%f,%f,%f\n",mpu6050.mpu6050_data.accel_convert[0], mpu6050.mpu6050_data.accel_convert[1], mpu6050.mpu6050_data.accel_convert[2]);
        //printf("%f,%f,%f\n", mpu6050.mpu6050_data.theta_pitch, mpu6050.mpu6050_data.theta_roll, mpu6050.mpu6050_data.theta_yaw);
        //printf("%f\n", mpu6050.mpu6050_data.distance);
        //printf("%f\n",mpu6050.mpu6050_data.gyro_convert[0]);
        sleep_ms(115);
    }
    return 0;
}