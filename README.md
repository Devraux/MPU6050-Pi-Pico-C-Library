# MPU6050_MotionAnlyzer

![Visitors](https://visitor-badge.laobi.icu/badge?page_id=Devraux.MPU6050-Pi-Pico-C-Library)

This application is a simple IMU (Inertial Measurement Unit) based on MPU6050 and Raspberry Pi Pico, which allows you to:
* Measure raw linear acceleration
* Measure raw angular velocity
* Eliminate offset from readings (from both accelerometer and gyroscope)
* Remove gravitational acceleration from accelerometer readings
* Implement a compass (using gyroscope readings compensated with accelerometer data)
* Statistical functions like standard deviation and variance
* Configure MPU6050 settings such as sample rate, output filters, and more
* Configure MPU6050 and Raspberry Pi Pico connection using I2C, and set up interrupts

# Todo:
- [ ] Kalman Filter implementation
- [ ] Implement FIFO and Pi Pico DMA connection
- [ ] Code and device Unit Tests 
- [x] Improve accuracy
- [x] Code Refactoring

> [!CAUTION]
This project is still under development and is not fully finished. Therefore, there may be some errors in the code that are being systematically fixed. Thank you for your patience.