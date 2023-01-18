# Calibrated-IMU
Calibration of Inertial Measurement Unit on all axis using MPU6050

The file named `IMU_Calibrated.ino` contains calculations for determining filtered angles with the help of raw Accelerometer and Gyroscope data.
Web Server source code for ESP32 is labelled as `IMU_connect.ino`.
For the general overview of how IMU is calibrated, `IMU_328P.ino` contains the port of ATMega328P.
Also, The file named `Fast_inverse_square_root.c` contains the calculaton of Fast inverse square root by the hexadecimal constant 0x5F3759DF through an algorithm that estimates 1/√x, the reciprocal (or multiplicative inverse) of the square root of a 32-bit floating-point number x in IEEE 754 floating-point format. `Summation_eqn.c` shows a normal approach of solving algebraic summation equation (outputs a lesser result than practical calculation because of operator size)