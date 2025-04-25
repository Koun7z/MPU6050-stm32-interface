//
// Created by pwoli on 20.03.2025.
//

#ifndef MPU_CONFIG_H
#define MPU_CONFIG_H

// Measurement range of accelerometer (2, 4, 8 or 16 g)
#define ACCEL_RANGE 8
// Measurement range of gyroscope (250, 500, 500, 1000 or 2000 deg/s)
#define GYRO_RANGE 1000

/*
** Accelerometer data unit:
** 0 - [m/s^2]
** 1 - [g]
*/
#define ACCEL_G_OUTPUT 1

/*
** Gyroscope data unit:
** 0 -> [deg/s]
** 1 -> [rad/s]
*/
#define GYRO_RAD_OUTPUT 0

// Axis invertion
#define INVERT_ACCEL_X_AXIS 1
#define INVERT_ACCEL_Y_AXIS 1
#define INVERT_ACCEL_Z_AXIS 1

#define INVERT_GYRO_X_AXIS 0
#define INVERT_GYRO_Y_AXIS 0
#define INVERT_GYRO_Z_AXIS 0

// Static zero reference offset for the gyroscope
#define GYRO_X_STATIC_CALIBRATION 0.0f
#define GYRO_Y_STATIC_CALIBRATION 0.0f
#define GYRO_Z_STATIC_CALIBRATION 0.0f

// Static zero reference offset for the acceletometer
#define ACCEL_X_STATIC_CALIBRATION 0.0f
#define ACCEL_Y_STATIC_CALIBRATION 0.0f
#define ACCEL_Z_STATIC_CALIBRATION 0.0f

// Static zero reference offset for the temerature sensor
#define TEMPERATURE_STATIC_CALIBRATION 0

// I2C timeout when communicating in blocking mode
#define INIT_TIMEOUT 1000

#endif //MPU_CONFIG_H
