/*
 * MPU6050.h
 *
 *  Created on: Jan 17, 2025
 *      Author: pwoli
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include <stdbool.h>
#include "stm32l4xx_hal.h"

#define MPU_ADDRESS 0xD0

// I2C timeout when comunicatin in blocking mode
#define INIT_TIMEOUT 1000

//Measurement range of accelerometer (2, 4, 8 or 16 g)
#define ACCEL_RANGE 8
//Measurement range of gyroscope (250, 500, 500, 1000 or 2000 deg/s)
#define GYRO_RANGE  1000

// Static zero reference offset for the gyroscope
#define GYRO_X_STATIC_CALIBRATION 5.2228
#define GYRO_Y_STATIC_CALIBRATION 0.5760
#define GYRO_Z_STATIC_CALIBRATION -2.2012

// Static zero reference offset for the Acceletometer
#define ACCEL_X_STATIC_CALIBRATION 0.0247
#define ACCEL_Y_STATIC_CALIBRATION -0.0092
#define ACCEL_Z_STATIC_CALIBRATION -0.0411

// Static zero reference offset for the temerature sensor
#define TEMPERATURE_STATIC_CALIBRATION 0

struct MPU_Data
{
	float AccelX;
	float AccelY;
	float AccelZ;

	float GyroX;
	float GyroY;
	float GyroZ;

	float Temp;
};

extern struct MPU_Data MPU_Data;

/**
 * @brief  Initialise comunication and basic parameters of the MPU6050 sensor
 * @param  i2c handle
 * @retval bool - Initialization succesful
 */
bool MPU_Init(I2C_HandleTypeDef* hi2c);

/**
 * @brief  Parse data after reading using DMA
 * @param  i2c handle
 */
void MPU_HandleRX(I2C_HandleTypeDef* hi2c);

/**
 * @brief  Read the gyroscope data into passed struct
 * @param  MPU_Data IMU data structure
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef MPU_ReadGyroData(struct MPU_Data* data);

/**
 * @brief  Read the accelerometer data into passed struct
 * @param  MPU_Data IMU data structure
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef MPU_ReadAccelData(struct MPU_Data* data);

/**
 * @brief  Read the temperature data into passed struct
 * @param  MPU_Data IMU data structure
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef MPU_ReadTempData(struct MPU_Data* data);

/**
 * @brief  Read all IMU data into passed struct
 * @param  MPU_Data IMU data structure
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef MPU_ReadAll(struct MPU_Data* data);

/**
 * @brief  Start reading all data using DMA
 * @param  MPU_Data IMU data structure
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef MPU_ReadAllDMA();

#endif /* INC_MPU6050_H_ */
