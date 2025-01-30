/*
 * MPU6050.h
 *
 *  Created on: Jan 17, 2025
 *      Author: pwoli
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#ifdef STM32L476xx
#  include "stm32l4xx_hal.h"
#else
#  ifdef STM32F411xE
#	include "stm32f4xx_hal.h"
#  endif
#endif

#include <stdbool.h>

struct IMU_Data
{
	float AccelX;
	float AccelY;
	float AccelZ;

	float GyroX;
	float GyroY;
	float GyroZ;

	float PitchAngle;
	float RollAngle;

	float Temp;
};

/**
 * @brief  Initialise comunication and basic parameters of the MPU6050 sensor
 * @param  i2c handle
 * @retval bool - Initialization succesful
 */
bool MPU_Init(I2C_HandleTypeDef* hi2c);

/**
 * @brief  Parse data after reading using DMA
 * @param[in]  hi2c - i2c handle
 * @param[out] data -
 */
void MPU_HandleRX(I2C_HandleTypeDef* hi2c, struct IMU_Data* data);

/**
 * @brief  Read the gyroscope data into passed struct
 * @param  MPU_Data IMU data structure
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef MPU_ReadGyroData(struct IMU_Data* data);

/**
 * @brief  Read the accelerometer data into passed struct
 * @param  MPU_Data IMU data structure
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef MPU_ReadAccelData(struct IMU_Data* data);

/**
 * @brief  Read the temperature data into passed struct
 * @param  MPU_Data IMU data structure
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef MPU_ReadTempData(struct IMU_Data* data);

/**
 * @brief  Read all IMU data into passed struct
 * @param  MPU_Data IMU data structure
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef MPU_ReadAll(struct IMU_Data* data);

/**
 * @brief  Start reading all data using DMA
 * @param  MPU_Data IMU data structure
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef MPU_ReadAllDMA();

float* MPU_CallibrateGyro(uint32_t t);
float* MPU_CallibrateAccel(uint32_t t);

#endif /* INC_MPU6050_H_ */
