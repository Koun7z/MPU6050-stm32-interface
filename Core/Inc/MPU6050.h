/*
 * MPU6050.h
 *
 *  Created on: Jan 17, 2025
 *      Author: pwoli
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

// Platform dependent include
// Delete this one and use your own if needed
#include "stm32f4xx_hal.h"  // <- F411 blackpill used for testing

#include <stdbool.h>


/**
 * @brief Data structure holding MPU6050 readings
 */
struct IMU_Data
{
	float AccelX;
	float AccelY;
	float AccelZ;

	float GyroX;
	float GyroY;
	float GyroZ;

	// TODO: Orientaton calculation
	float PitchAngle;
	float RollAngle;

	float Temp;
};


/**
 * @brief  	    Initialise comunication and basic parameters of the MPU6050 sensor
 * @params[in] *i2c   i2c handle
 * @retval      bool
 * 				- true  Initialization succesful
 * 				- flase Initialization failed
 */
bool MPU_Init(I2C_HandleTypeDef* hi2c);

/**
 * @brief       Parse data after reading using DMA
 * @param[in]  *hi2c  i2c handle
 * @param[out] *data  IMU data structure
 */
void MPU_HandleRX(I2C_HandleTypeDef* hi2c, struct IMU_Data* data);

/**
 * @brief  	    Read the gyroscope data into passed struct
 * @param[out]  MPU_Data  IMU data structure
 * @retval      HAL_StatusTypeDef
 * 				- HAL_OK      = 0x00U
 * 	  	  	  	- HAL_ERROR   = 0x01U
 *				- HAL_BUSY    = 0x02U
 *			 	- HAL_TIMEOUT = 0x03U
 */
HAL_StatusTypeDef MPU_ReadGyroData(struct IMU_Data* data);

/**
 * @brief  		Read the accelerometer data into passed struct
 * @param[out]  MPU_Data  IMU data structure
 * @retval      HAL_StatusTypeDef
 * 				- HAL_OK      = 0x00U
 * 	  	  	  	- HAL_ERROR   = 0x01U
 *				- HAL_BUSY    = 0x02U
 *			 	- HAL_TIMEOUT = 0x03U
 */
HAL_StatusTypeDef MPU_ReadAccelData(struct IMU_Data* data);

/**
 * @brief  	    Read the temperature data into passed struct
 * @param[out]  MPU_Data  IMU data structure
 * @retval      HAL_StatusTypeDef
 * 				- HAL_OK      = 0x00U
 * 	  	  	  	- HAL_ERROR   = 0x01U
 *				- HAL_BUSY    = 0x02U
 *			 	- HAL_TIMEOUT = 0x03U
 */
HAL_StatusTypeDef MPU_ReadTempData(struct IMU_Data* data);

/**
 * @brief       Read all IMU data into passed struct
 * @param[out]  MPU_Data  IMU data structure
 * @retval      HAL_StatusTypeDef
 * 				- HAL_OK      = 0x00U
 * 	  	  	  	- HAL_ERROR   = 0x01U
 *				- HAL_BUSY    = 0x02U
 *			 	- HAL_TIMEOUT = 0x03U
 */
HAL_StatusTypeDef MPU_ReadAll(struct IMU_Data* data);

/**
 * @brief   Start reading all data using DMA
 * @retval  HAL_StatusTypeDef
 * 		    - HAL_OK      = 0x00U
 * 	  	  	- HAL_ERROR   = 0x01U
 *		    - HAL_BUSY    = 0x02U
 *			- HAL_TIMEOUT = 0x03U
 */
HAL_StatusTypeDef MPU_RequestAllDMA();

/**
 * @brief      Calculates offset from zero by averaging the readings for given time
 * 			   and applies it to all future readings.
 * @param[in]  t  calibration time
 */
void MPU_CallibrateGyro(uint32_t t);

/**
 * @brief      Calculates offset from level position by averaging the readings for given time
 * 			   and applies it to all future readings.
 * @param[in]  t  calibration time
 */
void MPU_CallibrateAccel(uint32_t t);

#endif /* INC_MPU6050_H_ */
