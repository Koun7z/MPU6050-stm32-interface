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
typedef struct
{
	float AccelX;
	float AccelY;
	float AccelZ;

	float GyroX;
	float GyroY;
	float GyroZ;

	float Temp;
} MPU_Data_Instance;


/**
 * @brief  	    Initialise comunication and basic parameters of the MPU6050 sensor
 * @params[in] *i2c      i2c handle
 * @retval      bool
 * 				- true   Initialization succesful
 * 				- flase  Initialization failed
 */
bool MPU_Init(I2C_HandleTypeDef* hi2c);

/**
 * @brief       Parse data after reading using DMA
 * @param[in]  *hi2c  i2c handle
 * @param[out] *data  IMU data structure
 */
void MPU_HandleRX(const I2C_HandleTypeDef* hi2c, MPU_Data_Instance* data);

/**
 * @brief  	    Read the gyroscope data into passed struct
 * @param[out]  MPU_Data  IMU data structure
 * @retval      HAL_StatusTypeDef
 * 				- HAL_OK      = 0x00U
 * 	  	  	  	- HAL_ERROR   = 0x01U
 *				- HAL_BUSY    = 0x02U
 *			 	- HAL_TIMEOUT = 0x03U
 */
HAL_StatusTypeDef MPU_ReadGyroData(MPU_Data_Instance* data);

/**
 * @brief  		Read the accelerometer data into passed struct
 * @param[out]  MPU_Data  IMU data structure
 * @retval      HAL_StatusTypeDef
 * 				- HAL_OK      = 0x00U
 * 	  	  	  	- HAL_ERROR   = 0x01U
 *				- HAL_BUSY    = 0x02U
 *			 	- HAL_TIMEOUT = 0x03U
 */
HAL_StatusTypeDef MPU_ReadAccelData(MPU_Data_Instance* data);

/**
 * @brief  	    Read the temperature data into passed struct
 * @param[out]  MPU_Data  IMU data structure
 * @retval      HAL_StatusTypeDef
 * 				- HAL_OK      = 0x00U
 * 	  	  	  	- HAL_ERROR   = 0x01U
 *				- HAL_BUSY    = 0x02U
 *			 	- HAL_TIMEOUT = 0x03U
 */
HAL_StatusTypeDef MPU_ReadTempData(MPU_Data_Instance* data);

/**
 * @brief       Read all IMU data into passed struct
 * @param[out]  MPU_Data  IMU data structure
 * @retval      HAL_StatusTypeDef
 * 				- HAL_OK      = 0x00U
 * 	  	  	  	- HAL_ERROR   = 0x01U
 *				- HAL_BUSY    = 0x02U
 *			 	- HAL_TIMEOUT = 0x03U
 */
HAL_StatusTypeDef MPU_ReadAll(MPU_Data_Instance* data);

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
void MPU_CalibrateGyro(uint32_t t);

/**
 * @brief      Calculates offset from level position by averaging the readings for given time
 * 			   and applies it to all future readings.
 * @param[in]  t  calibration time
 */
void MPU_CalibrateAccel(uint32_t t);

#endif /* INC_MPU6050_H_ */
