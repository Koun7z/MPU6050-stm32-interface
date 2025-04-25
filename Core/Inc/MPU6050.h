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

#define MPU_ADDRESS_1 0xD0
#define MPU_ADDRESS_2 0xD2

/**
 * @brief Data structure holding MPU6050 confing and readings
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

	float GyroOffset[3];
	float AccelOffset[3];

	uint8_t MPU_Address;
	I2C_HandleTypeDef* hi2c;
} MPU_Instance;


/**
 * @brief  	    Initialise comunication and basic parameters of the MPU6050 sensor
 * @params[in] *i2c      i2c handle
 * @retval      bool
 * 				- true   Initialization succesful
 * 				- flase  Initialization failed
 */
bool MPU_Init(MPU_Instance* mpu, const I2C_HandleTypeDef* hi2c, uint8_t address);

/**
 * @brief       Parse data after reading using DMA
 * @param[in]  *hi2c  i2c handle
 * @param[out] *data  IMU data structure
 */
void MPU_HandleRX(MPU_Instance* mpu);

/**
 * @brief  	    Read the gyroscope data into passed struct
 * @param[out]  MPU_Data  IMU data structure
 * @retval      HAL_StatusTypeDef
 * 				- HAL_OK      = 0x00U
 * 	  	  	  	- HAL_ERROR   = 0x01U
 *				- HAL_BUSY    = 0x02U
 *			 	- HAL_TIMEOUT = 0x03U
 */
HAL_StatusTypeDef MPU_ReadGyroData(MPU_Instance* mpu);

/**
 * @brief  		Read the accelerometer data into passed struct
 * @param[out]  MPU_Data  IMU data structure
 * @retval      HAL_StatusTypeDef
 * 				- HAL_OK      = 0x00U
 * 	  	  	  	- HAL_ERROR   = 0x01U
 *				- HAL_BUSY    = 0x02U
 *			 	- HAL_TIMEOUT = 0x03U
 */
HAL_StatusTypeDef MPU_ReadAccelData(MPU_Instance* mpu);

/**
 * @brief  	    Read the temperature data into passed struct
 * @param[out]  MPU_Data  IMU data structure
 * @retval      HAL_StatusTypeDef
 * 				- HAL_OK      = 0x00U
 * 	  	  	  	- HAL_ERROR   = 0x01U
 *				- HAL_BUSY    = 0x02U
 *			 	- HAL_TIMEOUT = 0x03U
 */
HAL_StatusTypeDef MPU_ReadTempData(MPU_Instance* mpu);

/**
 * @brief       Read all IMU data into passed struct
 * @param[out]  MPU_Data  IMU data structure
 * @retval      HAL_StatusTypeDef
 * 				- HAL_OK      = 0x00U
 * 	  	  	  	- HAL_ERROR   = 0x01U
 *				- HAL_BUSY    = 0x02U
 *			 	- HAL_TIMEOUT = 0x03U
 */
HAL_StatusTypeDef MPU_ReadAll(MPU_Instance* mpu);

/**
 * @brief   Start reading all data using DMA
 * @retval  HAL_StatusTypeDef
 * 		    - HAL_OK      = 0x00U
 * 	  	  	- HAL_ERROR   = 0x01U
 *		    - HAL_BUSY    = 0x02U
 *			- HAL_TIMEOUT = 0x03U
 */
HAL_StatusTypeDef MPU_RequestAllDMA(const MPU_Instance* mpu);

/**
 * @brief      Calculates offset from zero by averaging the readings for given time
 * 			   and applies it to all future readings.
 * @param[in]  t  calibration time
 */
void MPU_CalibrateGyro(MPU_Instance* mpu, uint32_t t);

/**
 * @brief      Calculates offset from level position by averaging the readings for given time
 * 			   and applies it to all future readings.
 * @param[in]  t  calibration time
 */
void MPU_CalibrateAccel(MPU_Instance* mpu, uint32_t t);

#endif /* INC_MPU6050_H_ */
