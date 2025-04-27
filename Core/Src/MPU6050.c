/*
 * MPU6050.c
 *
 *  Created on: Jan 17, 2025
 *      Author: pwoli
 */

#include "MPU6050.h"

#include "MPU_Config.h"

#define WHO_AM_I     0x75
#define PWR_MGMT_1   0x6B
#define CONFIG       0x1A
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0x1C
#define SMPRT_DIV    0x19
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43
#define TEMP_OUT_H   0x41

#define MPU_ADC_RES 65536.0f

#if GYRO_RAD_OUTPUT
#  define GYRO_UNIT_CONST 0.01745329251994f
#else
#  define GYRO_UNIT_CONST 1.0f
#endif

#if ACCEL_G_OUTPUT
#  define ACCEL_UNIT_CONST 1.0f
#else
#  define ACCEL_UINT_CONST 9.80665f
#endif


static uint8_t rxBuff[14];

static bool dataRequested = false;

// TODO: Add proper runtime configuration
bool MPU_Init(MPU_Instance* mpu, const I2C_HandleTypeDef* hi2c, const uint8_t address)
{
	mpu->GyroOffset[0] = 0.0f;
	mpu->GyroOffset[1] = 0.0f;
	mpu->GyroOffset[2] = 0.0f;

	mpu->AccelOffset[0] = 0.0f;
	mpu->AccelOffset[1] = 0.0f;
	mpu->AccelOffset[2] = 0.0f;


	mpu->hi2c        = (I2C_HandleTypeDef*)hi2c;
	mpu->MPU_Address = address;

	uint8_t id;
	uint8_t data;

	HAL_I2C_Mem_Read(mpu->hi2c, mpu->MPU_Address, WHO_AM_I, 1, &id, 1, INIT_TIMEOUT);

	if(id == 104)
	{
		// Power mode
		data = 0;
		if(HAL_I2C_Mem_Write(mpu->hi2c, mpu->MPU_Address, PWR_MGMT_1, 1, &data, 1, INIT_TIMEOUT))
		{
			return false;
		}

		// Clock divider
		data = 0;
		if(HAL_I2C_Mem_Write(mpu->hi2c, mpu->MPU_Address, SMPRT_DIV, 1, &data, 1, INIT_TIMEOUT))
		{
			return false;
		}

		// External sync + lowpass filter
		data = 0;
		if(HAL_I2C_Mem_Write(mpu->hi2c, mpu->MPU_Address, CONFIG, 1, &data, 1, INIT_TIMEOUT))
		{
			return false;
		}

		// Gyro range
#if GYRO_RANGE == 250
		data = 0x0;
#elif GYRO_RANGE == 500
		data = 0x8;
#elif GYRO_RANGE == 1000
		data = 0x10;
#elif GYRO_RANGE == 2000
		data = 0x18;
#else
#  error "Incorrect gyroscope range value"
#endif
		if(HAL_I2C_Mem_Write(mpu->hi2c, mpu->MPU_Address, GYRO_CONFIG, 1, &data, 1, INIT_TIMEOUT))
		{
			return false;
		}

		// Accel Range
#if ACCEL_RANGE == 2
		data = 0x0;
#elif ACCEL_RANGE == 4
		data = 0x8;
#elif ACCEL_RANGE == 8
		data = 0x10;
#elif ACCEL_RANGE == 16
		data = 0x18;
#else
#  error "Incorrect accelerometer range value"
#endif
		if(HAL_I2C_Mem_Write(mpu->hi2c, mpu->MPU_Address, ACCEL_CONFIG, 1, &data, 1, INIT_TIMEOUT))
		{
			return false;
		}

		return true;
	}

	return false;
}

void MPU_HandleRX(MPU_Instance* mpu)
{
	dataRequested = false;

	mpu->AccelX = (int16_t)(rxBuff[0] << 8 | rxBuff[1]) / (MPU_ADC_RES / ACCEL_RANGE / 2.0f) * ACCEL_UNIT_CONST
	              * ACCEL_X_AXIS_DIRECTION
	            - ACCEL_X_STATIC_CALIBRATION - mpu->AccelOffset[0];
	mpu->AccelY = (int16_t)(rxBuff[2] << 8 | rxBuff[3]) / (MPU_ADC_RES / ACCEL_RANGE / 2.0f) * ACCEL_UNIT_CONST
	              * ACCEL_Y_AXIS_DIRECTION
	            - ACCEL_Y_STATIC_CALIBRATION - mpu->AccelOffset[1];
	mpu->AccelZ = (int16_t)(rxBuff[4] << 8 | rxBuff[5]) / (MPU_ADC_RES / ACCEL_RANGE / 2.0f) * ACCEL_UNIT_CONST
	              * ACCEL_Z_AXIS_DIRECTION
	            - ACCEL_Z_STATIC_CALIBRATION - mpu->AccelOffset[2];

	mpu->Temp = (int16_t)(rxBuff[6] << 8 | rxBuff[7]) / 340.0f + 36.43f - TEMPERATURE_STATIC_CALIBRATION;

	mpu->GyroX = (int16_t)(rxBuff[8] << 8 | rxBuff[9]) / (MPU_ADC_RES / GYRO_RANGE / 2.0f) * GYRO_UNIT_CONST
	             * GYRO_X_AXIS_DIRECTION
	           - GYRO_X_STATIC_CALIBRATION - mpu->GyroOffset[0];
	mpu->GyroY = (int16_t)(rxBuff[10] << 8 | rxBuff[11]) / (MPU_ADC_RES / GYRO_RANGE / 2.0f) * GYRO_UNIT_CONST
	             * GYRO_Y_AXIS_DIRECTION
	           - GYRO_Y_STATIC_CALIBRATION - mpu->GyroOffset[1];
	mpu->GyroZ = (int16_t)(rxBuff[12] << 8 | rxBuff[13]) / (MPU_ADC_RES / GYRO_RANGE / 2.0f) * GYRO_UNIT_CONST
	             * GYRO_Z_AXIS_DIRECTION
	           - GYRO_Z_STATIC_CALIBRATION - mpu->GyroOffset[2];
}

HAL_StatusTypeDef MPU_RequestAllDMA(const MPU_Instance* mpu)
{
	dataRequested = true;
	return HAL_I2C_Mem_Read_DMA(mpu->hi2c, mpu->MPU_Address, ACCEL_XOUT_H, 1, rxBuff, 14);
}

HAL_StatusTypeDef MPU_ReadGyroData(MPU_Instance* mpu)
{
	uint8_t buff[6];
	const HAL_StatusTypeDef status =
	  HAL_I2C_Mem_Read(mpu->hi2c, mpu->MPU_Address, GYRO_XOUT_H, 1, (uint8_t*)&buff, 6, INIT_TIMEOUT);

	if(status)
	{
		return status;
	}

	mpu->GyroX =
	  (int16_t)(buff[0] << 8 | buff[1]) / (MPU_ADC_RES / GYRO_RANGE / 2.0f) * GYRO_UNIT_CONST * GYRO_X_AXIS_DIRECTION
	  - GYRO_X_STATIC_CALIBRATION - mpu->GyroOffset[0];
	mpu->GyroY =
	  (int16_t)(buff[2] << 8 | buff[3]) / (MPU_ADC_RES / GYRO_RANGE / 2.0f) * GYRO_UNIT_CONST * GYRO_Y_AXIS_DIRECTION
	  - GYRO_Y_STATIC_CALIBRATION - mpu->GyroOffset[1];
	mpu->GyroZ =
	  (int16_t)(buff[4] << 8 | buff[5]) / (MPU_ADC_RES / GYRO_RANGE / 2.0f) * GYRO_UNIT_CONST * GYRO_Z_AXIS_DIRECTION
	  - GYRO_Z_STATIC_CALIBRATION - mpu->GyroOffset[2];

	return status;
}

HAL_StatusTypeDef MPU_ReadAccelData(MPU_Instance* mpu)
{
	uint8_t buff[6];
	const HAL_StatusTypeDef status =
	  HAL_I2C_Mem_Read(mpu->hi2c, mpu->MPU_Address, ACCEL_XOUT_H, 1, (uint8_t*)&buff, 6, INIT_TIMEOUT);

	if(status)
	{
		return status;
	}

	mpu->AccelX =
	  (int16_t)(buff[0] << 8 | buff[1]) / (MPU_ADC_RES / ACCEL_RANGE / 2.0f) * ACCEL_UNIT_CONST * ACCEL_X_AXIS_DIRECTION
	  - ACCEL_X_STATIC_CALIBRATION - mpu->AccelOffset[0];
	mpu->AccelY =
	  (int16_t)(buff[2] << 8 | buff[3]) / (MPU_ADC_RES / ACCEL_RANGE / 2.0f) * ACCEL_UNIT_CONST * ACCEL_Y_AXIS_DIRECTION
	  - ACCEL_Y_STATIC_CALIBRATION - mpu->AccelOffset[0];
	mpu->AccelZ =
	  (int16_t)(buff[4] << 8 | buff[5]) / (MPU_ADC_RES / ACCEL_RANGE / 2.0f) * ACCEL_UNIT_CONST * ACCEL_Z_AXIS_DIRECTION
	  - ACCEL_Z_STATIC_CALIBRATION - mpu->AccelOffset[0];

	return status;
}

HAL_StatusTypeDef MPU_ReadTempData(MPU_Instance* mpu)
{
	uint8_t buff[2];
	const HAL_StatusTypeDef status =
	  HAL_I2C_Mem_Read(mpu->hi2c, mpu->MPU_Address, TEMP_OUT_H, 1, (uint8_t*)&buff, 2, INIT_TIMEOUT);

	if(status)
	{
		return status;
	}

	mpu->Temp = (int16_t)(buff[0] << 8 | buff[1]) / 340.0f + 36.43f - TEMPERATURE_STATIC_CALIBRATION;

	return status;
}

HAL_StatusTypeDef MPU_ReadAll(MPU_Instance* mpu)
{
	uint8_t buff[14];
	const HAL_StatusTypeDef status =
	  HAL_I2C_Mem_Read(mpu->hi2c, mpu->MPU_Address, ACCEL_XOUT_H, 1, (uint8_t*)&buff, 14, INIT_TIMEOUT);

	if(status)
	{
		return status;
	}

	mpu->AccelX =
	  (int16_t)(buff[0] << 8 | buff[1]) / (MPU_ADC_RES / ACCEL_RANGE / 2.0f) * ACCEL_UNIT_CONST * ACCEL_X_AXIS_DIRECTION
	  - ACCEL_X_STATIC_CALIBRATION - mpu->AccelOffset[0];
	mpu->AccelY =
	  (int16_t)(buff[2] << 8 | buff[3]) / (MPU_ADC_RES / ACCEL_RANGE / 2.0f) * ACCEL_UNIT_CONST * ACCEL_Y_AXIS_DIRECTION
	  - ACCEL_Y_STATIC_CALIBRATION - mpu->AccelOffset[1];
	mpu->AccelZ =
	  (int16_t)(buff[4] << 8 | buff[5]) / (MPU_ADC_RES / ACCEL_RANGE / 2.0f) * ACCEL_UNIT_CONST * ACCEL_Z_AXIS_DIRECTION
	  - ACCEL_Z_STATIC_CALIBRATION - mpu->AccelOffset[2];

	mpu->Temp = (int16_t)(rxBuff[6] << 8 | rxBuff[7]) / 340.0f + 36.43f - TEMPERATURE_STATIC_CALIBRATION;

	mpu->GyroX =
	  (int16_t)(buff[8] << 8 | buff[9]) / (MPU_ADC_RES / GYRO_RANGE / 2.0f) * GYRO_UNIT_CONST * GYRO_X_AXIS_DIRECTION
	  - GYRO_X_STATIC_CALIBRATION - mpu->GyroOffset[0];
	mpu->GyroY =
	  (int16_t)(buff[10] << 8 | buff[11]) / (MPU_ADC_RES / GYRO_RANGE / 2.0f) * GYRO_UNIT_CONST * GYRO_Y_AXIS_DIRECTION
	  - GYRO_Y_STATIC_CALIBRATION - mpu->GyroOffset[1];
	mpu->GyroZ =
	  (int16_t)(buff[12] << 8 | buff[13]) / (MPU_ADC_RES / GYRO_RANGE / 2.0f) * GYRO_UNIT_CONST * GYRO_Z_AXIS_DIRECTION
	  - GYRO_Z_STATIC_CALIBRATION - mpu->GyroOffset[2];

	return status;
}

void MPU_CalibrateGyro(MPU_Instance* mpu, uint32_t t)
{
	float offsetX  = 0.0f;
	float offsetY  = 0.0f;
	float offsetZ  = 0.0f;
	uint32_t count = 0;

	MPU_Instance data   = {.AccelX = 0.0f, .AccelY = 0.0f, .AccelZ = 0.0f, .GyroX = 0.0f, .GyroY = 0.0f, .GyroZ = 0.0f};
	data.hi2c           = mpu->hi2c;
	data.MPU_Address    = mpu->MPU_Address;
	const uint32_t stop = HAL_GetTick() + t;

	while(HAL_GetTick() < stop)
	{
		MPU_ReadGyroData(&data);

		offsetX += data.GyroX;
		offsetY += data.GyroY;
		offsetZ += data.GyroZ;

		count++;
	}

	mpu->GyroOffset[0] = -offsetX * GYRO_X_AXIS_DIRECTION / count;
	mpu->GyroOffset[1] = -offsetY * GYRO_Y_AXIS_DIRECTION / count;
	mpu->GyroOffset[2] = -offsetZ * GYRO_Z_AXIS_DIRECTION / count;
}

void MPU_CalibrateAccel(MPU_Instance* mpu, uint32_t t)
{
	float offsetX  = 0.0f;
	float offsetY  = 0.0f;
	float offsetZ  = 0.0f;
	uint32_t count = 0;

	MPU_Instance data = {.AccelX = 0.0f, .AccelY = 0.0f, .AccelZ = 0.0f, .GyroX = 0.0f, .GyroY = 0.0f, .GyroZ = 0.0f};
	data.hi2c         = mpu->hi2c;
	data.MPU_Address  = mpu->MPU_Address;

	const uint32_t stop = HAL_GetTick() + t;

	while(HAL_GetTick() < stop)
	{
		MPU_ReadAccelData(&data);

		offsetX += data.AccelX;
		offsetY += data.AccelY;
		offsetZ += data.AccelZ - 1;

		count++;
	}

	mpu->AccelOffset[0] = -offsetX * ACCEL_X_AXIS_DIRECTION / count;
	mpu->AccelOffset[1] = -offsetY * ACCEL_Y_AXIS_DIRECTION / count;
	mpu->AccelOffset[2] = -offsetZ * ACCEL_Z_AXIS_DIRECTION / count;
}

void MPU_CalibrateAll(MPU_Instance* mpu, uint32_t t)
{
	float acc_offsetX  = 0.0f;
	float acc_offsetY  = 0.0f;
	float acc_offsetZ  = 0.0f;
	float gyro_offsetX  = 0.0f;
	float gyro_offsetY  = 0.0f;
	float gyro_offsetZ  = 0.0f;

	uint32_t count = 0;

	MPU_Instance data = {.AccelX = 0.0f, .AccelY = 0.0f, .AccelZ = 0.0f, .GyroX = 0.0f, .GyroY = 0.0f, .GyroZ = 0.0f};
	data.hi2c         = mpu->hi2c;
	data.MPU_Address  = mpu->MPU_Address;

	const uint32_t stop = HAL_GetTick() + t;

	while(HAL_GetTick() < stop)
	{
		MPU_ReadAccelData(&data);

		acc_offsetX += data.AccelX;
		acc_offsetY += data.AccelY;
		acc_offsetZ += data.AccelZ - 1;

		gyro_offsetX += data.GyroX;
		gyro_offsetY += data.GyroY;
		gyro_offsetZ += data.GyroZ;

		count++;
	}

	mpu->AccelOffset[0] = -acc_offsetX * ACCEL_X_AXIS_DIRECTION / count;
	mpu->AccelOffset[1] = -acc_offsetY * ACCEL_Y_AXIS_DIRECTION / count;
	mpu->AccelOffset[2] = -acc_offsetZ * ACCEL_Z_AXIS_DIRECTION / count;

	mpu->GyroOffset[0] = -gyro_offsetX * GYRO_X_AXIS_DIRECTION / count;
	mpu->GyroOffset[1] = -gyro_offsetY * GYRO_Y_AXIS_DIRECTION / count;
	mpu->GyroOffset[2] = -gyro_offsetZ * GYRO_Z_AXIS_DIRECTION / count;
}

