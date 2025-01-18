/*
 * MPU6050.c
 *
 *  Created on: Jan 17, 2025
 *      Author: pwoli
 */

#include "MPU6050.h"

#define WHO_AM_I     0x75
#define PWR_MGMT_1   0x6B
#define CONFIG       0x1A
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0x1C
#define SMPRT_DIV    0x19
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43
#define TEMP_OUT_H   0x41

#define MPU_ADC_RES 65536

I2C_HandleTypeDef* _i2c;
uint8_t _rxBuff[14];

struct MPU_Data MPU_Data;

bool MPU_Init(I2C_HandleTypeDef* hi2c)
{
	_i2c = hi2c;

	uint8_t id;
	uint8_t data;

	HAL_I2C_Mem_Read(hi2c, MPU_ADDRESS, WHO_AM_I, 1, &id, 1, INIT_TIMEOUT);

	if(id == 104)
	{
		// Power mode
		data = 0;
		if(HAL_I2C_Mem_Write(_i2c, MPU_ADDRESS, PWR_MGMT_1, 1, &data, 1, INIT_TIMEOUT))
		{
			return false;
		}

		// Clock divider
		data = 0;
		if(HAL_I2C_Mem_Write(_i2c, MPU_ADDRESS, SMPRT_DIV, 1, &data, 1, INIT_TIMEOUT))
		{
			return false;
		}

		// External sync + lowpass filter
		data = 0x5;
		if(HAL_I2C_Mem_Write(_i2c, MPU_ADDRESS, CONFIG, 1, &data, 1, INIT_TIMEOUT))
		{
			return false;
		}

		// Gyro range (default 250 deg/d)
#if GYRO_RANGE == 500
		data = 0b00001000;
#elif GYRO_RANGE == 1000
		data = 0b00010000;
#elif GYRO_RANGE == 2000
		data = 0b00011000;
#else
		data = 0b00000000;
#endif
		if(HAL_I2C_Mem_Write(_i2c, MPU_ADDRESS, GYRO_CONFIG, 1, &data, 1, INIT_TIMEOUT))
		{
			return false;
		}

		// Accel Range (default 8g)
#if ACCEL_RANGE == 2
		data = 0b00000000;
#elif ACCEL_RANGE == 4
		data = 0b00001000;
#elif ACCEL_RANGE == 16
		data = 0b00011000;
#else
		data = 0b00010000;
#endif
		if(HAL_I2C_Mem_Write(_i2c, MPU_ADDRESS, ACCEL_CONFIG, 1, &data, 1, INIT_TIMEOUT))
		{
			return false;
		}

		return true;
	}

	return false;
}

void MPU_HandleRX(I2C_HandleTypeDef* hi2c)
{
	if(hi2c == _i2c)
	{
		MPU_Data.AccelX =
		  (int16_t)(_rxBuff[0] << 8 | _rxBuff[1]) / (MPU_ADC_RES / ACCEL_RANGE / 2.0) - ACCEL_X_STATIC_CALIBRATION;
		MPU_Data.AccelY =
		  (int16_t)(_rxBuff[2] << 8 | _rxBuff[3]) / (MPU_ADC_RES / ACCEL_RANGE / 2.0) - ACCEL_Y_STATIC_CALIBRATION;
		MPU_Data.AccelZ =
		  (int16_t)(_rxBuff[4] << 8 | _rxBuff[5]) / (MPU_ADC_RES / ACCEL_RANGE / 2.0) - ACCEL_Z_STATIC_CALIBRATION;

		MPU_Data.Temp = (int16_t)(_rxBuff[6] << 8 | _rxBuff[7]) / 340.0 + 36.43 - TEMPERATURE_STATIC_CALIBRATION;

		MPU_Data.GyroX =
		  (int16_t)(_rxBuff[8] << 8 | _rxBuff[9]) / (MPU_ADC_RES / GYRO_RANGE / 2.0) - GYRO_X_STATIC_CALIBRATION;
		MPU_Data.GyroY =
		  (int16_t)(_rxBuff[10] << 8 | _rxBuff[11]) / (MPU_ADC_RES / GYRO_RANGE / 2.0) - GYRO_Y_STATIC_CALIBRATION;
		MPU_Data.GyroZ =
		  (int16_t)(_rxBuff[12] << 8 | _rxBuff[13]) / (MPU_ADC_RES / GYRO_RANGE / 2.0) - GYRO_Z_STATIC_CALIBRATION;
	}
}

HAL_StatusTypeDef MPU_ReadAllDMA()
{
	return HAL_I2C_Mem_Read_DMA(_i2c, MPU_ADDRESS, ACCEL_XOUT_H, 1, _rxBuff, 14);
}

HAL_StatusTypeDef MPU_ReadGyroData(struct MPU_Data* data)
{
	uint8_t buff[6];
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(_i2c, MPU_ADDRESS, GYRO_XOUT_H, 1, (uint8_t*)&buff, 6, INIT_TIMEOUT);

	if(status)
	{
		return status;
	}

	data->GyroX =
	  (int16_t)(buff[0] << 8 | buff[1]) / (MPU_ADC_RES / GYRO_RANGE / 2.0) - GYRO_X_STATIC_CALIBRATION;
	data->GyroY =
	  (int16_t)(buff[2] << 8 | buff[3]) / (MPU_ADC_RES / GYRO_RANGE / 2.0) - GYRO_Y_STATIC_CALIBRATION;
	data->GyroZ =
	  (int16_t)(buff[4] << 8 | buff[5]) / (MPU_ADC_RES / GYRO_RANGE / 2.0) - GYRO_Z_STATIC_CALIBRATION;

	return status;
}

HAL_StatusTypeDef MPU_ReadAccelData(struct MPU_Data* data)
{
	uint8_t buff[6];
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(_i2c, MPU_ADDRESS, ACCEL_XOUT_H, 1, (uint8_t*)&buff, 6, INIT_TIMEOUT);

	if(status)
	{
		return status;
	}

	data->AccelX = (int16_t)(buff[0] << 8 | buff[1]) / (MPU_ADC_RES / ACCEL_RANGE / 2.0) - ACCEL_X_STATIC_CALIBRATION;
	data->AccelY = (int16_t)(buff[2] << 8 | buff[3]) / (MPU_ADC_RES / ACCEL_RANGE / 2.0) - ACCEL_Y_STATIC_CALIBRATION;
	data->AccelZ = (int16_t)(buff[4] << 8 | buff[5]) / (MPU_ADC_RES / ACCEL_RANGE / 2.0) - ACCEL_Z_STATIC_CALIBRATION;

	return status;
}

HAL_StatusTypeDef MPU_ReadTempData(struct MPU_Data* data)
{
	uint8_t buff[2];
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(_i2c, MPU_ADDRESS, TEMP_OUT_H, 1, (uint8_t*)&buff, 2, INIT_TIMEOUT);

	if(status)
	{
		return status;
	}

	data->Temp = (int16_t)(_rxBuff[6] << 8 | _rxBuff[7]) / 340.0 + 36.43 - TEMPERATURE_STATIC_CALIBRATION;

	return status;
}

HAL_StatusTypeDef MPU_ReadAll(struct MPU_Data* data)
{
	uint8_t buff[14];
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(_i2c, MPU_ADDRESS, ACCEL_XOUT_H, 1, (uint8_t*)&buff, 14, INIT_TIMEOUT);

	if(status)
	{
		return status;
	}

	data->AccelX =
			  (int16_t)(_rxBuff[0] << 8 | _rxBuff[1]) / (MPU_ADC_RES / ACCEL_RANGE / 2.0) - ACCEL_X_STATIC_CALIBRATION;
	data->AccelY =
			  (int16_t)(_rxBuff[2] << 8 | _rxBuff[3]) / (MPU_ADC_RES / ACCEL_RANGE / 2.0) - ACCEL_Y_STATIC_CALIBRATION;
	data->AccelZ =
			  (int16_t)(_rxBuff[4] << 8 | _rxBuff[5]) / (MPU_ADC_RES / ACCEL_RANGE / 2.0) - ACCEL_Z_STATIC_CALIBRATION;

	data->Temp = (int16_t)(_rxBuff[6] << 8 | _rxBuff[7]) / 340.0 + 36.43 - TEMPERATURE_STATIC_CALIBRATION;

	data->GyroX =
			  (int16_t)(_rxBuff[8] << 8 | _rxBuff[9]) / (MPU_ADC_RES / GYRO_RANGE / 2.0) - GYRO_X_STATIC_CALIBRATION;
	data->GyroY =
			  (int16_t)(_rxBuff[10] << 8 | _rxBuff[11]) / (MPU_ADC_RES / GYRO_RANGE / 2.0) - GYRO_Y_STATIC_CALIBRATION;
	data->GyroZ =
			  (int16_t)(_rxBuff[12] << 8 | _rxBuff[13]) / (MPU_ADC_RES / GYRO_RANGE / 2.0) - GYRO_Z_STATIC_CALIBRATION;

	return status;
}
