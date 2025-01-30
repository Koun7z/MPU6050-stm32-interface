/*
 * MPU6050.c
 *
 *  Created on: Jan 17, 2025
 *      Author: pwoli
 */

#include "MPU6050.h"

#define MPU_ADDRESS 0xD0

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

// I2C timeout when communicating in blocking mode
#define INIT_TIMEOUT 1000

// Measurement range of accelerometer (2, 4, 8 or 16 g)
#define ACCEL_RANGE 8
// Measurement range of gyroscope (250, 500, 500, 1000 or 2000 deg/s)
#define GYRO_RANGE 1000

// Static zero reference offset for the gyroscope
#define GYRO_X_STATIC_CALIBRATION 5.3728
#define GYRO_Y_STATIC_CALIBRATION 0.5760
#define GYRO_Z_STATIC_CALIBRATION -2.0512

// Static zero reference offset for the Acceletometer
#define ACCEL_X_STATIC_CALIBRATION 0.0247
#define ACCEL_Y_STATIC_CALIBRATION -0.0092
#define ACCEL_Z_STATIC_CALIBRATION -0.0411

// Static zero reference offset for the temerature sensor
#define TEMPERATURE_STATIC_CALIBRATION 0

I2C_HandleTypeDef* _i2c;
uint8_t _rxBuff[14];

bool _dataRequested = false;

float _gyroOffset[3]  = {0, 0, 0};
float _accelOffset[3] = {0, 0, 0};

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
	if(hi2c != _i2c || !_dataRequested)
	{
		return;
	}

	_dataRequested = false;

	MPU_Data.AccelX = (int16_t)(_rxBuff[0] << 8 | _rxBuff[1]) / (MPU_ADC_RES / ACCEL_RANGE / 2.0)
	                - ACCEL_X_STATIC_CALIBRATION - _accelOffset[0];
	MPU_Data.AccelY = (int16_t)(_rxBuff[2] << 8 | _rxBuff[3]) / (MPU_ADC_RES / ACCEL_RANGE / 2.0)
	                - ACCEL_Y_STATIC_CALIBRATION - _accelOffset[1];
	MPU_Data.AccelZ = (int16_t)(_rxBuff[4] << 8 | _rxBuff[5]) / (MPU_ADC_RES / ACCEL_RANGE / 2.0)
	                - ACCEL_Z_STATIC_CALIBRATION - _accelOffset[2];

	MPU_Data.Temp = (int16_t)(_rxBuff[6] << 8 | _rxBuff[7]) / 340.0 + 36.43 - TEMPERATURE_STATIC_CALIBRATION;

	MPU_Data.GyroX = (int16_t)(_rxBuff[8] << 8 | _rxBuff[9]) / (MPU_ADC_RES / GYRO_RANGE / 2.0)
	               - GYRO_X_STATIC_CALIBRATION - _gyroOffset[0];
	MPU_Data.GyroY = (int16_t)(_rxBuff[10] << 8 | _rxBuff[11]) / (MPU_ADC_RES / GYRO_RANGE / 2.0)
	               - GYRO_Y_STATIC_CALIBRATION - _gyroOffset[1];
	MPU_Data.GyroZ = (int16_t)(_rxBuff[12] << 8 | _rxBuff[13]) / (MPU_ADC_RES / GYRO_RANGE / 2.0)
	               - GYRO_Z_STATIC_CALIBRATION - _gyroOffset[2];
}

HAL_StatusTypeDef MPU_ReadAllDMA()
{
	_dataRequested = true;
	return HAL_I2C_Mem_Read_DMA(_i2c, MPU_ADDRESS, ACCEL_XOUT_H, 1, _rxBuff, 14);
}

HAL_StatusTypeDef MPU_ReadGyroData(struct IMU_Data* data)
{
	uint8_t buff[6];
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(_i2c, MPU_ADDRESS, GYRO_XOUT_H, 1, (uint8_t*)&buff, 6, INIT_TIMEOUT);

	if(status)
	{
		return status;
	}

	data->GyroX =
	  (int16_t)(buff[0] << 8 | buff[1]) / (MPU_ADC_RES / GYRO_RANGE / 2.0) - GYRO_X_STATIC_CALIBRATION - _gyroOffset[0];
	data->GyroY =
	  (int16_t)(buff[2] << 8 | buff[3]) / (MPU_ADC_RES / GYRO_RANGE / 2.0) - GYRO_Y_STATIC_CALIBRATION - _gyroOffset[1];
	data->GyroZ =
	  (int16_t)(buff[4] << 8 | buff[5]) / (MPU_ADC_RES / GYRO_RANGE / 2.0) - GYRO_Z_STATIC_CALIBRATION - _gyroOffset[2];

	return status;
}

HAL_StatusTypeDef MPU_ReadAccelData(struct IMU_Data* data)
{
	uint8_t buff[6];
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(_i2c, MPU_ADDRESS, ACCEL_XOUT_H, 1, (uint8_t*)&buff, 6, INIT_TIMEOUT);

	if(status)
	{
		return status;
	}

	data->AccelX = (int16_t)(buff[0] << 8 | buff[1]) / (MPU_ADC_RES / ACCEL_RANGE / 2.0) - ACCEL_X_STATIC_CALIBRATION
	             - _accelOffset[0];
	data->AccelY = (int16_t)(buff[2] << 8 | buff[3]) / (MPU_ADC_RES / ACCEL_RANGE / 2.0) - ACCEL_Y_STATIC_CALIBRATION
	             - _accelOffset[0];
	data->AccelZ = (int16_t)(buff[4] << 8 | buff[5]) / (MPU_ADC_RES / ACCEL_RANGE / 2.0) - ACCEL_Z_STATIC_CALIBRATION
	             - _accelOffset[0];

	return status;
}

HAL_StatusTypeDef MPU_ReadTempData(struct IMU_Data* data)
{
	uint8_t buff[2];
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(_i2c, MPU_ADDRESS, TEMP_OUT_H, 1, (uint8_t*)&buff, 2, INIT_TIMEOUT);

	if(status)
	{
		return status;
	}

	data->Temp = (int16_t)(buff[6] << 8 | buff[7]) / 340.0 + 36.43 - TEMPERATURE_STATIC_CALIBRATION;

	return status;
}

HAL_StatusTypeDef MPU_ReadAll(struct IMU_Data* data)
{
	uint8_t buff[14];
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(_i2c, MPU_ADDRESS, ACCEL_XOUT_H, 1, (uint8_t*)&buff, 14, INIT_TIMEOUT);

	if(status)
	{
		return status;
	}

	data->AccelX = (int16_t)(buff[0] << 8 | buff[1]) / (MPU_ADC_RES / ACCEL_RANGE / 2.0) - ACCEL_X_STATIC_CALIBRATION
	             - _accelOffset[0];
	data->AccelY = (int16_t)(buff[2] << 8 | buff[3]) / (MPU_ADC_RES / ACCEL_RANGE / 2.0) - ACCEL_Y_STATIC_CALIBRATION
	             - _accelOffset[1];
	data->AccelZ = (int16_t)(buff[4] << 8 | buff[5]) / (MPU_ADC_RES / ACCEL_RANGE / 2.0) - ACCEL_Z_STATIC_CALIBRATION
	             - _accelOffset[2];

	data->Temp = (int16_t)(_rxBuff[6] << 8 | _rxBuff[7]) / 340.0 + 36.43 - TEMPERATURE_STATIC_CALIBRATION;

	data->GyroX =
	  (int16_t)(buff[8] << 8 | buff[9]) / (MPU_ADC_RES / GYRO_RANGE / 2.0) - GYRO_X_STATIC_CALIBRATION - _gyroOffset[0];
	data->GyroY = (int16_t)(buff[10] << 8 | buff[11]) / (MPU_ADC_RES / GYRO_RANGE / 2.0) - GYRO_Y_STATIC_CALIBRATION
	            - _gyroOffset[1];
	data->GyroZ = (int16_t)(buff[12] << 8 | buff[13]) / (MPU_ADC_RES / GYRO_RANGE / 2.0) - GYRO_Z_STATIC_CALIBRATION
	            - _gyroOffset[2];

	return status;
}

float* MPU_CallibrateGyro(uint32_t t)
{
	float offsetX  = 0;
	float offsetY  = 0;
	float offsetZ  = 0;
	uint32_t count = 0;

	struct IMU_Data data;
	uint32_t stop = HAL_GetTick() + t;

	while(HAL_GetTick() < stop)
	{
		MPU_ReadGyroData(&data);

		offsetX += data.GyroX;
		offsetY += data.GyroY;
		offsetZ += data.GyroZ;

		count++;
	}

	_gyroOffset[0] = offsetX / count;
	_gyroOffset[1] = offsetY / count;
	_gyroOffset[2] = offsetZ / count;

	return &_gyroOffset;
}

float* MPU_CallibrateAccel(uint32_t t)
{
	float offsetX  = 0;
	float offsetY  = 0;
	float offsetZ  = 0;
	uint32_t count = 0;

	struct IMU_Data data;
	uint32_t stop = HAL_GetTick() + t;

	while(HAL_GetTick() < stop)
	{
		MPU_ReadGyroData(&data);

		offsetX += data.AccelX;
		offsetY += data.AccelY;
		offsetZ += data.AccelZ;

		count++;
	}

	_accelOffset[0] = offsetX / count;
	_accelOffset[1] = offsetY / count;
	_accelOffset[2] = offsetZ / count - 1;

	return &_accelOffset;
}
