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

// Measurement range of accelerometer (2, 4, 8 or 16 g)
#define ACCEL_RANGE 8
// Measurement range of gyroscope (250, 500, 500, 1000 or 2000 deg/s)
#define GYRO_RANGE 250

// Axis invertion
#define INVERT_ACCEL_X_AXIS 0
#define INVERT_ACCEL_Y_AXIS 0
#define INVERT_ACCEL_Z_AXIS 0

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


I2C_HandleTypeDef* _i2c;
uint8_t _rxBuff[14];

bool _dataRequested = false;

float _gyroOffset[3]  = {0.0f, 0.0f, 0.0f};
float _accelOffset[3] = {0.0f, 0.0f, 0.0f};


static inline void invertAccelAxis(struct IMU_Data* data)
{
#if INVERT_ACCEL_X_AXIS
	data->AccelX *= -1;
#endif

#if INVERT_ACCCEL_Y_AXIS
	data->AccelY *= -1;
#endif

#if INVERT_ACCEL_Z_AXIS
	data->AccelZ *= -1;
#endif
}

static inline void invertGyroAxis(struct IMU_Data* data)
{
#if INVERT_GYRO_X_AXIS
	data->GyroX *= -1;
#endif

#if INVERT_GYRO_Y_AXIS
	data->GyroY *= -1;
#endif

#if INVERT_GYRO_Z_AXIS
	data->GyroZ *= -1;
#endif
}

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
		data = 0;
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

void MPU_HandleRX(I2C_HandleTypeDef* hi2c, struct IMU_Data* data)
{
	if(hi2c != _i2c || !_dataRequested)
	{
		return;
	}

	_dataRequested = false;

	data->AccelX = (int16_t)(_rxBuff[0] << 8 | _rxBuff[1]) / (MPU_ADC_RES / ACCEL_RANGE / 2.0f)
	             - ACCEL_X_STATIC_CALIBRATION - _accelOffset[0];
	data->AccelY = (int16_t)(_rxBuff[2] << 8 | _rxBuff[3]) / (MPU_ADC_RES / ACCEL_RANGE / 2.0f)
	             - ACCEL_Y_STATIC_CALIBRATION - _accelOffset[1];
	data->AccelZ = (int16_t)(_rxBuff[4] << 8 | _rxBuff[5]) / (MPU_ADC_RES / ACCEL_RANGE / 2.0f)
	             - ACCEL_Z_STATIC_CALIBRATION - _accelOffset[2];

	data->Temp = (int16_t)(_rxBuff[6] << 8 | _rxBuff[7]) / 340.0f + 36.43f - TEMPERATURE_STATIC_CALIBRATION;

	data->GyroX = (int16_t)(_rxBuff[8] << 8 | _rxBuff[9]) / (MPU_ADC_RES / GYRO_RANGE / 2.0f)
	            - GYRO_X_STATIC_CALIBRATION - _gyroOffset[0];
	data->GyroY = (int16_t)(_rxBuff[10] << 8 | _rxBuff[11]) / (MPU_ADC_RES / GYRO_RANGE / 2.0f)
	            - GYRO_Y_STATIC_CALIBRATION - _gyroOffset[1];
	data->GyroZ = (int16_t)(_rxBuff[12] << 8 | _rxBuff[13]) / (MPU_ADC_RES / GYRO_RANGE / 2.0f)
	            - GYRO_Z_STATIC_CALIBRATION - _gyroOffset[2];

	invertAccelAxis(data);
	invertGyroAxis(data);
}

HAL_StatusTypeDef MPU_RequestAllDMA()
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

	data->GyroX = (int16_t)(buff[0] << 8 | buff[1]) / (MPU_ADC_RES / GYRO_RANGE / 2.0f) - GYRO_X_STATIC_CALIBRATION
	            - _gyroOffset[0];
	data->GyroY = (int16_t)(buff[2] << 8 | buff[3]) / (MPU_ADC_RES / GYRO_RANGE / 2.0f) - GYRO_Y_STATIC_CALIBRATION
	            - _gyroOffset[1];
	data->GyroZ = (int16_t)(buff[4] << 8 | buff[5]) / (MPU_ADC_RES / GYRO_RANGE / 2.0f) - GYRO_Z_STATIC_CALIBRATION
	            - _gyroOffset[2];

	invertGyroAxis(data);

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

	data->AccelX = (int16_t)(buff[0] << 8 | buff[1]) / (MPU_ADC_RES / ACCEL_RANGE / 2.0f) - ACCEL_X_STATIC_CALIBRATION
	             - _accelOffset[0];
	data->AccelY = (int16_t)(buff[2] << 8 | buff[3]) / (MPU_ADC_RES / ACCEL_RANGE / 2.0f) - ACCEL_Y_STATIC_CALIBRATION
	             - _accelOffset[0];
	data->AccelZ = (int16_t)(buff[4] << 8 | buff[5]) / (MPU_ADC_RES / ACCEL_RANGE / 2.0f) - ACCEL_Z_STATIC_CALIBRATION
	             - _accelOffset[0];

	invertAccelAxis(data);

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

	data->Temp = (int16_t)(buff[0] << 8 | buff[1]) / 340.0f + 36.43f - TEMPERATURE_STATIC_CALIBRATION;

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

	data->AccelX = (int16_t)(buff[0] << 8 | buff[1]) / (MPU_ADC_RES / ACCEL_RANGE / 2.0f) - ACCEL_X_STATIC_CALIBRATION
	             - _accelOffset[0];
	data->AccelY = (int16_t)(buff[2] << 8 | buff[3]) / (MPU_ADC_RES / ACCEL_RANGE / 2.0f) - ACCEL_Y_STATIC_CALIBRATION
	             - _accelOffset[1];
	data->AccelZ = (int16_t)(buff[4] << 8 | buff[5]) / (MPU_ADC_RES / ACCEL_RANGE / 2.0f) - ACCEL_Z_STATIC_CALIBRATION
	             - _accelOffset[2];

	data->Temp = (int16_t)(_rxBuff[6] << 8 | _rxBuff[7]) / 340.0f + 36.43f - TEMPERATURE_STATIC_CALIBRATION;

	data->GyroX = (int16_t)(buff[8] << 8 | buff[9]) / (MPU_ADC_RES / GYRO_RANGE / 2.0f) - GYRO_X_STATIC_CALIBRATION
	            - _gyroOffset[0];
	data->GyroY = (int16_t)(buff[10] << 8 | buff[11]) / (MPU_ADC_RES / GYRO_RANGE / 2.0f) - GYRO_Y_STATIC_CALIBRATION
	            - _gyroOffset[1];
	data->GyroZ = (int16_t)(buff[12] << 8 | buff[13]) / (MPU_ADC_RES / GYRO_RANGE / 2.0f) - GYRO_Z_STATIC_CALIBRATION
	            - _gyroOffset[2];

	invertAccelAxis(data);
	invertGyroAxis(data);

	return status;
}

void MPU_CallibrateGyro(uint32_t t)
{
	float offsetX  = 0.0f;
	float offsetY  = 0.0f;
	float offsetZ  = 0.0f;
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

#if INVERT_GYRO_X_AXIS
	_gyroOffset[0] = -offsetX / count;
#else
	_gyroOffset[0] = offsetX / count;
#endif

#if INVERT_GYRO_Y_AXIS
	_gyroOffset[1] = -offsetY / count;
#else
	_gyroOffset[1] = offsetY / count;
#endif

#if INVERT_GYRO_Z_AXIS
	_gyroOffset[2] = -offsetZ / count;
#else
	_gyroOffset[2] = offsetZ / count;
#endif
}

void MPU_CallibrateAccel(uint32_t t)
{
	float offsetX  = 0.0f;
	float offsetY  = 0.0f;
	float offsetZ  = 0.0f;
	uint32_t count = 0;

	struct IMU_Data data;
	uint32_t stop = HAL_GetTick() + t;

	while(HAL_GetTick() < stop)
	{
		MPU_ReadAccelData(&data);

		offsetX += data.AccelX;
		offsetY += data.AccelY;
		offsetZ += data.AccelZ;

		count++;
	}

#if INVERT_ACCEL_X_AXIS
	_accelOffset[0] = -offsetX / count;
#else
	_accelOffset[0] = (offsetX / count);
#endif

#if INVERT_ACCEL_Y_AXIS
	_accelOffset[1] = -offsetY / count;
#else
	_accelOffset[1] = offsetY / count;
#endif

#if INVERT_ACCEL_Z_AXIS
	_accelOffset[2] = -offsetZ / count - 1.0f;
#else
	_accelOffset[2] = offsetZ / count - 1.0f;
#endif
}
