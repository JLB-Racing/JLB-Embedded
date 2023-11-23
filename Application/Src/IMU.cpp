/*
 * IMU.c
 *
 *  Created on: Oct 26, 2023
 *      Author: horgo
 */

#include "IMU.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1;
uint8_t IMU_initialized = 0u;
IMU_signals_s imu = {0};

uint8_t readByteFromIMU(uint8_t address)
{
	uint8_t ret = 0u;
	HAL_I2C_Master_Transmit(&hi2c1, IMU_I2C_WRITE_ADDRESS, &address, 1, 100);
	HAL_I2C_Master_Receive(&hi2c1, IMU_I2C_READ_ADDRESS, &ret, 1, 100);
	return ret;
}
uint8_t test_data[6] = {0u};

void IMU_Task()
{
	if(IMU_initialized == 0u)
	{
		uint8_t whoami = 0u;
		uint8_t whoami_reg = WHO_AM_I_ADDR;
		HAL_I2C_Master_Transmit(&hi2c1, IMU_I2C_WRITE_ADDRESS, &whoami_reg, 1, 100);
		HAL_I2C_Master_Receive(&hi2c1, IMU_I2C_READ_ADDRESS, &whoami, 1, 100);
		if(whoami != 0x6A)
		{
			return;
		}

		uint8_t pData[2] = {CTRL1_XL_ADDR, 0x60};
		//HAL_I2C_Master_Transmit(&hi2c1, IMU_I2C_WRITE_ADDRESS, pData, 2, 100);

		pData[0] = CTRL2_G_ADDR;
		HAL_I2C_Master_Transmit(&hi2c1, IMU_I2C_WRITE_ADDRESS, pData, 2, 100);

		uint8_t ctrl3c = readByteFromIMU(0x12);
		ctrl3c = ctrl3c & (~0x04);
		pData[0] = 0x12;
		pData[1] = ctrl3c;
		HAL_I2C_Master_Transmit(&hi2c1, IMU_I2C_WRITE_ADDRESS, pData, 2, 100);

		IMU_initialized = 1u;
	}
	else
	{
		uint8_t status = 0u;
		uint8_t status_reg = STATUS_REG;
		HAL_I2C_Master_Transmit(&hi2c1, IMU_I2C_WRITE_ADDRESS, &status_reg, 1, 100);
		HAL_I2C_Master_Receive(&hi2c1, IMU_I2C_READ_ADDRESS, &status, 1, 100);

		//Accelerometer new data available
		if((status & 0x01) == 1u)
		{
			uint8_t tmp_low, tmp_high;
			tmp_low = readByteFromIMU(OUTX_L_XL);
			tmp_high = readByteFromIMU(OUTX_H_XL);
			imu.acc_x = ((int16_t)((tmp_high << 8u)| tmp_low)) * AXL_SENSITIVITY;

			tmp_low = readByteFromIMU(OUTY_L_XL);
			tmp_high = readByteFromIMU(OUTY_L_XL);
			imu.acc_y = ((int16_t)((tmp_high << 8u)| tmp_low)) * AXL_SENSITIVITY;

			tmp_low = readByteFromIMU(OUTZ_L_XL);
			tmp_high = readByteFromIMU(OUTZ_L_XL);
			imu.acc_z = ((int16_t)((tmp_high << 8u)| tmp_low)) * AXL_SENSITIVITY;

		}
		//Gyroscope new data available
		if((status & 0x02) == 2u)
		{
			uint8_t tmp_low, tmp_high;

			tmp_low = readByteFromIMU(OUTZ_L_G);
			tmp_high = readByteFromIMU(OUTZ_H_G);
			imu.yaw = ((int16_t)((tmp_high << 8u)| tmp_low)) * G_SENSITIVITY / 1000.0f * 0.017453;

			//tmp_low = readByteFromIMU(OUTX_L_G);
			//tmp_high = readByteFromIMU(OUTX_H_G);
			//imu.roll = ((int16_t)((tmp_high << 8u)| tmp_low)) * G_SENSITIVITY;

			//tmp_low = readByteFromIMU(OUTY_L_G);
			//tmp_high = readByteFromIMU(OUTY_H_G);
			//imu.pitch = ((int16_t)((tmp_high << 8u)| tmp_low)) * G_SENSITIVITY;

		}
	}
}
