/*
 * IMU.c
 *
 *  Created on: Oct 26, 2023
 *      Author: horgo
 */

#include "IMU.h"
#include "lsm6dsl.h"

extern LSM6DSL_CommonDrv_t LSM6DSL_COMMON_Driver;
extern LSM6DSL_ACC_Drv_t LSM6DSL_ACC_Driver;
extern LSM6DSL_GYRO_Drv_t LSM6DSL_GYRO_Driver;

LSM6DSL_Object_t imu_obj;
LSM6DSL_Axes_t imu_acc_axes;
LSM6DSL_Axes_t imu_gyr_axes;

void IMU_Init()
{
	LSM6DSL_COMMON_Driver.Init(&imu_obj);
	LSM6DSL_ACC_Driver.Enable(&imu_obj);
	LSM6DSL_GYRO_Driver.Enable(&imu_obj);
	LSM6DSL_ACC_Driver.SetOutputDataRate(&imu_obj, 210.0f);
	LSM6DSL_GYRO_Driver.SetOutputDataRate(&imu_obj, 210.0f);
}

void IMU_Task()
{
	LSM6DSL_ACC_Driver.GetAxes(&imu_obj, &imu_acc_axes);
	LSM6DSL_GYRO_Driver.GetAxes(&imu_obj, &imu_gyr_axes);
}
