/*
 * IMU.h
 *
 *  Created on: Oct 26, 2023
 *      Author: horgo
 */

#ifndef SRC_IMU_H_
#define SRC_IMU_H_

#define IMU_I2C_READ_ADDRESS 		0b11010111
#define IMU_I2C_WRITE_ADDRESS 		0b11010110

#define CTRL1_XL_ADDR				0x10
#define CTRL2_G_ADDR				0x11
#define WHO_AM_I_ADDR				0x0F
#define STATUS_REG					0x1E

#define AXL_SENSITIVITY				0.061f	//[mg/LSB]
#define G_SENSITIVITY				3.81469f	//[mdps/LSB]

#define OUTX_L_G					0x22
#define OUTX_H_G					0x23
#define OUTY_L_G					0x24
#define OUTY_H_G					0x25
#define OUTZ_L_G					0x26
#define OUTZ_H_G					0x27

#define OUTX_L_XL					0x28
#define OUTX_H_XL					0x29
#define OUTY_L_XL					0x2A
#define OUTY_H_XL					0x2B
#define OUTZ_L_XL					0x2C
#define OUTZ_H_XL					0x2D



typedef struct
{
	float acc_x;	//[mg]
	float acc_y;	//[mg]
	float acc_z;	//[mg]

	float yaw;		//[dps]
	float pitch;	//[dps]
	float roll;		//[dps]
} IMU_signals_s;

void IMU_Task();

#endif /* SRC_IMU_H_ */
