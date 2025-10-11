/*
 * IMU.c
 *
 *  Created on: Sep 15, 2025
 *      Author: Yanis
 */

#include "main.h"
#include <math.h>
#include "string.h"
#include <stdio.h>
#include "IMU.h"

uint8_t Rec_Data1[6];
uint8_t Rec_Data2[6];
I2C_HandleTypeDef hi2c1;
uint8_t confirm;


void MPU6050_Init(Diag_lst_t *diag_lst)
{

	static uint8_t Data;

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &(diag_lst->IMU_state), 1, 1000);

	if (diag_lst->IMU_state == 0x68)
	{
		confirm = 1;


		Data = 0;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR,PWR_MGMT_1_REG, 1, &Data, 1, 1000);

		Data = 0x07;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR,ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);

	}
}

void IMU_Read(Swing_IMU_t *swing)
{
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACC_X_H_REG, 1, Rec_Data1, 6, 1000);
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_YOUT_H_REG, 1, Rec_Data2, 6, 1000);
	swing->Acc_X_raw = (int16_t)(Rec_Data1[0] << 8 | Rec_Data1[1]) - 450;
	swing->Omega_Y_raw = (int16_t)(Rec_Data2[0] << 8 | Rec_Data2[1]);
}

void Decel_Check(int *decel_sts, Swing_IMU_t *swing)
{
	static int decel_cnt = 0;

	if ((abs(swing->Acc_X) < (int16_t)(abs(swing->Omega_Y)*0.2)) && (decel_cnt < 5))  //deceleration of the swing
		  {
			  decel_cnt++;
		  }
	else if ((abs(swing->Acc_X) < (int16_t)(abs(swing->Omega_Y)*0.2)) && (decel_cnt > 0))  //reset once Swing_IMU_t rotates in opposite direction
		  {
			  decel_cnt--;
		  }

	if (decel_cnt == 5)  //set target speed to 0 if for 10 consequent samples the Rocker is decelerating
		  {
			  *decel_sts = 1;
		  }
	if ((swing->Omega_Y * swing->Omega_Y_z1) < 0)  //the swing is accelerating back
		  {
			  *decel_sts = 0;
			   decel_cnt = 0;
		  }
}


