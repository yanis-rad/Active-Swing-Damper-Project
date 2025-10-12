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


/**
 * @brief Initializes the MPU6050 sensor and verifies its communication.
 *
 * This function checks the presence of the MPU6050 sensor by reading the
 * WHO_AM_I register and comparing it against the expected device ID (0x68).
 * If the device responds correctly, the function proceeds to configure
 * the power management, sample rate, and sensitivity settings for both
 * the accelerometer and gyroscope through I2C communication.
 *
 * The IMU state field within the diagnostic structure is updated with the
 * WHO_AM_I register value to reflect sensor communication status.
 *
 * @param[out] diag_lst Pointer to a Diag_lst_t structure used to store
 *                      diagnostic information, specifically:
 *                      - IMU_state: The value read from the WHO_AM_I register.
 *
 * @return None
 *
 * @note This function sets:
 *       - Power management to active mode.
 *       - Sample rate divider to 0x07 (1 kHz / (1 + 7) = 125 Hz).
 *       - Accelerometer and gyroscope configuration to ±2g and ±250°/s respectively.
 */
void MPU6050_Init(Diag_lst_t *diag_lst)
{

	uint8_t Data;

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &(diag_lst->IMU_state), 1, 1000);

	if (diag_lst->IMU_state == 0x68)
	{

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

/**
 * @brief Initializes the rocker IMU structure to default values.
 *
 * This function resets all fields within the Rocker_IMU_t structure to zero.
 * @param[out] rocker Pointer to a Rocker_IMU_t structure that will be initialized.
 *
 * @return None
 *
 */
void IMU_Init(Rocker_IMU_t *rocker)
{
	rocker->check = 0;
	rocker->Acc_X_raw = 0;
	rocker->Omega_Y_raw = 0;
	rocker->Acc_X = 0;
	rocker->Acc_X_z1 = 0;
	rocker->Omega_Y = 0;
	rocker->Omega_Y_z1 = 0;
}


/**
 * @brief Reads accelerometer and gyroscope data from the MPU6050 sensor.
 *
 * This function retrieves raw accelerometer and gyroscope readings from
 * the MPU6050 over the I2C interface and updates the corresponding fields
 * in the rocker IMU structure. The raw X-axis acceleration value is offset
 * by 450 counts to compensate for sensor bias. The raw Y-axis angular velocity
 * is stored without scaling or filtering for later processing.
 *
 * @param[out] rocker Pointer to a Rocker_IMU_t structure where the retrieved
 *                    IMU data will be stored:
 *                    - Acc_X_raw: Raw X-axis acceleration (offset-corrected)
 *                    - Omega_Y_raw: Raw Y-axis angular velocity
 *
 * @return None
 */
void IMU_Read(Rocker_IMU_t *rocker)
{
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACC_X_H_REG, 1, Rec_Data1, 6, 1000);
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_YOUT_H_REG, 1, Rec_Data2, 6, 1000);
	rocker->Acc_X_raw = (int16_t)(Rec_Data1[0] << 8 | Rec_Data1[1]) - 450;
	rocker->Omega_Y_raw = (int16_t)(Rec_Data2[0] << 8 | Rec_Data2[1]);
}


/**
 * @brief Checks if the rocker mechanism is decelerating.
 *
 * This function monitors the acceleration and angular velocity of the rocker
 * to determine if it is in a deceleration phase. Empirical observations
 * showed that the rocker typically reaches its maximum angular velocity when
 * the X-axis acceleration goes below one-fifth of the raw Omega_Y value.
 * When this condition is met for 5 consecutive samples, a deceleration status
 * flag is set.
 * The status is reset when the rocker begins accelerating in the opposite direction.
 *
 * @param[out] decel_sts Pointer to an integer flag indicating deceleration status.
 *                       - 1: Rocker is decelerating
 *                       - 0: Rocker is not decelerating
 * @param[in]  rocker Pointer to a Rocker_IMU_t structure containing IMU data:
 *                    - Acc_X: Linear acceleration in the X-axis
 *                    - Omega_Y: Angular velocity around the Y-axis
 *                    - Omega_Y_z1: Previous angular velocity around the Y-axis
 *
 * @return None
 */
void IMU_Decel_Check(int *decel_sts, Rocker_IMU_t *rocker)
{
	static int decel_cnt = 0;

	//Monitoring of the decelerarion of the rocker
	if ((abs(rocker->Acc_X) < (int16_t)(abs(rocker->Omega_Y)*0.2)) && (decel_cnt < 5))
		  {
			  decel_cnt++;
		  }
	else if ((abs(rocker->Acc_X) < (int16_t)(abs(rocker->Omega_Y)*0.2)) && (decel_cnt > 0))
		  {
			  decel_cnt--;
		  }

	//Set target speed to 0 if for 5 consequent samples the Rocker is decelerating
	if (decel_cnt == 5)
		  {
			  *decel_sts = 1;
		  }

	//Reset once Rocker starts rotating in opposite direction
	if ((rocker->Omega_Y * rocker->Omega_Y_z1) < 0)  //the swing is accelerating back
		  {
			  *decel_sts = 0;
			   decel_cnt = 0;
		  }
}


