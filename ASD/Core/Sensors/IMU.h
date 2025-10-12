/*
 * IMU.h
 *
 *  Created on: Sep 15, 2025
 *      Author: Yanis
 */
#include <math.h>
#include "string.h"
#include <stdio.h>
#include "DSC.h"
#include <stdlib.h>
#include "stm32f4xx_hal.h"

#ifndef SENSORS_IMU_H_
#define SENSORS_IMU_H_

typedef struct
{
	uint8_t   check;
	int16_t   Acc_X_raw;
	int16_t   Omega_Y_raw;
	int16_t   Acc_X;
	int16_t   Acc_X_z1;
    int16_t   Omega_Y;
    int16_t   Omega_Y_z1;
}Rocker_IMU_t;

extern uint8_t Rec_Data1[6];
extern uint8_t Rec_Data2[6];
extern I2C_HandleTypeDef hi2c1;
#define MPU6050_ADDR         0xD0  // 8-bit I2C address (0x68 << 1)
#define WHO_AM_I_REG         0x75
#define PWR_MGMT_1_REG       0x6B
#define SMPLRT_DIV_REG       0x19
#define GYRO_CONFIG_REG      0x1B
#define ACCEL_CONFIG_REG     0x1C
#define GYRO_YOUT_H_REG      0x45
#define ACC_X_H_REG          0x3B

/** @brief Computes the PID controller output for motor speed control. */
void MPU6050_Init(Diag_lst_t *diag_lst);

/** @brief Initializes the rocker IMU structure to default values.  */
void IMU_Init(Rocker_IMU_t *rocker);

/** @brief Reads accelerometer and gyroscope data from the MPU6050 sensor. */
void IMU_Read(Rocker_IMU_t *rocker);

/** @brief Checks if the rocker mechanism is decelerating. */
void IMU_Decel_Check(int *decel_sts, Rocker_IMU_t *rocker);

#endif /* SENSORS_IMU_H_ */
