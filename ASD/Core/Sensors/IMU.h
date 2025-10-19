#ifndef SENSORS_IMU_H_
#define SENSORS_IMU_H_
/* ============================ */
/*        Include Files         */
/* ============================ */
#include "stm32f4xx_hal.h"
#include "DSC.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ============================ */
/*        Type Definitions      */
/* ============================ */
typedef struct
{
    uint8_t   check;          /**< Sensor check status byte */
    int16_t   Acc_X_raw;      /**< Raw accelerometer X-axis data */
    int16_t   Omega_Y_raw;    /**< Raw gyroscope Y-axis data */
    int16_t   Acc_X;          /**< Processed accelerometer X-axis value */
    int16_t   Acc_X_z1;       /**< Previous accelerometer X-axis value */
    int16_t   Omega_Y;        /**< Processed gyroscope Y-axis value */
    int16_t   Omega_Y_z1;     /**< Previous gyroscope Y-axis value */
} Rocker_IMU_t;

/* ============================ */
/*        Global Variables      */
/* ============================ */
extern uint8_t Rec_Data1[6];
extern uint8_t Rec_Data2[6];
extern I2C_HandleTypeDef hi2c1;

/* ============================ */
/*        Macro Definitions     */
/* ============================ */
#define MPU6050_ADDR         0xD0  /**< 8-bit I2C address (0x68 << 1) */
#define WHO_AM_I_REG         0x75
#define PWR_MGMT_1_REG       0x6B
#define SMPLRT_DIV_REG       0x19
#define GYRO_CONFIG_REG      0x1B
#define ACCEL_CONFIG_REG     0x1C
#define GYRO_YOUT_H_REG      0x45
#define ACC_X_H_REG          0x3B

/* ============================ */
/*     Function Declarations    */
/* ============================ */

/** @brief Computes the PID controller output for motor speed control. */
void MPU6050_Init(Diag_lst_t *diag_lst);

/** @brief Initializes the rocker IMU structure to default values.  */
void IMU_Init(Rocker_IMU_t *rocker);

/** @brief Reads accelerometer and gyroscope data from the MPU6050 sensor. */
void IMU_Read(Rocker_IMU_t *rocker);

/** @brief Checks if the rocker mechanism is decelerating. */
void IMU_Decel_Check(int *decel_sts, Rocker_IMU_t *rocker);

#endif /* SENSORS_IMU_H_ */
