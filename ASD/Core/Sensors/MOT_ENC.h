/*
 * MOT_ENC.c
 *
 *  Created on: Sep 15, 2025
 *      Author: Yanis
 */


#ifndef SENSORS_MOT_ENC_H_
#define SENSORS_MOT_ENC_H_

#include "main.h"
#include <math.h>
#include "string.h"
#include <stdio.h>
#include "DSC.h"

extern float delta_us;
extern float delta_s;
extern float delta_revs;
extern TIM_HandleTypeDef htim2;


typedef struct
{
	int32_t  Motor_pos;
	int16_t  Motor_spd_raw;
	int16_t  Motor_spd_raw_z1;
	int16_t  Motor_spd;
	int16_t  Motor_spd_z1;
}Motor_t;

/** @brief Initializes the motor structure to default values. */
void Motor_Init(Motor_t *motor);

/** @brief Calculates the motor speed based on encoder position and elapsed time. */
void Motor_Speed_Calc(Motor_t *motor,  float *delta_us,  float *delta_s);


#endif
