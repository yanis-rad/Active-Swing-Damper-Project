/*
 * ENC.h
 *
 *  Created on: Sep 15, 2025
 *      Author: Yanis
 */



#ifndef SENSORS_ENC_H_
#define SENSORS_ENC_H_

#include "main.h"
#include <math.h>
#include "string.h"
#include <stdio.h>
#include "DSC.h"

extern float delta_us;
extern float delta_s;
extern float delta_revs;
extern TIM_HandleTypeDef htim2;
extern int PIN_IN3_STATE;
extern int PIN_IN4_STATE;
#define CLKWISE ((PIN_IN3_STATE==1) && (PIN_IN4_STATE == 0))
#define ACLKWISE ((PIN_IN3_STATE==0) && (PIN_IN4_STATE == 1))



typedef struct
{
	int32_t  Motor_pos;
	int16_t  Motor_spd_raw;
	int16_t  Motor_spd_raw_z1;
	int16_t  Motor_spd;
	int16_t  Motor_spd_z1;
}Motor_t;

void Speed_Calc(Motor_t *motor, Diag_lst_t *diag_lst,  float *delta_us,  float *delta_s);


#endif /* SENSORS_ENC_H_ */
