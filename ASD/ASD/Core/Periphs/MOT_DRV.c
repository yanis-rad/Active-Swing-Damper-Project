/*
 * MOT_DRV.c
 *
 *  Created on: Sep 16, 2025
 *      Author: Yanis
 */

#include "MOT_DRV.h"

TIM_HandleTypeDef htim1;


void Motor_Drive(PID_t *pid)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, PIN_IN3_STATE);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, PIN_IN4_STATE);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pid->PWM_Output);
}

void PWM_PinState_Set(PID_t *pid, int decel_sts, int *PIN_IN3_STATE, int *PIN_IN4_STATE)
{
	if ((abs(pid->Tgt_spd) < 300) || decel_sts) //braking speed band
	{
		*PIN_IN3_STATE = 1;
		*PIN_IN4_STATE = 1;
		pid->Tgt_spd = 0;
	}
	else if (pid->Tgt_spd >= 300)
	{
		*PIN_IN3_STATE = 1;
		*PIN_IN4_STATE = 0;
    }
	else if (pid->Tgt_spd <= -300)
	{
		*PIN_IN3_STATE = 0;
		*PIN_IN4_STATE = 1;
	}
}

