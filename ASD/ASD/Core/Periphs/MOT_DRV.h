/*
 * MOT_DRV.h
 *
 *  Created on: Sep 16, 2025
 *      Author: Yanis
 */

#ifndef CONTROL_MOT_DRV_H_
#define CONTROL_MOT_DRV_H_

#include "PID.h"
#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim1;

void Motor_Drive(PID_t *pid);
void PWM_PinState_Set(PID_t *pid, int decel_sts, int *PIN_IN3_STATE, int *PIN_IN4_STATE);


#endif /* CONTROL_MOT_DRV_H_ */
