/*
 * PID.h
 *
 *  Created on: Sep 16, 2025
 *      Author: Yanis
 */

#ifndef CONTROL_PID_H_
#define CONTROL_PID_H_

#include <math.h>
#include "string.h"
#include <stdio.h>
#include "IMU.h"
#include "ENC.h"

typedef struct
{
	double    Kp;
	double    Ki;
	double    Kd;
	double    P_term;
	double    I_term;
	double    D_term;
	uint16_t  PWM_Output;
	uint16_t  PWM_min;
	uint16_t  PWM_max;
	int16_t   Tgt_spd;
}PID_t;

void PID_Init(PID_t *pid);
void TgtSpd_Set(Swing_IMU_t *swing, PID_t *pid, int decel_sts, double scale_factor);
void PID_Control(PID_t *pid, Motor_t *motor);
void PWM_Limiter(PID_t *pid, int decel_sts);

#endif /* CONTROL_PID_H_ */
