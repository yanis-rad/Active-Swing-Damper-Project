/*
 * PID.c
 *
 *  Created on: Sep 16, 2025
 *      Author: Yanis
 */

#include <stdio.h>
#include "IMU.h"
#include "ENC.h"
#include "PID.h"


void PID_Init(PID_t *pid)
{
	pid->Kp = 20;
    pid->Ki = 0;
    pid->Kd = 0.1;
    pid->PWM_min = 0;
    pid->PWM_max = 19999;
}

void TgtSpd_Set(Swing_IMU_t *swing, PID_t *pid, int decel_sts, double scale_factor)
{
	if (decel_sts)
	{
		pid->Tgt_spd = 0;
	}
	else
	{
		pid->Tgt_spd = (int16_t) (scale_factor * swing->Omega_Y);
	}
}

void PID_Control(PID_t *pid, Motor_t *motor)
{
	static int16_t error = 0;

    error = abs(pid->Tgt_spd) - abs(motor->Motor_spd);
	  if (error > 0)
	  {
		  pid->P_term = pid->Kp * error;
	  }
	  else
	  {
		  pid->P_term = 0;
	  }
	  pid->I_term += pid->Ki * error * delta_s;
	  pid->D_term = pid->Kd * ((motor->Motor_spd - motor->Motor_spd_z1)/delta_s);

	  pid->PWM_Output = (uint16_t)(pid->P_term + 0*pid->I_term + pid->D_term);
}

void PWM_Limiter(PID_t *pid, int decel_sts)
{
	if (pid->PWM_Output >= pid->PWM_max)
	{
				pid->PWM_Output = pid->PWM_max;  //limit PWM output
	}
	else if ((!ACLKWISE) && (!CLKWISE) && decel_sts)
    {
				pid->PWM_Output = 19000;
	}
}

