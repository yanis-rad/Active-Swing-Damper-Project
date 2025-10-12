/*
 * PID.h
 *
 *  Created on: Sep 16, 2025
 *      Author: Yanis
 */

#ifndef CONTROL_PID_H_
#define CONTROL_PID_H_

#include <math.h>
#include <MOT_ENC.h>
#include "string.h"
#include <stdio.h>
#include "IMU.h"

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

/** @brief Initializes PID gains and PWM limits. */
void PID_Init(PID_t *pid);

/** @brief Computes the PID controller output for motor speed control. */
void PID_Control(PID_t *pid, int16_t PID_motor_spd_input, int16_t PID_motor_spd_input_z1);

/** @brief Limits the PWM output of a PID controller and applies braking when decelerating. */
void PID_PWM_Limiter(PID_t *pid, int decel_sts, int aclkwise, int clkwise);

#endif
