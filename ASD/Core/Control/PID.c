/*
 * PID.c
 *
 *  Created on: Sep 16, 2025
 *      Author: Yanis
 */

#include <MOT_ENC.h>
#include <stdio.h>
#include "IMU.h"
#include "PID.h"

/**
 * @brief Initializes the PID controller parameters and output limits.
 *
 * This function sets default PID gain values and PWM output limits
 * for the motor speed controller. The integral gain (`Ki`) is initialized
 * to 0, so integral action is disabled by default. The PWM output range
 * is set to ensure safe operation of the motor driver.
 *
 * @param[out] pid Pointer to a PID_t structure to initialize.
 *                  Fields initialized include:
 *                  - Kp: Proportional gain
 *                  - Ki: Integral gain
 *                  - Kd: Derivative gain
 *                  - PWM_min: Minimum allowable PWM output
 *                  - PWM_max: Maximum allowable PWM output
 *
 * @return None
 *
 */
void PID_Init(PID_t *pid)
{
	pid->Kp = 20;
    pid->Ki = 0;
    pid->Kd = 0.1;
    pid->PWM_min = 0;
    pid->PWM_max = 19999;
}


/**
 * @brief Computes the PID controller output for motor speed regulation.
 *
 * This function calculates the proportional (P), integral (I), and derivative (D)
 * terms of a PID controller based on the difference between the target motor speed
 * and the measured motor speed. The resulting control output determines the PWM
 * command to drive the motor. The proportional term is applied only when the
 * measured speed is below the target, preventing excessive correction when
 * overshooting.
 *
 * @param[in,out] pid Pointer to a PID_t structure containing the PID parameters and output:
 *                    - Kp, Ki, Kd: PID gains
 *                    - Tgt_spd: Target motor speed
 *                    - P_term, I_term, D_term: Individual PID terms
 *                    - PWM_Output: Final computed PWM output
 * @param[in] PID_motor_spd_input Current measured motor speed.
 * @param[in] PID_motor_spd_input_z1 Previous motor speed sample (used for derivative calculation).
 *
 * @return None
 */
void PID_Control(PID_t *pid, int16_t PID_motor_spd_input)
{
	int16_t error = 0;
	int16_t error_z1 = 0;

    error = abs(pid->Tgt_spd) - abs(PID_motor_spd_input);
	if (error > 0)
	{
		pid->P_term = pid->Kp * error;
	}
	else
	{
		pid->P_term = 0;
	}
	pid->I_term += pid->Ki * error * delta_s;
	pid->D_term = pid->Kd * ((error - error_z1)/delta_s);

	pid->PWM_Output = (uint16_t)(pid->P_term + 0*pid->I_term + pid->D_term);

	error_z1 = error;
}

/**
 * @brief Limits the PWM output of a PID controller and applies braking when decelerating.
 *
 * This function ensures that the PID-generated PWM does not exceed the maximum allowed value.
 * Additionally, when both rotation direction flags indicate no motion (anti-clockwise and clockwise
 * inactive) and a deceleration condition is confirmed, a fixed braking PWM is applied to slow the motor.
 *
 * @param[in,out] pid Pointer to a PID_t structure containing:
 *                    - PWM_Output: Current PWM output (modified if limiting or braking)
 *                    - PWM_max: Maximum allowable PWM output
 * @param[in] decel_sts Deceleration status flag:
 *                      - 1: Rocker is decelerating (apply brake)
 *                      - 0: Normal operation
 * @param[in] aclkwise Anti-clockwise rotation flag (0 = inactive, 1 = active)
 * @param[in] clkwise Clockwise rotation flag (0 = inactive, 1 = active)
 *
 * @return None
 *
 * @note Braking is applied only when both rotation flags are inactive and deceleration is confirmed.
 */
void PID_PWM_Limiter(PID_t *pid, int decel_sts, int aclkwise, int clkwise)
{
	if (pid->PWM_Output >= pid->PWM_max)
	{
		pid->PWM_Output = pid->PWM_max;  //limit PWM output
	}
	else if ((!aclkwise) && (!clkwise) && decel_sts)  //brake when decel_sts is confirmed
    {
		pid->Tgt_spd = 0;
		pid->PWM_Output = 19000;
	}
}

