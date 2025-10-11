/** @file ENC.c
 *  @brief DC motor speed calculator based on encoder data
 *
 *  This file contains the function needed to compute the speed
 *  based on encoder data. The encoder is data is based on a two
 *  out-of-phase output channels
 *
 *  @author Yanislav Radyul
 */

#include "ENC.h"

float delta_us;
float delta_s;
TIM_HandleTypeDef htim2;
float delta_revs;
int PIN_IN3_STATE;
int PIN_IN4_STATE;

/**
  * @brief Speed computation using encoder data sampled in time
  * @param Diag_lst,
  * @retval None
  */

void Speed_Calc(Motor_t *motor, Diag_lst_t *diag_lst,  float *delta_us,  float *delta_s)
{
	static int32_t last_time = 0;
    static uint32_t now = 0;
	static uint32_t delta = 0;
	static int32_t delta_pos = 0;
	static int32_t motor_pos_z1 = 0;

	now = DWT->CYCCNT;  //DWT ticks at CPU clock and not HCLK
	delta = now - last_time;
	*delta_us = delta / (SystemCoreClock / 1000000.0f);  // 100 MHz HCLK = 100.000.000 cycles in a second
	*delta_s =  *delta_us / 1e6f;
	last_time = now;

	motor->Motor_pos = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);


	delta_pos = (motor->Motor_pos - motor_pos_z1);
	delta_revs = delta_pos / 80.0f;                      //ticks per revolution

	if (*delta_us > 1.0f) {
	    motor->Motor_spd_raw = (delta_revs * 60.0f * 1e6f) / *delta_us; // RPM
	} else {
	    motor->Motor_spd_raw = 0.0f;
	}
	motor_pos_z1 = motor->Motor_pos;

}
