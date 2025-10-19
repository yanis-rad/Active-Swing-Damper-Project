/* ============================ */
/*        Include Files         */
/* ============================ */
#include "MOT_ENC.h"

/* ============================ */
/*        Global Variables      */
/* ============================ */
float delta_us = 0.0f;
float delta_s  = 0.0f;
float delta_revs = 0.0f;

TIM_HandleTypeDef htim2; /**< Timer handle used for encoder input */

/* ============================ */
/*     Function Definitions     */
/* ============================ */

/**
 * @brief Initializes the motor structure to default values.
 *
 * This function resets all fields within the Motor_t structure to zero,
 * ensuring a known and stable starting state before motor control or
 * speed measurements begin. Both raw and filtered speed values, as well
 * as previous sample data, are cleared.
 *
 * @param[out] motor Pointer to a Motor_t structure to initialize.
 *                  Fields initialized include:
 *                  - Motor_pos: Encoder position
 *                  - Motor_spd_raw: Raw motor speed
 *                  - Motor_spd_raw_z1: Previous raw motor speed
 *                  - Motor_spd: Filtered motor speed
 *                  - Motor_spd_z1: Previous filtered motor speed
 *
 * @return None
 *
 */
void Motor_Init(Motor_t *motor)
{
	motor->Motor_pos = 0;
	motor->Motor_spd_raw = 0;
	motor->Motor_spd_raw_z1 = 0;
	motor->Motor_spd = 0;
	motor->Motor_spd_z1 = 0;
}


/**
 * @brief Calculates the motor speed based on encoder position and elapsed time.
 *
 * This function reads the current motor encoder position and computes the
 * raw motor speed in RPM. It uses the DWT cycle counter to measure elapsed
 * time between calls, converts it to microseconds and seconds, and calculates
 * the difference in encoder counts to estimate the motor revolutions.
 *
 * @param[in,out] motor Pointer to a Motor_t structure where the current motor
 *                      position and raw speed will be stored:
 *                      - Motor_pos: Current encoder count
 *                      - Motor_spd_raw: Calculated speed in RPM
 * @param[out] delta_us Pointer to a float where the elapsed time in microseconds
 *                      will be stored.
 * @param[out] delta_s Pointer to a float where the elapsed time in seconds
 *                      will be stored.
 *
 * @return None
 *
 * @note
 * - Assumes a 100 MHz CPU clock for the DWT cycle counter.
 * - delta_revs is calculated considering 80 ticks per revolution (two channels).
 * - If the measured delta_us is less than 1 μs, the motor speed is set to 0 to
 *   avoid invalid results.
 */
void Motor_Speed_Calc(Motor_t *motor,  float *delta_us,  float *delta_s)
{
	static int32_t last_time = 0;
	static int32_t motor_pos_z1 = 0;
    uint32_t now = 0;
	uint32_t delta = 0;
	int32_t  delta_pos = 0;

	now = DWT->CYCCNT;  //DWT ticks at CPU clock and not HCLK
	delta = now - last_time;
	*delta_us = delta / (SystemCoreClock / 1000000.0f);  // 100 MHz HCLK = 100.000.000 cycles in a second
	*delta_s =  *delta_us / 1e6f;
	last_time = now;

	motor->Motor_pos = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
	delta_pos = (motor->Motor_pos - motor_pos_z1);
	delta_revs = delta_pos / 80.0f;                      //Ticks per revolution considering two channels

	if (*delta_us > 1.0f)
	{
	    motor->Motor_spd_raw = (delta_revs * 60.0f * 1e6f) / *delta_us; // RPM
	}
	else
	{
	    motor->Motor_spd_raw = 0.0f;
	}
	motor_pos_z1 = motor->Motor_pos;

}
