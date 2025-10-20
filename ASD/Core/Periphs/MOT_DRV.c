/* ============================ */
/*        Include Files         */
/* ============================ */
#include "MOT_DRV.h"


/* ============================ */
/*        TIM1 Handle            */
/* ============================ */
TIM_HandleTypeDef htim1;

/* ============================ */
/*     Function Definitions     */
/* ============================ */

/**
 * @brief Sets the motor driver pins and PWM output to control motor speed and direction.
 *
 * This function updates the IN3 and IN4 pins to control motor rotation direction
 * and sets the PWM duty cycle to control motor speed. It directly interacts with
 * GPIO and timer hardware.
 *
 * @param[in] pwm_command PWM duty cycle value to apply to the motor (via TIM1 Channel 2).
 * @param[in] pin_in3_state State of IN3 pin (0 = low, 1 = high) for motor direction control.
 * @param[in] pin_in4_state State of IN4 pin (0 = low, 1 = high) for motor direction control.
 *
 * @return None
 *
 * @note
 * - IN3 and IN4 combination determines forward, reverse, or braking.
 * - PWM value must not exceed the configured timer period to avoid saturation.
 */
void Motor_Drive_Speed_Set(uint16_t pwm_command, int pin_in3_state,  int pin_in4_state)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, pin_in3_state);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, pin_in4_state);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_command);
}

/**
 * @brief Sets motor driver IN3/IN4 pin states based on target speed and deceleration.
 *
 * This function controls the motor driver pins to determine rotation direction
 * or braking. Speeds within ±300 are treated as an inactivity band and the motor
 * is stopped. Braking is also applied if deceleration is confirmed.
 *
 * @param[in] tgt_spd Target motor speed (signed integer)
 * @param[in] decel_sts Deceleration status flag:
 *                      - 1: Motor is decelerating (apply brake)
 *                      - 0: Normal operation
 * @param[out] pin_in3_state Pointer to the variable representing IN3 pin state (0 or 1)
 * @param[out] pin_in4_state Pointer to the variable representing IN4 pin state (0 or 1)
 *
 * @return None
 *
 * @note
 * - Positive `tgt_spd` sets IN3 high and IN4 low (forward)
 * - Negative `tgt_spd` sets IN3 low and IN4 high (reverse)
 * - Both pins high correspond to braking
 */
void Motor_Drive_PinState_Set(int16_t tgt_spd, int decel_sts, int *pin_in3_state, int *pin_in4_state)
{
	if ((abs(tgt_spd) < 60) || decel_sts) //Inactivity speed band for values lower than 60 RPM
	{
		*pin_in3_state = 1;
		*pin_in4_state = 1;
	}
	else if (tgt_spd >= 60)
	{
		*pin_in3_state = 1;
		*pin_in4_state = 0;
    }
	else if (tgt_spd <= -60)  
	{
		*pin_in3_state = 0;
		*pin_in4_state = 1;
	}
}

