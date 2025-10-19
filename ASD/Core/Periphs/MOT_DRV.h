#ifndef CONTROL_MOT_DRV_H_
#define CONTROL_MOT_DRV_H_

/* ============================ */
/*        Include Files         */
/* ============================ */
#include "PID.h"
#include "stm32f4xx_hal.h"

/* ============================ */
/*        TIM1 Handle           */
/* ============================ */
extern TIM_HandleTypeDef htim1;

/* ============================ */
/*     Function Declarations    */
/* ============================ */

/** @brief Sets the motor driver pins and PWM output to control motor speed and direction. */
void Motor_Drive_Speed_Set(uint16_t pwm_command, int pin_in3_state,  int pin_in4_state);

/** @brief Sets the motor drive pin states based on target speed and deceleration status. */
void Motor_Drive_PinState_Set(int16_t tgt_spd, int decel_sts, int *pin_in3_state, int *pin_in4_state);

#endif
