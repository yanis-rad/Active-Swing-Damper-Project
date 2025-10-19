#ifndef SENSORS_MOT_ENC_H_
#define SENSORS_MOT_ENC_H_

/* ============================ */
/*        Include Files         */
/* ============================ */
#include "main.h"
#include "DSC.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

/* ============================ */
/*        Global Variables      */
/* ============================ */
extern float delta_us;
extern float delta_s;
extern float delta_revs;
extern TIM_HandleTypeDef htim2;

/* ============================ */
/*        Type Definitions      */
/* ============================ */
typedef struct
{
    int32_t  Motor_pos;         /**< Encoder position count */
    int16_t  Motor_spd_raw;     /**< Raw speed value from encoder */
    int16_t  Motor_spd_raw_z1;  /**< Previous raw speed value */
    int16_t  Motor_spd;         /**< Filtered motor speed */
    int16_t  Motor_spd_z1;      /**< Previous filtered motor speed */
} Motor_t;

/* ============================ */
/*     Function Declarations    */
/* ============================ */

/** @brief Initializes the motor structure to default values. */
void Motor_Init(Motor_t *motor);

/** @brief Calculates the motor speed based on encoder position and elapsed time. */
void Motor_Speed_Calc(Motor_t *motor,  float *delta_us,  float *delta_s);


#endif
