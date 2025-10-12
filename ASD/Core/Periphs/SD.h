/*
 * LOG_SD.h
 *
 *  Created on: Sep 21, 2025
 *      Author: Yanis
 */

#ifndef PERIPHS_SD_H_
#define PERIPHS_SD_H_

#include <MOT_ENC.h>
#include "fatfs.h"
#include "DSC.h"
#include "PID.h"

extern FATFS FatFs;
extern FIL Fil;
extern UINT WWC; // Read/Write Word Counter
extern char RW_Buffer[200];

/** @brief Initializes the SD card and starts a new sensor data log file. */
void SDLog_Startup(void);

/** @brief Writes sensor and control data to the SD card log file and periodically synchronizes. */
uint8_t SDLog_Write(Motor_t *motor, PID_t *pid, Rocker_IMU_t *rocker, int decel_sts, float delta_us);


#endif
