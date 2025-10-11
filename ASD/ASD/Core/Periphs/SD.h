/*
 * LOG_SD.h
 *
 *  Created on: Sep 21, 2025
 *      Author: Yanis
 */

#ifndef PERIPHS_SD_H_
#define PERIPHS_SD_H_

#include "fatfs.h"
#include "DSC.h"
#include "PID.h"
#include "ENC.h"

extern FATFS FatFs;
extern FIL Fil;
extern UINT WWC; // Read/Write Word Counter
extern char RW_Buffer[200];


void SDLog_Startup(void);
void SDLog_Write(Motor_t *motor, PID_t *pid, Swing_IMU_t *swing, Diag_lst_t *diag_lst, int decel_sts, float delta_us);


#endif /* PERIPHS_SD_H_ */
