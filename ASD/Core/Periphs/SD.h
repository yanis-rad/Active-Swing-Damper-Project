#ifndef PERIPHS_SD_H_
#define PERIPHS_SD_H_


/* ============================ */
/*        Include Files         */
/* ============================ */
#include <MOT_ENC.h>
#include "fatfs.h"
#include "DSC.h"
#include "PID.h"

/* ============================ */
/*        Global Variables      */
/* ============================ */
extern FATFS FatFs;
extern FIL Fil;
extern UINT WWC; // Read/Write Word Counter
extern char RW_Buffer[200];

/* ============================ */
/*     Function Declarations    */
/* ============================ */

/** @brief Initializes the SD card and starts a new sensor data log file. */
void SDLog_Startup(void);

/** @brief Writes sensor and control data to the SD card log file and periodically synchronizes. */
void SDLog_Write(Motor_t *motor, PID_t *pid, Diag_lst_t *diag_lst, Rocker_IMU_t *rocker, int decel_sts, float delta_us);


#endif
