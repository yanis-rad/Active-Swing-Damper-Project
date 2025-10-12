/*
 * SD.c
 *
 *  Created on: Sep 21, 2025
 *      Author: Yanis
 */

#include "SD.h"

FATFS FatFs;
FIL Fil;
UINT WWC; // Read/Write Word Counter
char RW_Buffer[200];

/**
 * @brief Initializes the SD card and starts a new sensor data log file.
 *
 * This function mounts the FAT filesystem, opens (or creates) the log file
 * "Sensor_data_log.txt" in append mode, and writes a header indicating the
 * start of a new data logging session. Column headers are also written to
 * label the recorded sensor and control values.
 *
 * @return None
 *
 * @note
 * - Ensure that the SD card is properly connected and FATFs is initialized.
 * - The log file is appended if it already exists.
 * - Column headers include: Time, Position, Speed, Target Speed, Omega_y, PWM Output, Decel_Sts, Acc_X
 */
void SDLog_Startup(void)
{
	f_mount(&FatFs, "", 1);
	f_open(&Fil, "Sensor_data_log.txt", FA_WRITE | FA_OPEN_APPEND);
	f_puts("***************************************************NEW DATA LOG***************************************************\n", &Fil);
	f_puts("Time      Position        Speed         Target Speed         Omega_y        PWM Output        Decel_Sts      Acc_X \n", &Fil);
}


/**
 * @brief Writes sensor and control data to the SD card log file and periodically synchronizes.
 *
 * This function formats and writes a single line of data to "Sensor_data_log.txt",
 * including motor position, speed, target speed, IMU readings, PWM output, and deceleration status.
 * The buffer is synchronized to the SD card every 30 writes to ensure data integrity, and
 * the status of the last sync operation is returned.
 *
 * @param[in] motor Pointer to the Motor_t structure containing motor position and speed.
 * @param[in] pid Pointer to the PID_t structure containing target speed and PWM output.
 * @param[in] rocker Pointer to the Rocker_IMU_t structure containing IMU data.
 * @param[in] decel_sts Deceleration status flag (1 if decelerating, 0 otherwise).
 * @param[in] delta_us Elapsed time in microseconds since the last sample.
 *
 * @return uint8_t Status of the last SD card sync operation:
 *                 - 0: Success
 *                 - Non-zero: Error code from f_sync()
 *
 * @note
 * - The log line includes: Time, Motor Position, Motor Speed, Target Speed, Omega_Y, PWM Output, Decel_Sts, Acc_X
 * - `f_sync()` is called every 30 writes to flush the buffer to the SD card.
 * - Ensure `RW_Buffer` is large enough to hold a single formatted line.
 */
uint8_t SDLog_Write(Motor_t *motor, PID_t *pid, Rocker_IMU_t *rocker, int decel_sts, float delta_us)
{
	static int f_sync_count = 0;
	uint8_t f_sync_sts;

	f_sync_count++;
	snprintf(RW_Buffer, sizeof(RW_Buffer), "%8u %8ld %8d %8d %8d %8u %8u %8d\n",(uint16_t) delta_us, motor->Motor_pos, motor->Motor_spd, pid->Tgt_spd, rocker->Omega_Y , pid->PWM_Output, decel_sts, rocker->Acc_X);
	f_write(&Fil, RW_Buffer, strlen(RW_Buffer), &WWC);
	if (f_sync_count == 30)
	{
		f_sync_sts = f_sync(&Fil);
	    f_sync_count = 0;
	}

	return f_sync_sts;
}
