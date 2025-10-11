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

void SDLog_Startup(void)
{
	f_mount(&FatFs, "", 1);
	f_open(&Fil, "Sensor_data_log.txt", FA_WRITE | FA_OPEN_APPEND);
	f_puts("***************************************************NEW DATA LOG***************************************************\n", &Fil);
	f_puts("Time    Position     Speed    Filtered Speed   Target Speed    Omega_y    PWM Output      PWM  \n", &Fil);
}

void SDLog_Write(Motor_t *motor, PID_t *pid, Swing_IMU_t *swing, Diag_lst_t *diag_lst, int decel_sts, float delta_us)
{
	static int f_sync_count = 0;

	f_sync_count++;
	snprintf(RW_Buffer, sizeof(RW_Buffer), "%8u %8ld %8d %8d %8d %8u %8u %8d %8d\n",(uint16_t) delta_us, (motor->Motor_pos), motor->Motor_spd, pid->Tgt_spd, swing->Omega_Y , pid->PWM_Output, decel_sts, 0, swing->Acc_X);
	f_write(&Fil, RW_Buffer, strlen(RW_Buffer), &WWC);
	if (f_sync_count == 30)
	{
		diag_lst->MicroSD_state = f_sync(&Fil);
	    f_sync_count = 0;
	}
}
