/*
 * DSC.c
 *
 *  Created on: Sep 15, 2025
 *      Author: Yanis
 */
//* Includes ------------------------------------------------------------------*/
#include "DSC.h"
#include "string.h"

/**
  * @brief Diagnostic check of the system through three main status variables
  * @param Diag_lst
  * @retval None
  */

void Diag_Init(Diag_lst_t *diag_lst)
{
	diag_lst->IMU_state = 0;
	diag_lst->MicroSD_state = 0;
	diag_lst->Error = 0;
}

void Diag_Check(Diag_lst_t *diag_lst)
{
	if ((diag_lst->MicroSD_state != 0)   || \
	    (diag_lst->IMU_state != 104) )
		  {
	          diag_lst->Error = 1;
		  }
		  else
		  {
			  diag_lst->Error = 0;
		  }
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, diag_lst->Error); //RED led
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, !(diag_lst->Error)); //GREEN led
}
