/*
 * DSC.h
 *
 *  Created on: Sep 15, 2025
 *      Author: Yanis
 */

#ifndef SRC_DSC_H_
#define SRC_DSC_H_
//* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "fatfs.h"
#include "string.h"
#include <stdio.h>

typedef struct
{
	uint8_t IMU_state;
	uint8_t MicroSD_state;
	uint8_t Error;

}Diag_lst_t;

void Diag_Init(Diag_lst_t *diag_lst);
void Diag_Check(Diag_lst_t *diag_lst);

#endif /* SRC_DSC_H_ */
