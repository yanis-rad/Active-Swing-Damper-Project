#ifndef SRC_DSC_H_
#define SRC_DSC_H_

/* ============================ */
/*        Include Files         */
/* ============================ */
#include <stdint.h>
#include "fatfs.h"
#include "string.h"
#include <stdio.h>

/* ============================ */
/*        Type Definitions      */
/* ============================ */
typedef struct
{
	uint8_t IMU_state;
	uint8_t MicroSD_state;
	uint8_t Error;

}Diag_lst_t;

/* ============================ */
/*     Function Declarations    */
/* ============================ */

/** @brief  Initializes the diagnostic status structure. */
void Diag_Init(Diag_lst_t *diag_lst);

/** @brief  Performs a diagnostic check on system components and updates error status. */
void Diag_Check(Diag_lst_t *diag_lst);

#endif /* SRC_DSC_H_ */
