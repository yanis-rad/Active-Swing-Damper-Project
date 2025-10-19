/* ============================ */
/*        Include Files         */
/* ============================ */
#include "DSC.h"
#include "string.h"

/* ============================ */
/*     Function Definitions     */
/* ============================ */

/**
  * @brief  Initializes the diagnostic status structure.
  *
  * This function resets all diagnostic parameters in the provided
  * Diag_lst_t structure to their default values. It sets the IMU state,
  * MicroSD state, and error flag to zero, ensuring a clean starting point
  * before performing diagnostic checks.
  *
  * @param[out] diag_lst Pointer to a Diag_lst_t structure to be initialized.
  *
  * @retval None
  */
void Diag_Init(Diag_lst_t *diag_lst)
{
	diag_lst->IMU_state = 0;
	diag_lst->MicroSD_state = 0;
	diag_lst->Error = 0;
}

/**
  * @brief  Performs a diagnostic check on system components and updates error status.
  *
  * This function verifies the diagnostic states of key system components such as
  * the MicroSD and IMU. If any of these components report an unexpected state,
  * an error flag is set in the provided diagnostic structure. Additionally, the
  * function updates two indicator LEDs to reflect the current diagnostic status:
  *   - RED LED (GPIOB, PIN 8) is ON when an error is present.
  *   - GREEN LED (GPIOA, PIN 4) is ON when the system is healthy.
  *
  * @param[in,out] diag_lst Pointer to a Diag_lst_t structure containing
  *                         diagnostic state information and an error flag.
  *
  * @retval None
  */
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
