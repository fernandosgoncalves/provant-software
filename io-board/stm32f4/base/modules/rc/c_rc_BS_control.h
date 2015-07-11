/**
  ******************************************************************************
  * @file    modules/rc/c_rc_control.h
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    30-November-2013
  * @brief   Controle de estabilizacao para vôo com usando controle remoto manual.
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef C_RC_CONTROL_H
#define C_RC_CONTROL_H

#define ARM_MATH_CM4
#include "arm_math.h"
#include "c_rc_commons.h"
#include "pv_typedefs.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void c_rc_BS_control_init();
//backstepping attitude and height(AH) controller. Height control depends on global variable manual_height_control.
pv_type_actuation c_rc_BS_AH_controller(pv_type_datapr_attitude attitude,
				  pv_type_datapr_attitude attitude_reference,
				  pv_type_datapr_position position,
				  pv_type_datapr_position position_reference,
				  float throttle_control,
				  bool manual_height_control,
				  bool enable_integration);


#ifdef __cplusplus
}
#endif

#endif //C_RC_CONTROL_H
