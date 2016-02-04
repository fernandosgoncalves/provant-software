/**
  ******************************************************************************
  * @file    modules/rc/c_rc_LQR_control.h
  * @author  Rodrigo Donadel
  * @version V1.0.0
  * @date    08-December-2014
  * @brief   Controle de estabilizacao por realimentacao de estados calculado com LQR.
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef C_RC_LQR_CONTROL_H
#define C_RC_LQR_CONTROL_H

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

/* Exported macro ------------------------------------------------------------*/
//#define LQR_INTEGRATOR
/* Exported functions ------------------------------------------------------- */
 void c_rc_LQR2_control_init();

//LQR attitude and height(AH) controller. Height control depends on global variable manual_height_control.
pv_type_actuation c_rc_LQR2_controller(pv_type_datapr_attitude attitude,
				  pv_type_datapr_attitude attitude_reference,
				  pv_type_datapr_position position,
				  pv_type_datapr_position position_reference,
				  pv_type_datapr_servos servo,
				  pv_type_datapr_servos servo_reference,
				  float sample_time, bool stop);

#ifdef __cplusplus
}
#endif

#endif //C_RC_SFC_H
