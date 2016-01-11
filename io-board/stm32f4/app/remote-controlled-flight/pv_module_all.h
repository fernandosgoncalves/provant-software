/**
  ******************************************************************************
  * @file    app/remote-controlled-flight/pv_module_all.h
  * @author  Richard Alfonso Andrade
  * @version V1.0.0
  * @date    27-August-2014
  * @brief   Implementação unificada de Threads do VANT.
  ******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PV_MODULE_ALL_H
#define PV_MODULE_ALL_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
	/* FreeRTOS kernel includes */
	#include "FreeRTOS.h"
	#include "queue.h"
	#include "task.h"

	 /* kernel includes */
	 #include "c_common_gpio.h"
	 #include "c_common_i2c.h"
	 #include "c_common_uart.h"
	 #include "c_common_utils.h"

	/* proVANT includes */
	#include "c_io_blctrl.h"
	#include "c_io_servos.h"
	#include "c_io_imu.h"
	#include "c_io_sonar.h"
	#include "c_rc_receiver.h"
	#include "pv_typedefs.h"
	#include "c_datapr_MultWii.h"

	 /* Filtros Complementares */
	 #include "c_datapr_MahonyAHRS.h"
	 /* Filtro */
	 #include "c_datapr_filter.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
	/*Def de modulos*/
	#define ENABLE_IMU
	#define ENABLE_SERVO_READ
	#define ENABLE_ALTURA
	#define ENALBE_DEBUG
	#define ENABLE_SERVO_WRITE
	#define ENABLE_ESC
 	#define AXI2826
 	//#define AXI2814

	/*Def da IMU*/
	#define ATTITUDE_MINIMUM_STEP	0.01// Radians. Minimum change in angle that is passed to the controller
	//#define REF_ROLL_MAX		0.2 //radians
	//#define REF_PITCH_MAX		0.3 //radians
	//#define REF_YAW_MAX			0.0 //radians
	//#define REF_ROLL_BIAS		-0.0087 //radians
	//#define REF_PITCH_BIAS		-0.1169

	/*Def do SONAR*/
	#define LIMIT_SONAR_VAR
	#define SONAR_MAX_VAR		0.5
	#define DSB                 0.21     //Distancia do eijo B ao sonar em metros

	/*Define o filtro a utilizar para o sonar*/
	#define SONAR_FILTER_1_ORDER_10HZ
	//#define SONAR_FILTER_2_ORDER_10HZ

	/*Define o tempo de init */
	#define INIT_ITERATIONS 1000 //For a period of 5ms, it is 5 seconds
	#define ESC_MINIMUM_VELOCITY	10//esc set point value (0-255)

/* Exported functions ------------------------------------------------------- */
void module_all_init();
void module_all_run();

#ifdef __cplusplus
}
#endif

#endif
