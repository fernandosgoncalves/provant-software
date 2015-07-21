/**
  ******************************************************************************
  * @file    modules/common/c_common_utils.h
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    10-February-2014
  * @brief   Funções gerais para utilização em outros módulos.
  *
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef C_COMMON_UTILS_H
#define C_COMMON_UTILS_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_conf.h"
#include "FreeRTOS.h"
#define ARM_MATH_CM4
#include "arm_math.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define PV_CONSTANT_G    9.81 //gravity

#define RAD_TO_DEG 57.2957795131f

#ifndef M_PI
#define M_PI 3.14159265359
#endif

/* Exported macro ------------------------------------------------------------*/
#define C_COMMON_UTILS_1MS_DELAY for(int i=0; i--; i<168000) { __asm("NOP"); }

/* Exported functions ------------------------------------------------------- */

/* Funções matemáticas */
float c_common_utils_map(float x, float in_min, float in_max, float out_min, float out_max);
float c_common_utils_sat(float x, float min, float max);

/* Miscelânea */
void c_common_utils_delayms(int  ms);
void c_common_utils_delayus(long us);

void c_common_utils_resetSystem();

void c_common_utils_enSysTick();
long c_common_utils_getSysTickCount();
long c_common_utils_micros();
long c_common_utils_millis();

/* Manipulações de strings */
void c_common_utils_floatToString(float num, char * outbuf, char decplaces);

/* Header-defined wrapper functions ----------------------------------------- */


#ifdef __cplusplus
}
#endif

#endif //C_COMMON_UTILS_H

