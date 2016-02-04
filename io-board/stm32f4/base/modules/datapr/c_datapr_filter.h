/**
  ******************************************************************************
  * @file    modules/datapr/c_datapr_filter.c
  * @author  Rodrigo Donadel
  * @version V1.0.0
  * @date    06-August-2014
  * @brief   Filtros discretos, a principio para a filtragem de sinais dos sensores.
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef C_DATAPR_FILTER_H
#define C_DATAPR_FILTER_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_conf.h"
#include <math.h>
#include "c_rc_commons.h"



#ifdef __cplusplus
 extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/


/* Exported functions ------------------------------------------------------- */

 // Integrate the accelerometer data two times to obtain the estimated height
 float c_datapr_filter_estimate_height_acc(float *raw_acc, float *attitude);
 void reset_height_estimation(float new_initial_height, float new_initial_velocity_z);
 void c_datapr_filter_complementary(float *rpy, float c_datapr_filter_complementary_raw_acc[3], float c_datapr_filter_complementary_raw_gyr[3], float c_datapr_filter_complementary_mag[3], long sample_time_gyro_u);

#ifdef __cplusplus
}
#endif

#endif //C_IO_IMU_H
