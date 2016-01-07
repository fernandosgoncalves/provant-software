/**
  ******************************************************************************
  * @file    modules/io/c_io_imu.h
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    11-February-2014
  * @brief   Funções para IMUs (incialmente baseadas no CIs ITG3205 e ADXL345).
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef C_IO_IMU_H
#define C_IO_IMU_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_conf.h"

#include "c_common_i2c.h"
#include "c_common_utils.h"
#include "c_common_gpio.h"


#define ARM_MATH_CM4
#include "arm_math.h"
#include <math.h>

/* FreeRTOS kernel includes */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
//#define C_IO_IMU_USE_ITG_ADXL_HMC
//#define C_IO_IMU_USE_MPU6050_HMC5883
#define C_IO_IMU_USE_GY_87

/* Exported constants --------------------------------------------------------*/
#define PV_IMU_ROLL        0
#define PV_IMU_PITCH       1
#define PV_IMU_YAW         2
#define PV_IMU_DROLL       3
#define PV_IMU_DPITCH      4
#define PV_IMU_DYAW        5

#define PV_IMU_X           0
#define PV_IMU_Y           1
#define PV_IMU_Z           2


#define ACC_FILTER_2OD_5HZ
#define MAG_FILTER_2OD_5HZ

#ifdef C_IO_IMU_USE_ITG_ADXL_HMC
	#define OFFSET_GYRO_X		   	-0.054287654320988
	#define OFFSET_GYRO_Y		   	-0.017202469135802
	#define OFFSET_GYRO_Z			0.008808641975309

 	 // Accelerometer
 	 // "accel x,y,z (min/max) = X_MIN/X_MAX Y_MIN/Y_MAX Z_MIN/Z_MAX *1000"
 	 #define ACCEL_X_MIN ((float) -0.996)
 	 #define ACCEL_X_MAX ((float) 1.097)
 	 #define ACCEL_Y_MIN ((float) -1.031)
 	 #define ACCEL_Y_MAX ((float) 1.070)
 	 #define ACCEL_Z_MIN ((float) -1.115)
 	 #define ACCEL_Z_MAX ((float) 0.914)

 	 //Magnetometer parameters for calibration
 	 //see:http://diydrones.com/profiles/blogs/advanced-hard-and-soft-iron-magnetometer-calibration-for-dummies?id=705844%3ABlogPost%3A1676387&page=2#comments
 	 #define M11 1.807
 	 #define M12 -0.055
 	 #define M13 0.052
 	 #define M21 0.214
 	 #define M22 1.926
 	 #define M23 0.003
 	 #define M31 0.02
 	 #define M32 -0.048
 	 #define M33 2.071

 	 #define Bx -106.511
 	 #define By -150.561
 	 #define Bz -417.946

#endif
#ifdef C_IO_IMU_USE_GY_87
	#define OFFSET_GYRO_X		    -0.0261683229863
	#define OFFSET_GYRO_Y		    -0.0001869642579
	#define OFFSET_GYRO_Z			-0.0000596364945

 	 // Accelerometer
  	 // "accel x,y,z (min/max) = X_MIN/X_MAX Y_MIN/Y_MAX Z_MIN/Z_MAX *1000"
     #define ACCEL_X_MIN ((float) -0.996)
	 #define ACCEL_X_MAX ((float)  1.097)
	 #define ACCEL_Y_MIN ((float) -1.031)
	 #define ACCEL_Y_MAX ((float)  1.070)
	 #define ACCEL_Z_MIN ((float) -1.115)
	 #define ACCEL_Z_MAX ((float)  0.914)

  	 //Magnetometer parameters for calibration
  	 //see:http://diydrones.com/profiles/blogs/advanced-hard-and-soft-iron-magnetometer-calibration-for-dummies?id=705844%3ABlogPost%3A1676387&page=2#comments
  	 #define M11 1.807
  	 #define M12 -0.055
  	 #define M13 0.052
  	 #define M21 0.214
  	 #define M22 1.926
  	 #define M23 0.003
  	 #define M31 0.02
  	 #define M32 -0.048
  	 #define M33 2.071

  	 #define Bx -106.511
  	 #define By -150.561
  	 #define Bz -417.946
#endif


// Sensor calibration scale and offset values
// #define ACCEL_SENSIBILITY 256
#define ACCEL_X_OFFSET ((ACCEL_X_MIN + ACCEL_X_MAX) / 2.0f)
#define ACCEL_Y_OFFSET ((ACCEL_Y_MIN + ACCEL_Y_MAX) / 2.0f)
#define ACCEL_Z_OFFSET ((ACCEL_Z_MIN + ACCEL_Z_MAX) / 2.0f)
#define ACCEL_X_SCALE (1 / (ACCEL_X_MAX - ACCEL_X_OFFSET))
#define ACCEL_Y_SCALE (1 / (ACCEL_Y_MAX - ACCEL_Y_OFFSET))
#define ACCEL_Z_SCALE (1 / (ACCEL_Z_MAX - ACCEL_Z_OFFSET))

// define to use the calibration data. If not defined then the raw values of the sensors are used
#define CALIBRATE


/* Exported functions ------------------------------------------------------- */
 void c_io_imu_init(I2C_TypeDef* I2Cx);
 void c_io_imu_getRaw(float  * accRaw, float * gyrRaw, float * magRaw, long * sample_time__gyro_us);
 void c_io_imu_getBarometerRaw(long *pressure,float *temperature);
 float c_io_imu_getTemperature();
 long c_io_imu_getPressure();
 float c_io_imu_getAltitude();
 void c_io_imu_getComplimentaryRPY(float * acce_raw, float * gyro_raw, float * magn_raw, float sample_time, float * rpy);
 void c_io_imu_getKalmanFilterRPY(float * rpy, float * acce_raw, float * gyro_raw, float * magn_raw);
 void c_io_imu_initKalmanFilter();
 void c_io_imu_calibrate();
 void c_io_imu_Quaternion2Euler(float * q, float * rpy);
 void c_io_imu_Quaternion2EulerMadgwick(float * q, float * rpy);
 void c_io_imu_EulerMatrix(float * rpy, float * velAngular);
 float abs2(float num);

#ifdef __cplusplus
}
#endif

#endif //C_IO_IMU_H
