/**
  ******************************************************************************
  * @file    subsystems/subsys_ContinousContorl/inc/MpcControle.h
  * @author  Richard Andrade
  * @version V1.0.0
  * @date    17-Nov-2015
  * @brief   LQR Controler estabilization.
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef TESTACTUATOR_H_
#define TESTACTUATOR_H_
/* Includes ------------------------------------------------------------------*/
#include "Eigen/Dense"
#include "math.h"
#include <iostream>
#include <chrono>
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
namespace TEST {

class TESTActuator {
/* Exported functions ------------------------------------------------------- */
public:
	TESTActuator();
	virtual ~TESTActuator();
	Eigen::MatrixXf Controler(Eigen::MatrixXf channels);
/* Private functions ------------------------------------------------------- */
private:
};

} /* namespace TEST */

#endif /* LQRCONTORLER_H_ */
