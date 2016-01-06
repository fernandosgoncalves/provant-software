/**
  ******************************************************************************
  * @file    subsystems/subsys_ContinousContorl/inc/MpcControle.h
  * @author  Richard Andrade
  * @version V1.0.0
  * @date    27-Julho-2015
  * @brief   Controlador Preditivo.
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MPCBIROTOR_H_
#define MPCBIROTOR_H_
/* Includes ------------------------------------------------------------------*/
#include "Eigen/Dense"
#include "AircraftModel.h"
#include "math.h"
#include <iostream>
#include <chrono>
#include "qpOASES.hpp"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
namespace MPCBirotor {

class MpcBirotor {
/* Exported functions ------------------------------------------------------- */
public:
	MpcBirotor();
	virtual ~MpcBirotor();
	Eigen::MatrixXf Controler(Eigen::MatrixXf states);
/* Private functions ------------------------------------------------------- */
private:
	Eigen::MatrixXf TrajetoryReference(int k);
	Eigen::MatrixXf AcelerationReference(int k);
	Eigen::MatrixXf Pow(Eigen::MatrixXf matrix, int power);

};

} /* namespace mpc */

#endif /* MPCBIROTOR_H_ */