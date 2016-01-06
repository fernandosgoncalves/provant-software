/**
  ******************************************************************************
  * @file    subsystems/subsys_ContinousContorl/inc/MpcControle.h
  * @author  Richard Andrade
  * @version V1.0.0
  * @date    27-Julho-2015
  * @brief   Controlador Preditivo.
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MPCCONTORLER_H_
#define MPCCONTORLER_H_
/* Includes ------------------------------------------------------------------*/
#include "Eigen/Dense"
#include "LoadTranportationModel.h"
#include "AircraftModel.h"
#include "math.h"
#include <iostream>
#include <chrono>
#include "qpOASES.hpp"
#include "ReferenceTrajectory.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

namespace MPC {

class MpcControler {
/* Exported functions ------------------------------------------------------- */
public:
	MpcControler();
	virtual ~MpcControler();
	Eigen::MatrixXf Controler(Eigen::MatrixXf states);
/* Private functions ------------------------------------------------------- */
private:
	Eigen::MatrixXf TrajetoryReference(int k);
	Eigen::MatrixXf AcelerationReference(int k);
	Eigen::MatrixXf Pow(Eigen::MatrixXf matrix, int power);
	TRAJECTORY::ReferenceTrajectory * trajectory;

};

} /* namespace mpc */

#endif /* MPCCONTORLER_H_ */
