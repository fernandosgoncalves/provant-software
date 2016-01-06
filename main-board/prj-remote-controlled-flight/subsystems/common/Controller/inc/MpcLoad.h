/**
  ******************************************************************************
  * @file    subsystems/subsys_ContinousContorl/inc/MpcControle.h
  * @author  Richard Andrade
  * @version V1.0.0
  * @date    27-Julho-2015
  * @brief   Controlador Preditivo.
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MPCLOAD_H_
#define MPCLOAD_H_
/* Includes ------------------------------------------------------------------*/
#include "Eigen/Dense"
#include "LoadTranportationModel.h"
#include "math.h"
#include <iostream>
#include <chrono>
#include "qpOASES.hpp"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
namespace MPCLOAD {

class MpcLoad {
/* Exported functions ------------------------------------------------------- */
public:
	MpcLoad();
	virtual ~MpcLoad();
	Eigen::MatrixXf Controler(Eigen::MatrixXf states);
/* Private functions ------------------------------------------------------- */
private:
	Eigen::MatrixXf TrajetoryReference(int k);
	Eigen::MatrixXf AcelerationReference(int k);
	Eigen::MatrixXf Pow(Eigen::MatrixXf matrix, int power);

};

} /* namespace mpc */

#endif /* MPCLOAD_H_ */
