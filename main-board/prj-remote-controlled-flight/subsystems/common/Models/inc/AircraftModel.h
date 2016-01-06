/**
  ******************************************************************************
  * @file    subsystems/subsys_ContinousContorl/AircraftModel.h
  * @author  Richard Andrade
  * @version V1.0.0
  * @date    27-Julho-2015
  * @brief   Mathematic Model for the Birotor.
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef AIRCRAFTMODEL_H_
#define AIRCRAFTMODEL_H_
/* Includes ------------------------------------------------------------------*/
#include "Eigen/Dense"
#include "math.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
namespace model {

class AircraftModel {
/* Exported functions ------------------------------------------------------- */
public:
	AircraftModel();
	virtual ~AircraftModel();
	void LinearModel(Eigen::MatrixXf as, float ts,Eigen::MatrixXf * Az, Eigen::MatrixXf * Bz);
	Eigen::MatrixXf MatrixSumRho();
	Eigen::MatrixXf MatrixSumLambda();
	Eigen::MatrixXf RefrenceControl(Eigen::MatrixXf as);
	Eigen::MatrixXf MatrixTerminalCost();
	Eigen::VectorXf OutputMaxVector();
	Eigen::VectorXf OutputMinVector();
	Eigen::VectorXf ControlMaxVector();
	Eigen::VectorXf ControlMinVector();
/* Private functions ------------------------------------------------------- */
private:

};

} /* namespace aircraftmodel */

#endif /* AIRCRAFTMODEL_H_ */
