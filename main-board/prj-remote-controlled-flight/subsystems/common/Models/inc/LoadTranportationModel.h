/**
  ******************************************************************************
  * @file    subsystems/subsys_ContinousContorl/MpcControle.h
  * @author  Richard Andrade
  * @version V1.0.0
  * @date    27-Julho-2015
  * @brief   Mathematic Model for Load Transportation.
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LOADTRANPORTATIONMODEL_H_
#define LOADTRANPORTATIONMODEL_H_
/* Includes ------------------------------------------------------------------*/
#include "Eigen/Dense"
#include "math.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
namespace loadmodel {

class LoadTranportationModel {
/* Exported functions ------------------------------------------------------- */
public:
	LoadTranportationModel();
	virtual ~LoadTranportationModel();
	Eigen::MatrixXf MatrixA(Eigen::MatrixXf as, float ts);
	Eigen::MatrixXf MatrixB(float ts);
	Eigen::MatrixXf MatrixC();
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

} /* namespace loadmodel */

#endif /* LOADTRANPORTATIONMODEL_H_ */
