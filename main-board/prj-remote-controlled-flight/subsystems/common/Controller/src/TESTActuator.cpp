/**
  ****************************************************************************
  * @file    subsystems/subsys_ContinousContorl/src/MpcControle.cpp
  * @author  Richard Andrade
  * @version V1.0.0
  * @date    17-Nov-2015
  * @brief   Test atuadores.
  *****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "TESTActuator.h"
/* Private typedef -----------------------------------------------------------*/
using namespace Eigen;

namespace TEST {
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
int nWSR;
//std::chrono::steady_clock::time_point last;
/* Exported functions definitions --------------------------------------------*/

TESTActuator::TESTActuator() {
}

TESTActuator::~TESTActuator() {
	// TODO Auto-generated destructor stub
}

Eigen::MatrixXf TESTActuator::Controler(Eigen::MatrixXf channels){
	Eigen::MatrixXf u(4,1);
	u(0,0)=((channels(0,0)+100)/200)*17;
	u(1,0)=((channels(0,0)+100)/200)*17;
	u(2,0)=(channels(1,0)/84)*2;
	u(3,0)=(channels(1,0)/84)*2;
	// Total control action
	return u;
}
/* Private functions ------------------------------------------------------- */

} /* namespace TEST */
