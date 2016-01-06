/**
  ******************************************************************************
  * @file    subsystems/subsys_ContinousContorl/inc/ReferenceTrajectory.h
  * @author  Richard Andrade
  * @version V1.0.0
  * @date    17-Nov-2015
  * @brief   Reference Generator.
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef REFERENCETRAJECTORY_H_
#define REFERENCETRAJECTORY_H_
/* Includes ------------------------------------------------------------------*/
#include "Eigen/Dense"
#include "math.h"
#include <iostream>
#include <chrono>
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
namespace TRAJECTORY {

class ReferenceTrajectory {
/* Exported functions ------------------------------------------------------- */
public:
	ReferenceTrajectory();
	ReferenceTrajectory(float sample_time);
	virtual ~ReferenceTrajectory();
	Eigen::MatrixXf TrajetoryReference_LQR();
	Eigen::MatrixXf TrajetoryReference_MPC(long k);
	Eigen::MatrixXf AcelerationReference(int k);
/* Private functions ------------------------------------------------------- */
private:

};

} /* namespace TRAJECTORY */

#endif /* REFERENCETRAJECTORY_H_ */
