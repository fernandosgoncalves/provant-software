/**
  ******************************************************************************
  * @file    subsystems/subsys_ContinousContorl/src/ReferenceTrajectory.cpp
  * @author  Richard Andrade
  * @version V1.0.0
  * @date    17-Nov-2015
  * @brief   Reference Generator.
  *****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "ReferenceTrajectory.h"
/* Private typedef -----------------------------------------------------------*/
using namespace Eigen;

namespace TRAJECTORY {
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
int nWSR;
float psi_r;
/* Exported functions definitions --------------------------------------------*/

ReferenceTrajectory::ReferenceTrajectory() {

}

ReferenceTrajectory::~ReferenceTrajectory() {
	// TODO Auto-generated destructor stub
}

Eigen::MatrixXf ReferenceTrajectory::TrajetoryReference_LQR(){
	MatrixXf R(16,1);
	double x,y,z,phi,theta,psi,alphal,alphar;
	double dot_x,dot_y,dot_z,dot_phi,dot_theta,dot_psi,dot_alphal,dot_alphar;
	R.setZero();
	x=0;
	y=0;
	z=3;
	phi=0.0000890176;
	theta=0.0154833;
	psi=0;
	//std::cout<<"->"<<psi<<std::endl;
	alphar=-0.0154821;
	alphal=-0.0153665;
	dot_x=0;
	dot_y=0;
	dot_z=0;
	dot_phi=0;
	dot_theta=0;
	dot_psi=0;
	dot_alphar=0;
	dot_alphal=0;
	R<<x,y,z,phi,theta,psi,alphal,alphar,dot_x,dot_y,dot_z,dot_phi,
			dot_theta,dot_psi,dot_alphal,dot_alphar;
	return R;
}

/* Private functions ------------------------------------------------------- */

} /* namespace TRAJECTORY */