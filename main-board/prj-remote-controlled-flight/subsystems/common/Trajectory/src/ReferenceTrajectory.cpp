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
float sample_time;
/* Exported functions definitions --------------------------------------------*/
ReferenceTrajectory::ReferenceTrajectory() {

}

ReferenceTrajectory::ReferenceTrajectory(float sample) {
	sample_time=sample;
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
	z=1;
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

Eigen::MatrixXf ReferenceTrajectory::TrajetoryReference_MPC(long auxk){
	float k;
	k=auxk*sample_time;
	MatrixXf R(16,1);
	double x,y,z,phi,theta,psi,alphal,alphar;
	double dot_x,dot_y,dot_z,dot_phi,dot_theta,dot_psi,dot_alphal,dot_alphar;
	R.setZero();
	x=0;
	y=0;
	z=0.048;
	//z=3*(1-exp(-0.9*k));
	phi=0.0000890176;
	theta=0.0154833;
	psi=0.4887;
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
Eigen::MatrixXf ReferenceTrajectory::AcelerationReference(int auxk){
	float k;
	k=auxk*sample_time;
	MatrixXf asr(4,1);
	double dot2_x,dot2_y,dot2_z,dot2_psi;
	dot2_x=0;
	dot2_y=0;
	dot2_z=0;
	dot2_psi=0;
	asr<<dot2_x, dot2_y, dot2_z, dot2_psi;
	return asr;
}

/* Private functions ------------------------------------------------------- */

} /* namespace TRAJECTORY */
