/**
  ******************************************************************************
  * @file    subsystems/subsys_ContinousContorl/src/MpcControle.cpp
  * @author  Richard Andrade
  * @version V1.0.0
  * @date    27-Julho-2015
  * @brief   Controlador Preditivo.
  *****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "MpcControler.h"
/* Private typedef -----------------------------------------------------------*/
using namespace loadmodel;
using namespace Eigen;
using namespace qpOASES;
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
LoadTranportationModel * Model;
int s; // Number of states
int p; // Inputs number
int q; // Outputs number
int N; // Prediction Horizon
int M; // Prodiction Control
float ts; //Sample time
long k; // discret time
float frequency;
int lastTime;
double f;
MatrixXf Az(20,20);
MatrixXf Bz(20,4);
MatrixXf Wy; // Output weighting
MatrixXf Wu; //Control weighting
MatrixXf R(20*5,1);
MatrixXf Ur(4*1,1);
MatrixXf G(20*5,4*1);
MatrixXf Q(20*5,20);
MatrixXf xs(20,1);
MatrixXf deltaxs(20,1);
MatrixXf DeltaU(4*1,1);
MatrixXf xr(20,1);
MatrixXf as(4,1);
MatrixXf ur(4,1);
//quadprog
MatrixXd a(4,1);
VectorXd b(1,1);
VectorXd c(4);
//std::chrono::steady_clock::time_point last;
namespace MPC {
/* Exported functions definitions --------------------------------------------*/

MpcControler::MpcControler() {
	/*Initialization of all MPC variable*/
	s=20;
	p=4;
	q=20;
	N=5;
	M=1;
	k=0;
	frequency=M_PI/20;
	ts=0.01;
	//Initialize the mathematics model
	Model=new LoadTranportationModel();
	MatrixXf SumRho(2,2);
	MatrixXf SumLambda(2,2);
	MatrixXf TerminalCost(2,2);
	Wy.setZero(q*N,q*N); //Output weight matrix
	Wu.setZero(p*M,p*M); //Input weight matrix
	/*Output weight matrix creation*/
	SumRho=Model->MatrixSumRho();
	TerminalCost=Model->MatrixTerminalCost();
	for(int i=0;i<q*N;i+=q){
		if(i<q*(N-1)){
			Wy.block(i,i,q,q)=SumRho;
		}else{
			Wy.block(i,i,q,q)=TerminalCost;
		}
	}
	/*Input weight matrix creation*/
	SumLambda=Model->MatrixSumLambda();
	for(int i=0;i<p*M;i+=p){
		Wu.block(i,i,p,p)=SumLambda;
	}
	/*Matrix Bz calculation*/
	Bz=Model->MatrixB(ts);
//	last=std::chrono::steady_clock::now();
}

MpcControler::~MpcControler() {
	// TODO Auto-generated destructor stub
}

Eigen::MatrixXf MpcControler::Controler(Eigen::MatrixXf states){
	QProblem qp( 4,0 );
	Options options;
	qp.setOptions( options );
	auto start = std::chrono::steady_clock::now();
	//Vector of states
	xs<<0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1;
	//Vectors of reference trajectory and control
    xr=TrajetoryReference(k).block(0,0,q,1);
    as=AcelerationReference(k);
    ur=Model->RefrenceControl(AcelerationReference(k));
    //Variation of estates
    deltaxs=xs-xr;
    //Refrence vector of future variation
	R.setZero();
	for (int i=0, Naux=1;i<N*q;i+=q, Naux++){
		R.block(i,0,q,1)=TrajetoryReference(k+Naux).block(0,0,q,1)-
				TrajetoryReference(k+Naux-1).block(0,0,q,1);
	}
//	//Refrence control of future variation
	for(int i=0, Maux=1;i<M*p;i+=p, Maux++){
		if (k==0){
			Ur.block(i,0,p,1)=Model->RefrenceControl(AcelerationReference(k+Maux-1));
		}else{
			Ur.block(i,0,p,1)=Model->RefrenceControl(AcelerationReference(k+Maux-1))-
					Model->RefrenceControl(AcelerationReference(k+Maux-2));
		}
	}
	/*Low control calculation*/
    Az=Model->MatrixA(as,ts);
    G.setZero();
    for(int j=0, Maux=1;j<p*M;j+=p, Maux++){
		for(int i=0, Naux=1;i<q*N;i+=q, Naux++){
			if(Naux>=Maux){
				G.block(i,j,q,p)=Pow(Az,Naux-Maux)*Bz;
			}
		}
	}
	for(int i=0, Naux=1;i<q*N;i+=q, Naux++){
		Q.block(i,0,q,s)=Pow(Az,Naux);
	}

	DeltaU=((G.transpose()*Wy*G)+Wu).inverse()*(G.transpose()*Wy*(R-Q*deltaxs)+Wu*Ur);
	//std::cout << "DeltaU" << DeltaU << std::endl;

	MatrixXf H=(2*((G.transpose()*Wy*G)+Wu));//.cast<double>();
	H=(H*H.transpose())/2;
	std::cout<<"H="<<H<<std::endl;
	MatrixXf ft=(2*((Q*deltaxs-R).transpose()*Wy*G-Ur.transpose()*Wu));//.cast<double>();
	real_t Hq[4*4];
	real_t ftq[1*4];
	std::copy(H.data(),H.data()+H.size(),Hq);
	std::copy(ft.data(),ft.data()+ft.size(),ftq);
	/* Solve QP problem. */
	int nWSR = 5000;
	real_t cpu=3;
	qp.init( Hq,ftq,0,0,0,0,0, nWSR,&cpu);

	real_t xOpt[4];
	real_t yOpt[4+1];
	qp.getPrimalSolution( xOpt );
	qp.getDualSolution( yOpt );

	auto end = std::chrono::steady_clock::now();
	auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
	std::cout << "It took me " << (float)(elapsed.count()/1000) << " microseconds." << std::endl;
	std::cout<<"xOpt=";
	for(int i=0;i<4;i++){
		std::cout<<xOpt[i]<<" ";
	}
	std::cout<<std::endl;
	if (k<=5)
		k++;
	return ur;
}
/* Private functions ------------------------------------------------------- */
Eigen::MatrixXf MpcControler::TrajetoryReference(int k){
	MatrixXf R(20,1);
	double x,y,z,phi,theta,psi,alphal,alphar,gamma1,gamma2;
	double dot_x,dot_y,dot_z,dot_phi,dot_theta,dot_psi,dot_alphal,dot_alphar,dot_gamma1,dot_gamma2;
	R.setZero();
	x=0.5*cos(frequency*ts*k);
	y=0.5*sin(frequency*ts*k);
	z=3-2*cos(frequency*ts*k);
	phi=0.000197071;
	theta=0.0885222;
	psi=0;
	alphar=-0.0881387;
	alphal=-0.0882323;
	gamma1=-0.000195539;
	gamma2=-0.0885222;
	dot_x=-0.5*frequency*sin(frequency*ts*k);
	dot_y=0.5*frequency*cos(frequency*ts*k);
	dot_z=2*frequency*sin(frequency*ts*k);
	dot_phi=0;
	dot_theta=0;
	dot_psi=0;
	dot_alphar=0;
	dot_alphal=0;
	dot_gamma1=0;
	dot_gamma2=0;

	R<<x,y,z,phi,theta,psi,alphal,alphar,gamma1,gamma2,dot_x,dot_y,dot_z,dot_phi,
			dot_theta,dot_psi,dot_alphal,dot_alphar,dot_gamma1,dot_gamma2;
	return R;
}
Eigen::MatrixXf MpcControler::AcelerationReference(int k){
	MatrixXf asr(4,1);
	double dot2_x,dot2_y,dot2_z,dot2_psi;
	dot2_x=-0.5*pow(frequency,2)*cos(frequency*ts*k);
	dot2_y=-0.5*pow(frequency,2)*sin(frequency*ts*k);
	dot2_z=2*pow(frequency,2)*cos(frequency*ts*k);
	dot2_psi=0;
	asr<<dot2_x, dot2_y, dot2_z, dot2_psi;
	return asr;
}
Eigen::MatrixXf MpcControler::Pow(Eigen::MatrixXf matrix, int power){
	MatrixXf aux;
	if(power==0){
		aux=MatrixXf::Identity(matrix.rows(),matrix.cols());
	}else{
		aux=matrix;
		for(int i=1;i<power;i++){
			aux=aux*matrix;
		}
	}
	return aux;
}
} /* namespace mpc */