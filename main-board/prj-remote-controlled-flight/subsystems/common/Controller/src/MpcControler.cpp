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
using namespace model;
using namespace Eigen;
using namespace qpOASES;

namespace MPC {
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define definputs 4
#define defstates 16
#define prediction_horizon 5
#define prediction_control 1
#define deffreq M_PI/20
/* Private variables ---------------------------------------------------------*/
//LoadTranportationModel * Model;
AircraftModel * Model;
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
MatrixXf Az(defstates,defstates);
MatrixXf Bz(defstates,definputs);
MatrixXf Wy; // Output weighting
MatrixXf Wu; //Control weighting
MatrixXf R(defstates*prediction_horizon,1);
MatrixXf Ur(definputs*prediction_control,1);
MatrixXf G(defstates*prediction_horizon,definputs*prediction_control);
MatrixXf Q(defstates*prediction_horizon,defstates);
MatrixXf xs(defstates,1);
MatrixXf deltaxs(defstates,1);
MatrixXf DeltaU(definputs*prediction_control,1);
MatrixXf xr(defstates,1);
MatrixXf as(definputs,1);
MatrixXf ur(definputs,1);
MatrixXf Ymax(defstates*prediction_horizon,1);
MatrixXf Ymin(defstates*prediction_horizon,1);
//quadprog
QProblem qp(definputs*prediction_control,104);
Options options;
MatrixXf H;
MatrixXf ft;
VectorXf umax;
VectorXf umin;
MatrixXf Im(definputs*prediction_control,definputs*prediction_control);
MatrixXf dYmax;
MatrixXf dYmin;
MatrixXf dUmax(definputs*prediction_control,1);
MatrixXf dUmin(definputs*prediction_control,1);
MatrixXf Yr(defstates*prediction_horizon,1);
MatrixXf Ar(definputs*prediction_control,104);
MatrixXf lbr(104,1);
MatrixXf ubr(104,1);
real_t Hq[definputs*prediction_control*definputs*prediction_control];
real_t ftq[1*definputs*prediction_control];
real_t Arq[104*definputs*prediction_control];
real_t ubrq[104*1];
real_t lbrq[104*1];
real_t xOpt[definputs];

int nWSR;
//std::chrono::steady_clock::time_point last;
/* Exported functions definitions --------------------------------------------*/

MpcControler::MpcControler() {
	/*Initialization of all MPC variable*/
	s=defstates;
	p=definputs;
	q=defstates;
	N=prediction_horizon;
	M=prediction_control;
	k=0;
	frequency=deffreq;
	ts=0.01;
	//Initialize the mathematics model
	Model=new AircraftModel();
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
	/*----------- Contraints---------------*/
	Im.setIdentity();
	umax=Model->ControlMaxVector();
	umin=Model->ControlMinVector();
	VectorXf ymax=Model->OutputMaxVector();
	VectorXf ymin=Model->OutputMinVector();
	for (int i=0;i<N*q;i+=q){
	    Ymax.block(i,0,q,1)=ymax;
	    Ymin.block(i,0,q,1)=ymin;
	}
	/*Matrix Bz calculation*/
	options.printLevel=PL_NONE;
	qp.setOptions( options );
//	last=std::chrono::steady_clock::now();
}

MpcControler::~MpcControler() {
	// TODO Auto-generated destructor stub
}

Eigen::MatrixXf MpcControler::Controler(Eigen::MatrixXf states){
	Eigen::MatrixXf u(4,1);
	k=0;
	auto start = std::chrono::steady_clock::now();
	//Vector of states
	xs<<0,0,3,states.block(3,0,5,1),0,0,0,states.block(11,0,5,1);
	//Vectors of reference trajectory and control
    xr=TrajetoryReference(k).block(0,0,q,1);
    as=AcelerationReference(k);
    ur=Model->RefrenceControl(AcelerationReference(k));

    //Variation of estates
    deltaxs=xs-xr;
    std::cout<<"erros: "<<deltaxs<<std::endl;
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
	Model->LinearModel(as,ts,&Az,&Bz);

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

	//DeltaU=((G.transpose()*Wy*G)+Wu).inverse()*(G.transpose()*Wy*(R-Q*deltaxs)+Wu*Ur);
	H=(2*((G.transpose()*Wy*G)+Wu));
	//H=(H*H.transpose())/2;
	ft=(2*((Q*deltaxs-R).transpose()*Wy*G-Ur.transpose()*Wu));

	/*----------- Contraints---------------*/
	for(int i=0, Maux=1;i<M*p;i+=p, Maux++){
		dUmax.block(i,0,p,1)=umax-Model->RefrenceControl(AcelerationReference(k+Maux-1));
		dUmin.block(i,0,p,1)=Model->RefrenceControl(AcelerationReference(k+Maux-1))-umin;
	}

	for (int i=0, Naux=1;i<N*q;i+=q, Naux++){
		Yr.block(i,0,q,1)=TrajetoryReference(k+Naux).block(0,0,q,1);
	}
	dYmax=Ymax-(Q*deltaxs)-Yr;
	dYmin=(Q*deltaxs)+Yr-Ymin;
	Ar.block(0,0,definputs*prediction_control,definputs*prediction_control)=Im;
	Ar.block(0,definputs*prediction_control,definputs*prediction_control,defstates*prediction_horizon)=G.transpose();

	lbr.block(0,0,definputs*prediction_control,1)=-dUmin;
	lbr.block(definputs,0,defstates*prediction_horizon,1)=-dYmin;
	ubr.block(0,0,definputs*prediction_control,1)=dUmax;
	ubr.block(definputs,0,defstates*prediction_horizon,1)=dYmax;

	/*---------------------------------*/
	std::copy(H.data(),H.data()+H.size(),Hq);
	std::copy(ft.data(),ft.data()+ft.size(),ftq);
	std::copy(Ar.data(),Ar.data()+Ar.size(),Arq);
	std::copy(lbr.data(),lbr.data()+lbr.size(),lbrq);
	std::copy(ubr.data(),ubr.data()+ubr.size(),ubrq);
	//	/* Solve QP problem. */

//	real_t cpu=3;
	nWSR = 20;
	qp.init(Hq,ftq,Arq,0,0,lbrq,ubrq,nWSR);
	qp.getPrimalSolution(xOpt);

	u(0,0)=(ur(0,0)+xOpt[0])/1000;
	u(1,0)=(ur(1,0)+xOpt[1])/1000;
	u(2,0)=(ur(2,0)+xOpt[2])/1000;
	u(3,0)=(ur(3,0)+xOpt[3])/1000;
	auto end = std::chrono::steady_clock::now();
	auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
	std::cout << "Controll " << (float)(elapsed.count()/1000) << " miliseconds." << std::endl;
	std::cout << u << std::endl;
	qp.reset();
	return u;
}
/* Private functions ------------------------------------------------------- */
Eigen::MatrixXf MpcControler::TrajetoryReference(int k){
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
Eigen::MatrixXf MpcControler::AcelerationReference(int k){
	MatrixXf asr(4,1);
	double dot2_x,dot2_y,dot2_z,dot2_psi;
	dot2_x=0;
	dot2_y=0;
	dot2_z=0;
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
