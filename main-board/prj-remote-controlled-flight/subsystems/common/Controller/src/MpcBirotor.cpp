/**
  ******************************************************************************
  * @file    subsystems/subsys_ContinousContorl/src/MpcControle.cpp
  * @author  Richard Andrade
  * @version V1.0.0
  * @date    27-Julho-2015
  * @brief   Controlador Preditivo.
  *****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "MpcBirotor.h"
/* Private typedef -----------------------------------------------------------*/
using namespace model;
using namespace Eigen;
using namespace qpOASES;
using namespace TRAJECTORY;

namespace MPCBirotor {
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define PRED_HOR 5
#define PRED_CONT 1
/* Private variables ---------------------------------------------------------*/
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
/*MPC variable*/
MatrixXf Az(18,18);
MatrixXf Bz(18,4);
MatrixXf Wy; // Output weighting
MatrixXf Wu; //Control weighting
MatrixXf R(18*PRED_HOR,1);
MatrixXf Ur(4*PRED_CONT,1);
MatrixXf G(18*PRED_HOR,4*PRED_CONT);
MatrixXf Q(18*PRED_HOR,18);
MatrixXf DeltaU(4*PRED_CONT,1);
MatrixXf ar(4,1);
MatrixXf ur(4,1);
/*State variables*/
MatrixXf xs(16,1);
MatrixXf deltaxs(16,1);
MatrixXf xr(16,1);
MatrixXf xs_aumented(18,1);
/*Integrator variable*/
MatrixXf xsi(2,1);
MatrixXf xsiant(2,1);
MatrixXf deltaxsi(2,1);
MatrixXf deltaxsiant(2,1);
/*Constraint Variable*/
QProblem qp(4,18*PRED_HOR+4*PRED_CONT);
Options options;
MatrixXf H;
MatrixXf ft;
VectorXf umax;
VectorXf umin;
MatrixXf Im(4*PRED_CONT,4*PRED_CONT);
MatrixXf Ymax(18*PRED_HOR,1);
MatrixXf Ymin(18*PRED_HOR,1);
MatrixXf dYmax;
MatrixXf dYmin;
MatrixXf dUmax(4*PRED_CONT,1);
MatrixXf dUmin(4*PRED_CONT,1);
MatrixXf Yr(18*PRED_HOR,1);
MatrixXf Ar(4*PRED_CONT,(18*PRED_HOR+4*PRED_CONT));
MatrixXf lbr((18*PRED_HOR+4*PRED_CONT),1);
MatrixXf ubr((18*PRED_HOR+4*PRED_CONT),1);
real_t Hq[4*PRED_CONT*4*PRED_CONT];
real_t ftq[PRED_CONT*4];
real_t Arq[4*PRED_CONT*(18*PRED_HOR+4*PRED_CONT)];
real_t ubrq[(18*PRED_HOR+4*PRED_CONT)];
real_t lbrq[(18*PRED_HOR+4*PRED_CONT)];
real_t xOpt[4];

int nWSR;
//std::chrono::steady_clock::time_point last;
/* Exported functions definitions --------------------------------------------*/

MpcBirotor::MpcBirotor() {

	/*Initialization of all MPC variable*/
	s=18;
	p=4;
	q=18;
	N=PRED_HOR;
	M=PRED_CONT;
	k=0;
	frequency=M_PI/20;
	ts=0.012;
	/*Trajetoria*/
	trajectory=new ReferenceTrajectory(ts);
	//Initialize the mathematics model
	Model=new AircraftModel();
	MatrixXf SumRho(18,18);
	MatrixXf SumLambda(4,4);
	MatrixXf TerminalCost(18,18);
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

	deltaxsiant.setZero();
	xsiant.setZero();
	options.printLevel=PL_NONE;
	qp.setOptions( options );
//	last=std::chrono::steady_clock::now();
}

MpcBirotor::~MpcBirotor() {
	// TODO Auto-generated destructor stub
}

Eigen::MatrixXf MpcBirotor::Controler(Eigen::MatrixXf states,bool stop){
	Eigen::MatrixXf u(4,1);

	if (stop){
		xsiant.setZero();
		xsi.setZero();
		deltaxsiant.setZero();
		deltaxsi.setZero();
		k=0;
		qp.reset();
		u.setZero();
	}else{
		//Vectors of reference trajectory and control
		xs<<0,0,states(2),states.block(3,0,5,1),0,0,0,states.block(11,0,5,1);
		xr=trajectory->TrajetoryReference_MPC(k);
		std::cout<<"z= "<<xr(2)<<std::endl;
		ar=trajectory->AcelerationReference(k);
		ur=Model->RefrenceControl(ar);
		//Variation of estates
		deltaxs=xs-xr;
		//Vector integration of error(Trapezoidal method)
		deltaxsi<<xs(2,0)-xr(2,0),xs(5,0)-xr(5,0);
		xsi=xsiant+ts*(deltaxsi+deltaxsiant)/2;
		// Error state vector
		deltaxs=xs-xr;
		// augmented error state vector
		xs_aumented<<deltaxs,xsi;

		//Refrence vector of future variation
		R.setZero();
		for (int i=0, Naux=1;i<N*q;i+=q, Naux++){
			R.block(i,0,q-2,1)=trajectory->TrajetoryReference_MPC(k+Naux)-
					trajectory->TrajetoryReference_MPC(k+Naux-1);
		}

	//	//Refrence control of future variation
		for(int i=0, Maux=1;i<M*p;i+=p, Maux++){
			if (k==0){
				Ur.block(i,0,p,1)=Model->RefrenceControl(trajectory->AcelerationReference(k+Maux-1));
			}else{
				Ur.block(i,0,p,1)=Model->RefrenceControl(trajectory->AcelerationReference(k+Maux-1))-
						Model->RefrenceControl(trajectory->AcelerationReference(k+Maux-2));
			}
		}

		/*Low control calculation*/
		Model->LinearModel(ar,ts,&Az,&Bz);
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
		ft=(2*((Q*xs_aumented-R).transpose()*Wy*G-Ur.transpose()*Wu));

		/*----------- Contraints---------------*/
		for(int i=0, Maux=1;i<M*p;i+=p, Maux++){
			dUmax.block(i,0,p,1)=umax-Model->RefrenceControl(trajectory->AcelerationReference(k+Maux-1));
			dUmin.block(i,0,p,1)=Model->RefrenceControl(trajectory->AcelerationReference(k+Maux-1))-umin;
		}

		Yr.setZero();
		for (int i=0, Naux=1;i<N*q;i+=q, Naux++){
			Yr.block(i,0,q-2,1)=trajectory->TrajetoryReference_MPC(k+Naux);
		}

		dYmax=Ymax-(Q*xs_aumented)-Yr;
		dYmin=(Q*xs_aumented)+Yr-Ymin;
		Ar.block(0,0,p*M,p*M)=Im;
		Ar.block(0,p,p*M,q*N)=G.transpose();

		lbr.block(0,0,p*M,1)=-dUmin;
		lbr.block(p,0,q*N,1)=-dYmin;
		ubr.block(0,0,p*M,1)=dUmax;
		ubr.block(p,0,q*N,1)=dYmax;

		//std::cout<<"Ar= ("<<Ar.rows()<<","<<Ar.cols()<<")"<<std::endl;
		//std::cout<<"lbr= ("<<lbr.rows()<<","<<lbr.cols()<<")"<<std::endl;
		//std::cout<<"ubr= ("<<ubr.rows()<<","<<ubr.cols()<<")"<<std::endl;

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

		//Variable update
		xsiant=xsi;
		deltaxsiant=deltaxsi;

		qp.reset();
		k++;
	}
	return u;
}
/* Private functions ------------------------------------------------------- */
Eigen::MatrixXf MpcBirotor::Pow(Eigen::MatrixXf matrix, int power){
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
