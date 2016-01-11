/*
 *-----------------------------------------------------------------------------
 *  Filename:    CommLowLevelataManager.cpp
 *  Implementação do gerenciador e instanciador de submodulos.
 *-----------------------------------------------------------------------------
 *     ___                           _
 *    / _ \_ __ ___/\   /\__ _ _ __ | |_
 *   / /_)/ '__/ _ \ \ / / _` | '_ \| __|
 *  / ___/| | | (_) \ V / (_| | | | | |_
 *  \/    |_|  \___/ \_/ \__,_|_| |_|\__|
 *
 *-----------------------------------------------------------------------------
 *
 */

#include "ContinuousControlManager.h"
#include "proVantTypes.h"

//Internal
#include "debug.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>



using namespace Eigen;
using namespace loadmodel;
using namespace std;

ContinuousControlManager::ContinuousControlManager(std::string name) :
    interface(new ContinuousControlInterface("ContinuousControl:Interface")),
    // sm1(new SubModule1), // talvez fosse mais interessante construir os submodulos no init
    ms_sample_time(12),
    name_(name)
{
	//mpc=new MPC::MpcControler();
	//mpcload=new MPCLOAD::MpcLoad();
	//mpcbirotor=new MPCBirotor::MpcBirotor();
	//mpc=new MPC::MpcControler();
	lqr=new LQR::LQRControler();
	//test= new TEST::TESTActuator();
}

ContinuousControlManager::~ContinuousControlManager()
{

}

void ContinuousControlManager::Init()
{
    //PROVANT.init("/dev/ttyUSB0", 460800);
    //DEBUG(LEVEL_INFO, "Connected");

    // DEBUG(DEBUG_INFO, "Initializing: ") << __func__ ;

    /// Linkando os submodulos à interface do modulo
    // sm1->interface = interface;

    /// Conectar os submodulos, (caso eles se comuniquem por mensagem)
    /// ...

    /// Neste caso, conectar a notificacao da interface com o callback da classe
    // interface->q_in.notification.connect( boost::bind(&DataProcessingManager::inboxCallback, this) );

}

void ContinuousControlManager::Run()
{
    Init();
    // Algumas variaveis... 
    proVant::atitude atitude;
    proVant::position position;
    proVant::servos_state servos;
    proVant::controlOutput actuation;
    proVant::debug debug;
    proVant::rcNormalize rcNormalize;
    proVant::status status;
    int i=0;
    // Matrix class
    MatrixXf xs(16,1);
    MatrixXf ref(3,1);
    MatrixXf channels(4,1);
    MatrixXf u(4,1);
    start = boost::chrono::system_clock::now();
    last_start = boost::chrono::system_clock::now();

    for (int j=0;j<7;j++)
    	rcNormalize.normChannels[j]=0;
    // Loop principal!

    while(1) {
    	start= boost::chrono::system_clock::now();
    	auto sample_time = boost::chrono::duration_cast<boost::chrono::microseconds>(start-last_start);
    	last_start=start;

    	if(interface->pop(atitude, &interface->q_atitude_in)){
    		/*Atitude*/
//    		cout<<"Atitude Received C"<<endl;
//    		cout<<"Roll= "<<atitude.roll<<endl;
//    		cout<<"Pitch= "<<atitude.pitch<<endl;
//    		cout<<"Yaw= "<<atitude.yaw<<endl;
//    		cout<<"dotRoll= "<<atitude.dotRoll<<endl;
//    		cout<<"dotPitch= "<<atitude.dotPitch<<endl;
//    		cout<<"dotYaw= "<<atitude.dotYaw<<endl;
    	}
    	if(interface->pop(position, &interface->q_position_in)){
    		/*Position*/
//    		cout<<"Position Received C"<<endl;
//    		cout<<"X= "<<position.x<<endl;
//    		cout<<"Y= "<<position.y<<endl;
//    		cout<<"Z= "<<position.z<<endl;
//    		cout<<"dotX= "<<position.dotX<<endl;
//    		cout<<"dotY= "<<position.dotY<<endl;
//    		cout<<"dotZ= "<<position.dotZ<<endl;
    	}
    	if(interface->pop(servos, &interface->q_servos_in)){
    		/*Servos*/
//    		cout<<"Servos Received C"<<endl;
//    		cout<<"Alphal= "<<servos.alphal<<endl;
//    		cout<<"Alphar= "<<servos.alphar<<endl;
//    		cout<<"dotAlphal= "<<servos.dotAlphal<<endl;
//    		cout<<"dotAlphar= "<<servos.dotAlphar<<endl;
    	}
    	if(interface->pop(debug, &interface->q_debug_in)){
    		/*Servos*/
    	//  cout<<"Servos Received C"<<endl;
    	//  cout<<"Alphal= "<<servos.alphal<<endl;
    	//  cout<<"Alphar= "<<servos.alphar<<endl;
    	//  cout<<"dotAlphal= "<<servos.dotAlphal<<endl;
    	//  cout<<"dotAlphar= "<<servos.dotAlphar<<endl;
    	}
    	if(interface->pop(rcNormalize, &interface->q_rc_in)){
    	   	/*Servos*/
    	//	cout<<"channel[1]"<<rcNormalize.normChannels[1]<<endl;
    	//	cout<<"Alphar= "<<servos.alphar<<endl;
    	//  cout<<"dotAlphal= "<<servos.dotAlphal<<endl;
    	//  cout<<"dotAlphar= "<<servos.dotAlphar<<endl;
    	}
    	if(interface->pop(status, &interface->q_status_in)){
    	/*Servos*/
    	//	cout<<"channel[1]"<<rcNormalize.normChannels[1]<<endl;
    	//	cout<<"Alphar= "<<servos.alphar<<endl;
    	//  cout<<"dotAlphal= "<<servos.dotAlphal<<endl;
    	//  cout<<"dotAlphar= "<<servos.dotAlphar<<endl;
    	}
    	channels.setZero();
    	channels<<rcNormalize.normChannels[0],rcNormalize.normChannels[1],rcNormalize.normChannels[2],rcNormalize.normChannels[3];
    	xs.setZero();
    	xs<<position.x,position.y,position.z,atitude.roll,atitude.pitch,atitude.yaw,servos.alphar,servos.alphal
    			,position.dotX,position.dotY,position.dotZ,atitude.dotRoll,atitude.dotPitch,atitude.dotYaw,servos.dotAlphar,servos.dotAlphal;
    	ref<<(float)debug.debug[0]/100.0,(float)debug.debug[1]/100.0,(float)debug.debug[2]/100.0;
    	std::cout<<ref<<std::endl;
    	//u=mpc->Controler(xs);

    	u=lqr->Controler(xs,ref,status.stop);
    	//u=mpcload->Controler(xs);
    	//u=mpcbirotor->Controler(xs,status.stop);
    	//u=test->Controler(channels);


    	/////////////////////////////////
    	//std::cout<<u<<std::endl;
    	actuation.escRightNewtons=u(0,0);
    	actuation.escLeftNewtons=u(1,0);
    	actuation.servoRight=u(2,0);
    	actuation.servoLeft=u(3,0);
    	actuation.escLeftSpeed=0;
    	actuation.escRightSpeed=0;


    	interface->push(actuation, interface->q_actuation_out_);
    	interface->push(actuation, interface->q_actuation2_out_);


    	std::cout << "sp_thread2: " << sample_time.count()<< " microseconds." << std::endl;
    	i++;
    	auto elapsed = boost::chrono::duration_cast<boost::chrono::microseconds>(boost::chrono::system_clock::now()-start);
    	boost::this_thread::sleep_until(boost::chrono::system_clock::now() + boost::chrono::microseconds((ms_sample_time*1000)-elapsed.count()));
    }
}

void ContinuousControlManager::inboxCallback()
{
    DEBUG(LEVEL_INFO, "Got message! ") << name_;
}

