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

#include "CommLowLevelManager.h"
#include "proVantTypes.h"

//Internal
#include "debug.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <chrono>

using namespace std;

CommLowLevelManager::CommLowLevelManager(std::string name) :
    interface(new CommLowLevelInterface("CommLowLevel:Interface")),
    // sm1(new SubModule1), // talvez fosse mais interessante construir os submodulos no init
    ms_sample_time(6),
    name_(name)
{

}

CommLowLevelManager::~CommLowLevelManager()
{

}

void CommLowLevelManager::Init()
{
    //921600     460800
    PROVANT.init("/dev/ttyO1", 921600);
    DEBUG(LEVEL_INFO, "Connected 1");

    // DEBUG(DEBUG_INFO, "Initializing: ") << __func__ ;

    /// Linkando os submodulos à interface do modulo
    // sm1->interface = interface;

    /// Conectar os submodulos, (caso eles se comuniquem por mensagem)
    /// ...

    /// Neste caso, conectar a notificacao da interface com o callback da classe
    // interface->q_in.notification.connect( boost::bind(&DataProcessingManager::inboxCallback, this) );

}

void CommLowLevelManager::Run()
{
    Init();
    // Algumas variaveis... 
    proVant::atitude atitude, atitude_aux;
    proVant::position position, position_aux;
    proVant::servos_state servos, servos_aux;
    proVant::debug debug, debug_aux;
    proVant::rcNormalize rcNormalize, rcNormalize_aux;
    proVant::controlOutput actuation;
    proVant::controlOutput actuation2, actuation2_aux;
    proVant::status status;

    float data1[2]={};
    float data2[2]={};
    float data3[2]={};
    int flag=0;

    actuation.servoLeft=0;
    actuation.servoRight=0;
    actuation.escLeftNewtons=0;
    actuation.escRightNewtons=0;
    actuation.escLeftSpeed=0;
    actuation.escRightSpeed=0;
    // Loop principal!
    while(1) {
    	start= boost::chrono::system_clock::now();
    	auto sample_time = boost::chrono::duration_cast<boost::chrono::microseconds>(start-last_start);
    	last_start=start;

    	/*Recive states from Discovery*/
    	flag=PROVANT.updateData();

	#ifdef ENABLE_HIL
    	interface->pop(position, interface->q_position_in);
    	interface->pop(atitude, interface->q_atitude_in);
    	interface->pop(servos, interface->q_servos_in);
    	//interface->pop(debug, interface->q_debug_in);
    	//interface->pop(rcNormalize, interface->q_rc_in);
    	//interface->pop(status,interface->q_status_in);

    	//Send Control to Comptational Model
    	interface->pop(actuation, &interface->q_actuation_in
    	interface->push(actuation, interface->q_actuation_out_)
	else
    	atitude_aux = PROVANT.getVantData().getAtitude();
    	if (atitude_aux.dotPitch!=0 || atitude_aux.dotRoll!=0 || atitude_aux.dotYaw!=0 || atitude_aux.pitch!=0 || atitude_aux.roll!=0 || atitude_aux.yaw!=0){
    		atitude=atitude_aux;
    	}
    	position_aux= PROVANT.getVantData().getPosition();
    	if(position_aux.dotX!=0 || position_aux.dotY!=0 || position_aux.dotZ!=0 || position_aux.x!=0 || position_aux.y!=0 || position_aux.z!=0){
    		position=position_aux;
    	}
    	actuation2_aux=PROVANT.getVantData().getActuation();
    	if (actuation2_aux.escLeftNewtons!=0 || actuation2_aux.escRightNewtons!=0 || actuation2_aux.servoLeft!=0 || actuation2_aux.servoRight!=0){
    		actuation2=actuation2_aux;
    	}
    	servos_aux= PROVANT.getVantData().getServoState();    //Function made to save current as alpha and voltage as dotalpha
    	if(servos_aux.alphal!=0 || servos_aux.alphar!=0 || servos_aux.dotAlphal!=0 || servos_aux.dotAlphar!=0){
    		servos=servos_aux;
    	}
    	debug_aux=PROVANT.getVantData().getDebug();
    	if(debug_aux.debug[0]!=0 || debug_aux.debug[1]!=0 || debug_aux.debug[2]!=0){
    		debug=debug_aux;
    	}

    	//rcNormalize= PROVANT.getVantData().getNormChannels();
    	status=PROVANT.getVantData().getStatus();

    	//Send Control to Discovery
    	if(interface->pop(actuation, &interface->q_actuation_in)){
    		/*Control*/
    		data1[0]= actuation.servoLeft;
    		data1[1]= actuation.servoRight;
    		data3[0]= actuation.escLeftNewtons;
    		data3[1]= actuation.escLeftSpeed;
    		data2[0]= actuation.escRightNewtons;
    		data2[1]= actuation.escRightSpeed;
    		PROVANT.multwii2_sendControldataout(data1,data3,data2);
    		PROVANT.multwii_sendstack();
    	}

    	interface->push(debug, interface->q_debug_out_);
    	interface->push(rcNormalize, interface->q_rc_out_);
    	interface->push(status,interface->q_status_out_);

    	interface->push(debug, interface->q_debug2_out_);
    	interface->push(rcNormalize, interface->q_rc2_out_);
    	interface->push(actuation2, interface->q_actuation2_out_);
    	interface->push(status,interface->q_status2_out_);

	#endif


    	interface->push(position, interface->q_position_out_);
    	interface->push(atitude, interface->q_atitude_out_);
    	interface->push(servos, interface->q_servos_out_);

    	interface->push(position, interface->q_position2_out_);
    	interface->push(atitude, interface->q_atitude2_out_);
    	interface->push(servos, interface->q_servos2_out_);

    	//Elapsed time code
//    	auto end = std::chrono::steady_clock::now();
//    	auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
//    	std::cout << "It took me " << (float)(elapsed.count()/1000) << " miliseconds." << std::endl;

    	std::cout << "sp_thread1: " << sample_time.count()<< " microseconds." << std::endl;
    	auto elapsed = boost::chrono::duration_cast<boost::chrono::microseconds>(boost::chrono::system_clock::now()-start);
    	boost::this_thread::sleep_until(boost::chrono::system_clock::now() + boost::chrono::microseconds((ms_sample_time*1000)-elapsed.count()));
    }
}

void CommLowLevelManager::inboxCallback()
{
    DEBUG(LEVEL_INFO, "Got message! ") << name_;
}

