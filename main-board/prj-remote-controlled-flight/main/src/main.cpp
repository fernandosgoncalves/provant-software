/*
 *-----------------------------------------------------------------------------
 *  Filename:    main.cpp
 *-----------------------------------------------------------------------------
 *     ___                           _
 *    / _ \_ __ ___/\   /\__ _ _ __ | |_
 *   / /_)/ '__/ _ \ \ / / _` | '_ \| __|
 *  / ___/| | | (_) \ V / (_| | | | | |_
 *  \/    |_|  \___/ \_/ \__,_|_| |_|\__|
 *
 *-----------------------------------------------------------------------------
 */

#include <iostream>

#include "DataProcessingManager.h"
#include "ContinuousControlManager.h"
#include "CommLowLevelManager.h"

// STL
//#include <iostream>

// Boost
#include <boost/thread/thread.hpp>
#include <unistd.h>
#include <sched.h>
#include <cstdio>

class mainManager {
public:
    mainManager() {}
    ~mainManager() {}

};
//static boost::asio::io_service io;
int main(int argc, char ** argv) {
	int policy;
	pthread_t threadID;
	struct sched_param param;

    DataProcessingManager DataProcessing("DataProcessing:Manager"); //Thread de processamento de dados
    ContinuousControlManager   ContinuousControl("ContinuousControl:Manager");  //Thread de controle
    CommLowLevelManager        CommLowLevel("CommLowLevel:Manager");  //Thread de comunicacao com low level

    //Definição das interfaces, atribuição do envio das mensagens
    CommLowLevel.interface->q_atitude_out_ = &ContinuousControl.interface->q_atitude_in; //Atitude enviada do Low Level para o controle
    CommLowLevel.interface->q_position_out_ = &ContinuousControl.interface->q_position_in; //Posição enviada do Low Level para o controle
    CommLowLevel.interface->q_servos_out_ = &ContinuousControl.interface->q_servos_in; //Posição dos servos enviada do Low Level para o controle
    CommLowLevel.interface->q_debug_out_= &ContinuousControl.interface->q_debug_in; //Mensagens de debug
    CommLowLevel.interface->q_rc_out_= &ContinuousControl.interface->q_rc_in; //Dados do radio controle
    CommLowLevel.interface->q_status_out_= &ContinuousControl.interface->q_status_in; //
    ContinuousControl.interface->q_actuation_out_ = &CommLowLevel.interface->q_actuation_in; //Dados de atuação enviados para o Low Level

    CommLowLevel.interface->q_atitude2_out_ = &DataProcessing.interface->q_atitude_in; //Aqui
    CommLowLevel.interface->q_position2_out_ = &DataProcessing.interface->q_position_in; //Aqui
    CommLowLevel.interface->q_servos2_out_ = &DataProcessing.interface->q_servos_in; //Aqui
    CommLowLevel.interface->q_debug2_out_= &DataProcessing.interface->q_debug_in; //Aqui
    CommLowLevel.interface->q_rc2_out_= &DataProcessing.interface->q_rc_in; //Aqui
    CommLowLevel.interface->q_actuation2_out_ = &DataProcessing.interface->q_actuation_in; //Aqui
    CommLowLevel.interface->q_status2_out_= &ContinuousControl.interface->q_status_in; //Aqui

    boost::thread th1( boost::bind( &CommLowLevelManager::Run, CommLowLevel)); //AQUI


	boost::thread th2( boost::bind( &ContinuousControlManager::Run, ContinuousControl)); //AQUI


	boost::thread th3( boost::bind( &DataProcessingManager::Run, DataProcessing)); //AQUI

    th3.join();
    th2.join();
    th1.join();
}
