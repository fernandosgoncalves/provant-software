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

#include "ContinuousControlManager.h"
#include "DataProcessingManager.h"
#include "CommLowLevelManager.h"
#ifdef ENABLE_HIL
#include "CommHILManager.h"
#endif

// Boost
#include <boost/thread/thread.hpp>

#include <iostream>
#include <unistd.h>
#include <sched.h>
#include <cstdio>

class mainManager {
public:
	mainManager() {
	}
	~mainManager() {
	}

};
//static boost::asio::io_service io;
int main(int argc, char ** argv) {
	int policy;
	pthread_t threadID;
	struct sched_param param;

	ContinuousControlManager ContinuousControl("ContinuousControl:Manager"); //Thread de controle
	DataProcessingManager DataProcessing("DataProcessing:Manager"); //Thread de processamento de dados
	CommLowLevelManager CommLowLevel("CommLowLevel:Manager"); //Thread de comunicacao com low level
#ifdef ENABLE_HIL
			CommHILManager CommHIL("CommHIL:Manager"); //Thread de comunicacao com HIL
#endif

	//Definição das interfaces, atribuição do envio das mensagens
	CommLowLevel.interface->q_atitude_out_ =
			&ContinuousControl.interface->q_atitude_in; //Atitude enviada do Low Level para o controle
	CommLowLevel.interface->q_position_out_ =
			&ContinuousControl.interface->q_position_in; //Posição enviada do Low Level para o controle
	CommLowLevel.interface->q_servos_out_ =
			&ContinuousControl.interface->q_servos_in; //Posição dos servos enviada do Low Level para o controle
	CommLowLevel.interface->q_debug_out_ =
			&ContinuousControl.interface->q_debug_in; //Mensagens de debug
	CommLowLevel.interface->q_rc_out_ = &ContinuousControl.interface->q_rc_in; //Dados do radio controle
	CommLowLevel.interface->q_status_out_ =
			&ContinuousControl.interface->q_status_in; //

	ContinuousControl.interface->q_actuation_out_ =
			&CommLowLevel.interface->q_actuation_in; //Dados de atuação enviados para o Low Level

	CommLowLevel.interface->q_atitude2_out_ =
			&DataProcessing.interface->q_atitude_in; //Aqui
	CommLowLevel.interface->q_position2_out_ =
			&DataProcessing.interface->q_position_in; //Aqui
	CommLowLevel.interface->q_servos2_out_ =
			&DataProcessing.interface->q_servos_in; //Aqui
	CommLowLevel.interface->q_debug2_out_ =
			&DataProcessing.interface->q_debug_in; //Aqui
	CommLowLevel.interface->q_rc2_out_ = &DataProcessing.interface->q_rc_in; //Aqui
	CommLowLevel.interface->q_actuation2_out_ =
			&DataProcessing.interface->q_actuation_in; //Aqui
	CommLowLevel.interface->q_status2_out_ =
			&ContinuousControl.interface->q_status_in; //Aqui

#ifdef ENABLE_HIL
	CommLowLevel.interface->q_actuation_out_ = &CommHIL.interface->q_actuation_in;

	CommHIL.interface->q_atitude_out_ = &CommLowLevel.interface->q_atitude_in;
	CommHIL.interface->q_position_out_ = &CommLowLevel.interface->q_position_in;
	CommHIL.interface->q_servos_out_ = &CommLowLevel.interface->q_servos_in;
#endif

	boost::thread th2(
			boost::bind(&ContinuousControlManager::Run, ContinuousControl)); //AQUI
	boost::thread th3(boost::bind(&DataProcessingManager::Run, DataProcessing)); //AQUI
	boost::thread th1(boost::bind(&CommLowLevelManager::Run, CommLowLevel)); //AQUI
#ifdef ENABLE_HIL
			std::cout << "COMM HIL Thread created!";
			boost::thread th4( boost::bind( &CommHILManager::Run, CommHIL)); //AQUI
#endif

	th1.join();
	th2.join();
	th3.join();
#ifdef ENABLE_HIL
	th4.join();
#endif
}
