/*
 *-----------------------------------------------------------------------------
 *  Filename:    CommHILManager.cpp
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

#include "CommHILManager.h"

using namespace std;

CommHILManager::CommHILManager(std::string name) :
		interface(new CommHILInterface("CommHIL:Interface")),
		// sm1(new SubModule1), // talvez fosse mais interessante construir os submodulos no init
		ms_sample_time(6), name_(name) {

}

CommHILManager::~CommHILManager() {

}

void CommHILManager::Init() {
	//Create TCP/IP Objects to connect with Matlab Simulink
	comm1.init(8081, "192.168.0.117");

	comm2.init(8082, "192.168.0.117");

	comm1.connect();
	comm2.connect();

	cout << "Connections ok!" << endl;

	// Linkando os submodulos à interface do modulo
	// sm1->interface = interface;

	// Conectar os submodulos, (caso eles se comuniquem por mensagem)
	// ...

	// Neste caso, conectar a notificacao da interface com o callback da classe
	// interface->q_in.notification.connect( boost::bind(&DataProcessingManager::inboxCallback, this) );
}

void CommHILManager::Run() {
	Init();
	// Algumas variaveis...
	proVant::controlOutput actuation_aux;
	proVant::controlOutput actuation;
	proVant::states vantStates_aux;
	proVant::states vantStates;

	actuation.escRightNewtons = 0;
	actuation.escLeftNewtons = 0;
	actuation.escRightSpeed = 0;
	actuation.escLeftSpeed = 0;
	actuation.servoRight = 0;
	actuation.servoLeft = 0;

	// Loop principal!
	while (1) {
		start = boost::chrono::system_clock::now();
		auto sample_time = boost::chrono::duration_cast
				< boost::chrono::microseconds > (start - last_start);
		last_start = start;

		/*Recive states from Model*/
		vantStates_aux = comm2.receiveData();

		if (vantStates_aux.atitudeVant.dotPitch != 0
				|| vantStates_aux.atitudeVant.dotRoll != 0
				|| vantStates_aux.atitudeVant.dotYaw != 0
				|| vantStates_aux.atitudeVant.pitch != 0
				|| vantStates_aux.atitudeVant.roll != 0
				|| vantStates_aux.atitudeVant.yaw != 0) {
			vantStates.atitudeVant = vantStates_aux.atitudeVant;
		}

		if (vantStates_aux.positionVant.dotX != 0
				|| vantStates_aux.positionVant.dotY != 0
				|| vantStates_aux.positionVant.dotZ != 0
				|| vantStates_aux.positionVant.x != 0
				|| vantStates_aux.positionVant.y != 0
				|| vantStates_aux.positionVant.z != 0) {
			vantStates.positionVant = vantStates_aux.positionVant;
		}

		if (vantStates_aux.servos_stateVant.alphal != 0
				|| vantStates_aux.servos_stateVant.alphar != 0
				|| vantStates_aux.servos_stateVant.dotAlphal != 0
				|| vantStates_aux.servos_stateVant.dotAlphar != 0) {
			vantStates.servos_stateVant = vantStates_aux.servos_stateVant;
		}

		interface->push(vantStates.positionVant, interface->q_position_out_);
		interface->push(vantStates.atitudeVant, interface->q_atitude_out_);
		interface->push(vantStates.servos_stateVant, interface->q_servos_out_);

		//Send Control to Discovery
		if (interface->pop(actuation, &interface->q_actuation_in)) {
			/*Control*/
			//Implement de tcp/ip control send
			comm1.sendData(actuation);
		}

		//Elapsed time code
//    	auto end = std::chrono::steady_clock::now();
//    	auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
//    	std::cout << "It took me " << (float)(elapsed.count()/1000) << " miliseconds." << std::endl;

		std::cout << "sp_thread4: " << sample_time.count() << " microseconds."
				<< std::endl;
		auto elapsed = boost::chrono::duration_cast
				< boost::chrono::microseconds
				> (boost::chrono::system_clock::now() - start);
		boost::this_thread::sleep_until(
				boost::chrono::system_clock::now()
						+ boost::chrono::microseconds(
								(ms_sample_time * 1000) - elapsed.count()));
	}
}

void CommHILManager::inboxCallback() {
	DEBUG(LEVEL_INFO, "Got message! ") << name_;
}

