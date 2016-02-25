/*
 *-----------------------------------------------------------------------------
 *  Filename:    CommHILManager.h
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

/*** Renomear .cpp e o .h com o nome do módulo + "Module" (ex. ContinuousControlModule) ***/

#ifndef COMMUNICATION_HIL_MANAGER_H
#define COMMUNICATION_HIL_MANAGER_H

//Common Father
#include "AbstractModuleManager.h"

//Interface
#include "CommHILInterface.h"
#include "socketTCPIP.h"
#include "proVantTypes.h"

//Modulos (nao precisa, mas pode incluir todos os headers dos submodulos)
//#include "proVantProtocol.h"
//Internal
#include "debug.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <chrono>


/*! \brief Gerenciador padrão para módulos.
 *
 */
class CommHILManager: public AbstractModuleManager {
public:
	CommHILManager(std::string name);
	~CommHILManager();

	// Interface do modulo
	CommHILInterface * interface;
	void Run();

private:
	void Init();

	/***** Atributos e metodos especificos da topologia do modulo *****/
	socketTCPIP comm1;
	socketTCPIP comm2;
	// Tempo de amostragem para loop principal
	int ms_sample_time;
	boost::chrono::system_clock::time_point start;
	boost::chrono::system_clock::time_point last_start;

	// Nome do modulo
	std::string name_;

	/*! \brief Callback de recebimento de mensagem
	 *
	 *  Poderia ser definido em algum submodulo - depende da caracteristica desejada.
	 *  Neste exempolo, o funcionamento geral apenas é disparado com a chegada de alguma mensagem.
	 */
	void inboxCallback();

};

#endif // COMMUNICATION_HIL_MANAGER_H
