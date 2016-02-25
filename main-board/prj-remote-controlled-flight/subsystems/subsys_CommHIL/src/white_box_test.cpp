/*
 *-----------------------------------------------------------------------------
 *  Filename:    main.cpp
 *
 *-----------------------------------------------------------------------------
 *     ___                           _
 *    / _ \_ __ ___/\   /\__ _ _ __ | |_
 *   / /_)/ '__/ _ \ \ / / _` | '_ \| __|
 *  / ___/| | | (_) \ V / (_| | | | | |_
 *  \/    |_|  \___/ \_/ \__,_|_| |_|\__|
 *
 *-----------------------------------------------------------------------------
 *
 *  PROVANT - SUBSYS Template V1
 *
 *  Template C++ para projeto de um susbistema do VANT.
 *
 *  Implementa a classe abstrata de interface do módulo, a unidade de white-box
 *  e de black-box testing, além de algumas classes de teste do modulo.
 *
 *  provant_subsys_template_v1/             Pasta com subsistema
 *  |-  black_box_test.cpp                  Tem uma main para testes caixa preta
 *  |-  doc/
 *  |-  build/                              Modulo compilado.
 *  |-  module_template/
 *      |- src/
 *      |- inc/
 *      CMakeLists.txt
 */

/* TODO:
 *
 *      - Implementar container (queue de uma variavel so)
 *  - Implementar as perspectivas de teste
 *  - Chamar CMake recursivamente (para os varios modulos a partir de um central)
 *
 *  - Para o ARM, implementar o ARM_HAL como singleton também -> system init, not lazy
 */

// STL
#include <iostream>

// Boost
#include <boost/thread/thread.hpp>

// Test framework
// #include ...

// Project Includes
#include "CommHILManager.h"
#include "debug.h"

// Module and test dummy interface
CommHILManager *   module1;
CommHILInterface * dummyIface;

void testThread() {
    std::string msg("Test message!");
    proVant::atitude atitude, atd;
    atitude.roll = 0;
    atitude.pitch = 0;
    atitude.yaw = 0;
    atitude.dotRoll = 0;
    atitude.dotPitch = 0;
    atitude.dotYaw = 0;
    int i = 0, n;

    while(true) {
	atitude.roll = i;
        //dummyIface->push(msg, dummyIface->q_out_);
	//DEBUG(LEVEL_INFO, "Send message");
        dummyIface->push(atitude, dummyIface->q_atitude_out_);
	i++;
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    }
}


int main()
{
    // Create module
    module1 = new CommHILManager("Module1:Manager");

    //Dummy test interface
    dummyIface = new CommHILInterface("DummyIface");

    // Debug
    DEBUG(LEVEL_INFO, "Created!");

    // Connect
    //dummyIface->q_out_ = &module1->interface->q_in;
    //module1->interface->q_atitude_out_ = &dummyIface->q_atitude_in_;
    //dummyIface->q_atitude_out_ = &module1->interface->q_atitude_in;

    // Run module
    boost::thread th1( boost::bind( &CommHILManager::Run, module1) );
    boost::thread th2( testThread ) ;

    // Wait for threads to end
    th1.join();
    th2.join();
    return 0;
}

