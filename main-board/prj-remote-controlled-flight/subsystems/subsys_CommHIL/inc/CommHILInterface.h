/*
 *-----------------------------------------------------------------------------
 *  Filename:    CommHILInterface.h
 *  Implementação da interface para algum modulo especifico.
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

/*** Renomear .cpp e o .h com o nome do módulo + "Interface" (ex. DataProcessingInterface) ***/

#ifndef COMMUNICATION_HIL_INTERFACE_H
#define COMMUNICATION_HIL_INTERFACE_H

//Father
#include "AbstractMessageInterface.h"

#include "proVantTypes.h"

/*! \brief Implementação da interface entre os modulos.
 *
 *  A interface contem um MessageQueue para cada inbox que possuir (q_in, no caso),
 *  e ponteiros para o inbox da interface com o qual ela se conecta (q_out).
 */
class CommHILInterface : public AbstractMessageInterface
{
public:
    CommHILInterface(std::string name) :
    //q_out_(NULL),
    q_position_out_(NULL),
	q_atitude_out_(NULL),
	q_servos_out_(NULL),
	//q_debug_out_(NULL),
	//q_rc_out_(NULL),
    name_(name) { }

    ~CommHILInterface();

    // Inboxes
    // CommLowLevelManager consuming
    MsgQueue<proVant::controlOutput> q_actuation_in;

    MsgQueue<proVant::atitude> q_atitude_in_;
    MsgQueue<proVant::position> q_position_in_;
    MsgQueue<proVant::servos_state> q_servos_in_;
    //MsgQueue<proVant::debug> q_debug_in_;
    //MsgQueue<proVant::rcNormalize> q_rc_in_;
    //MsgQueue<proVant::status> q_status_in_;


    // Outboxes (ponteiros para inboxes alheios)
    //ControlOutput Producing
    MsgQueue<proVant::atitude>* q_atitude_out_;
    MsgQueue<proVant::position>* q_position_out_;
    MsgQueue<proVant::servos_state>* q_servos_out_;
    //MsgQueue<proVant::debug>* q_debug_out_;
    //MsgQueue<proVant::rcNormalize>* q_rc_out_;
    //MsgQueue<proVant::status>* q_status_out_;

    MsgQueue<proVant::controlOutput>* q_actuation_out;

private:
    std::string name_;

};

#endif // DATA_PROCESSING_INTERFACE_H
