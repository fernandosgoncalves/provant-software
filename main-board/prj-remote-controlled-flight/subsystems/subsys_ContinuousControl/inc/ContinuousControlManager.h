/*
 *-----------------------------------------------------------------------------
 *  Filename:    ContinuousControlManager.h
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

#ifndef CONTINUOUS_CONTROL_MANAGER_H
#define CONTINUOUS_CONTROL_MANAGER_H

//Common Father
#include "AbstractModuleManager.h"
#include "Eigen/Dense"
#include "LQRControler.h"
#include "TESTActuator.h"
#include "math.h"
#include <boost/chrono.hpp>
#include <boost/system/config.hpp>

//Interface
#include "ContinuousControlInterface.h"
#include "MpcControler.h"
#include "MpcBirotor.h"
#include "MpcLoad.h"

//Modulos (nao precisa, mas pode incluir todos os headers dos submodulos)

/*! \brief Gerenciador padrão para módulos.
 *
 */
class ContinuousControlManager : public AbstractModuleManager
{
public:
    ContinuousControlManager(std::string name);
    ~ContinuousControlManager();

    // Interface do modulo
    ContinuousControlInterface * interface;

    /*! \brief Implementação do ponto de entrada do modulo.
     *
     *  Equivalente à uma \em main para cada módulo. A implementação deste método é altamente
     *  dependente da estrutura e funcionamento deste módulo.
     */
    void Run();

private:
    /*! \brief Implementação da inicializacao de cada módulo.
     *
     */
    void Init();
    //proVantProtocol PROVANT;

    /***** Atributos e metodos especificos da topologia do modulo *****/

    // Tempo de amostragem para loop principal
    int ms_sample_time;
    MPC::MpcControler * mpc;
    MPCLOAD::MpcLoad * mpcload;
    MPCBirotor::MpcBirotor * mpcbirotor;
    LQR::LQRControler * lqr;
    TEST::TESTActuator * test;
    boost::chrono::system_clock::time_point start;
    boost::chrono::system_clock::time_point last_start;
    // Nome do modulo
    std::string name_;
    typedef boost::chrono::milliseconds ms;

    /*! \brief Callback de recebimento de mensagem
     *
     *  Poderia ser definido em algum submodulo - depende da caracteristica desejada.
     *  Neste exempolo, o funcionamento geral apenas é disparado com a chegada de alguma mensagem.
     */
    void inboxCallback();

 };

#endif // CONTINUOUS_CONTROL_MANAGER_H
