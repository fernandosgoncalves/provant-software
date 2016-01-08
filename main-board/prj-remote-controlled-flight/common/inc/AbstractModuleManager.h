/*
 *-----------------------------------------------------------------------------
 *  Filename:    AbstractModuleManager.h
 *  Gerenciador e instanciador de submodulos e interface para um dado modulo.
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

#ifndef ABSTRACT_MODULE_MANAGER_H
#define ABSTRACT_MODULE_MANAGER_H

// STL
#include <iostream>

//Boost
#include <boost/thread.hpp>
#include <boost/chrono.hpp>

/*! \brief Gerenciador padrão para módulos.
 *
 */
class AbstractModuleManager
{
public:
    AbstractModuleManager();
    ~AbstractModuleManager();

    /*! \brief Ponto de entrada do modulo.
     *
     *  Coloca submodulos em threads, se necessário. Roda o loop principal do programa.
     */
    virtual void Run() = 0;

private:
    /*! \brief Inicializacao de cada modulo.
     *
     *  Instancia os submodulos necessario e a interface do modulo.
     *  Coloca submodulos em threads, se necessário. É a função que deve ser chamada para
     *  instanciar todo o modulo.
     */
    virtual void Init() = 0;

 };

#endif // ABSTRACT_MODULE_MANAGER_H
