/**
  ******************************************************************************
  * @file    modules/common/pv_typedefs.h
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    10-February-2014
  * @brief   Definições de tipos e estruturas de mensagens para o projeto.
  *
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PV_TYPEDEFS_H
#define PV_TYPEDEFS_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_conf.h"

#ifdef __cplusplus
 extern "C" {
#endif

 /** @addtogroup Common_Components
   * @{
   */

 /** @addtogroup PV_Typedefs
   *
   * Definições de tipagem e estruras de mensagem para o projeto.
   * @{
   */

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/** \brief Um par de floats. */
typedef struct
{
  float accRaw[3];
  float gyrRaw[3];
  float magRaw[3];
  float temp;
  unsigned int  sampleTime;
} pv_type_imuOutput;

/** \brief Informações dos dados do controle remoto.*/
typedef struct 
{
  int  joystick[4];
  unsigned int vrPot;
  bool aButton;
  bool bButton;
  unsigned int  sampleTime;
} pv_type_receiverOutput;

/** \brief Informações do sonar*/
typedef struct 
{
  float altitude;
  unsigned int  sampleTime;
} pv_type_sonarOutput;

/** \brief Tipo do ESC com informações.*/
typedef struct 
{
  char  ID;
  float angularSpeed;
  float current;
  float voltage;
  float rpm;
  unsigned int  sampleTime;
} pv_type_escOutput;

/** \brief Dados do servo*/
typedef struct 
{
  char  ID;
  float angularSpeed;
  float angle;
  float torque;
  float rpm;
  unsigned int  sampleTime;
} pv_type_servoOutput;

/** \brief Estrutura para orientação do VANT.*/
typedef struct
{
  float roll, pitch, yaw;
  float dotRoll, dotPitch, dotYaw;
} pv_type_datapr_attitude;

/** \brief Estrutura para refrencia da orientação do VANT.*/
typedef struct
{
  float refroll, refpitch, refyaw;
  float refdotRoll, refdotPitch, refdotYaw;
} pv_type_datapr_attitude_refrence;

/** \brief Estrutura para posição do VANT.*/
typedef struct
{
  float x, y, z;
  float dotX, dotY, dotZ;
} pv_type_datapr_position;

/** \brief Estrutura para referencia da posição do VANT.*/
typedef struct
{
  float refx, refy, refz;
  float refdotX, refdotY, refdotZ;
} pv_type_datapr_position_reference;

/** \brief Estrutura para dados de atuação.*/
typedef struct
{
  bool  servoTorqueControlEnable;
  float servoLeft;
  float servoRight;
  float escRightSpeed;
  float escLeftSpeed;
} pv_type_actuation;

/** \brief Estrutura para dados de comportamento.*/
typedef struct
{
  float rpy[3];
  float drpy[3];
  float xyz[3];
  float dxyz[3];
} pv_type_vantBehavior;

/** \brief Estrutura para dados de comportamento.*/
typedef struct
{
  int timeStamp;
  bool validity;
  float lat;
  bool latDirection;
  float lon;
  bool lonDirection;
  float speedOverGround;
  float trueCourse;
  int dateStamp;
  float variation;
  bool variationDirection;
  char ModeIndicator;
  unsigned int sampleTime;
} pv_type_gpsOutput;

/** \brief Integral do erro dos angulos de orientacao VANT.*/
typedef struct {
	float z, roll, pitch, yaw;
} pv_type_stability_error;

typedef struct {
	float x, y, z, yaw;
} pv_type_pathtrack_error;

typedef struct {
        int16_t iJogData : 15;
        uint8_t uiReserved1 : 1;
        uint8_t uiStop : 1;
        uint8_t uiMode : 1; //1 : Speed Control
        uint8_t uiLed : 3; //Green, Blue, Red
        uint8_t uiJogInvalid : 1;
        uint8_t uiReserved2 : 2;
        uint8_t ucID : 8;
        uint8_t ucJogTime_ms;
} pv_ijog_herkulex;

typedef struct {
        int16_t iJogData : 15;
        uint8_t uiReserved1 : 1;
        uint8_t uiStop : 1;
        uint8_t uiMode : 1; //1 : Speed Control
        uint8_t uiLed : 3; //Green, Blue, Red
        uint8_t uiJogInvalid : 1;
        uint8_t uiReserved2 : 2;
        uint8_t ucID : 8;
} pv_sjog_herkulex;

typedef struct
{
        uint32_t heartBeat;
        int32_t  pwm;
        float angularSpeed;
        float position;
        uint32_t status;
} pv_msg_servo;

/* Exported messages ---------------------------------------------------------*/

/** \brief Estrutura de mensagens para os dois RX24f e ESCs.
 *
 * Pacote prevê passagem de referências de velocidade para os ESCs em <b>rad/s</b>,
 * referências agulares em <b>rad</b> e torque em <b>N.m</b>. Para o servo, o módulo de
 * IO automaticamente alternará entre referências de ângulo para torque se a flag
 *  \b servoTorqueControlEnable estiver setada.
 */

/** \brief Estrutura de mensagem de saida da estrutura thread input.*/
typedef struct
{
  pv_type_imuOutput      imuOutput;
  pv_type_receiverOutput receiverOutput;
  pv_type_sonarOutput    sonarOutput;
  pv_type_escOutput      escOutput;
  pv_type_servoOutput    servoOutput;
  pv_type_datapr_attitude attitude;
  pv_type_datapr_position position;
  pv_type_datapr_attitude_refrence  attitude_reference;
  pv_type_datapr_position_reference position_refrence;
  unsigned int cicleTime;
  unsigned int heartBeat;
  bool init;
  bool securityStop;
  bool flightmode;
  bool enableintegration;
} pv_msg_input;

/** \brief Estrutura de mensagem de saida da estrutura thread de controle.*/
typedef struct
{
  pv_type_actuation    actuation;
  pv_type_vantBehavior vantBehavior;
  unsigned int cicleTime;
  unsigned int heartBeat;
} pv_msg_controlOutput;

/** \brief Estrutura de mensagem de saida da estrutura thread de gps.*/
typedef struct
{
  pv_type_gpsOutput gpsOutput;
  unsigned int cicleTime;
  unsigned int heartBeat;
} pv_msg_gps;

/** \brief Estrutura de mensagem de saida da estrutura thread de state machine.*/
typedef struct
{
  unsigned int cicleTime;
  unsigned int heartBeat;
} pv_msg_sm;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* Header-defined wrapper functions ----------------------------------------- */

 /**
   * @}
   */
 /**
   * @}
   */

#ifdef __cplusplus
}
#endif

#endif //PV_TYPEDEFS_H

