/**
  ******************************************************************************
  * @file    modules/rc/pv_module_rc.c
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    30-November-2013
  * @brief   ...
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "c_io_servos.h"

/** @addtogroup ProVANT_app
  * @{
  */

/** @addtogroup app_co
  * \brief Módulo de administraçao de servos.
  *
  * Definição do módulo.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define USART_BAUDRATE     666666
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
char idServoRight;
char idServoLeft;
float alphaRight;
float alphaLeft;
float dotAlphaRight;
float dotAlphaLeft;
uint8_t status_errorRight;
uint8_t status_errorLeft;
uint8_t status_detailRight;
uint8_t status_detailLeft;
USART_TypeDef *USARTn = USART1;
/* Inboxes buffers */
pv_type_actuation    iActuation;
/* Outboxes buffers*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions definitions --------------------------------------------*/

/** \brief Inicializacao do módulo de controle + output.
  *
  * Instancia as Queues de comunicação inter-thread, inicializa a pinagem necessária para
  * os perifericos e aloca o que for necessário para as equações de controle.
  * @param  None
  * @retval None
  */

void c_io_servos_init()
{
	status_errorRight=0;
	status_errorLeft=0;
	status_detailRight=0;
	status_detailLeft=0;
	#ifdef SERVO_RX24F
		/* Inicializar os servos */
		idServoRight=2;
		idServoLeft=1;
		c_io_rx24f_init(1000000);
		c_common_utils_delayms(2);
		c_io_rx24f_setSpeed(idServoLeft, 200);
		c_io_rx24f_setSpeed(idServoRight, 200);
		c_common_utils_delayms(2);
		/* CCW Compliance Margin e CCW Compliance margin */
		c_io_rx24f_write(idServoLeft, 0x1A,0x03);
		c_io_rx24f_write(idServoLeft, 0x1B,0x03);
		c_io_rx24f_write(idServoRight, 0x1A,0x03);
		c_io_rx24f_write(idServoRight, 0x1B,0x03);
		c_common_utils_delayms(2);
		c_io_rx24f_move(idServoLeft, 130);
		c_io_rx24f_move(idServoRight, 150);
		c_common_utils_delayms(100);
	#endif
	#ifdef SERVO_HERKULEX
		/* Inicia a usart */
		idServoRight=253;
		idServoLeft=150;
		c_io_herkulex_init(USARTn,USART_BAUDRATE);
		c_io_herkulex_config(idServoRight);
		c_common_utils_delayms(12);
		c_io_herkulex_config(idServoLeft);
		c_io_herkulex_setPosition(idServoRight,0);
		c_io_herkulex_setPosition(idServoLeft,0);
		c_common_utils_delayms(2000);
	#endif
}

/** \brief Função principal do módulo de RC.
  * @param  None
  * @retval None
  *
  * Interpreta o recebimento de PPM, calcula sinais de controle e os envia
  * via interface.
  * Devido as diferenças do modelo matematica com a construção mecanica o sinal do angulo do servo direito deve
  * ser adaptado.
  *
  */
pv_type_datapr_servos c_io_servos_read()
{
	pv_type_datapr_servos servos;
	#ifdef SERVO_RX24F

	#endif
	#ifdef SERVO_HERKULEX
		if (c_io_herkulex_readData(idServoRight)){
			alphaRight   = c_io_herkulex_getPosition();
			dotAlphaRight= c_io_herkulex_getVelocity();
			status_errorRight = c_io_herkulex_getStatusError();
			status_detailRight = c_io_herkulex_getStatusDetail();
			if (status_errorRight)
				c_io_herkulex_clear(idServoRight);
		}

		if (c_io_herkulex_readData(idServoLeft)){
			alphaLeft = c_io_herkulex_getPosition();
			dotAlphaLeft = c_io_herkulex_getVelocity();
			status_errorLeft = c_io_herkulex_getStatusError();
			status_detailLeft = c_io_herkulex_getStatusDetail();
			if (status_errorLeft)
				c_io_herkulex_clear(idServoLeft);
		}
	#endif
	servos.alphal=-alphaLeft;
	servos.alphar=alphaRight;
	servos.dotAlphal=-dotAlphaLeft;
	servos.dotAlphar=dotAlphaRight;
	return servos;
}
/** \brief Função principal do módulo de RC.
  * @param  None
  * @retval None
  *
  * Interpreta o recebimento de PPM, calcula sinais de controle e os envia
  * via interface.
  * Devido as diferenças do modelo matematica com a construção mecanica o sinal do angulo do servo direito deve
  * ser adaptado.
  *
  */
void c_io_servos_writePosition(float positionRight, float positionLeft)
{
#ifdef SERVO_RX24F
	/* Escrita dos servos */
	if( (positionRight*RAD_TO_DEG<70) && (positionRight*RAD_TO_DEG>-70) )
		status_errorRight=c_io_rx24f_move(idServoRight, 150+positionRight*RAD_TO_DEG);

	if( (positionLeft*RAD_TO_DEG<70) && (positionLeft*RAD_TO_DEG>-70) )
		status_errorLeft=c_io_rx24f_move(idServoLeft, 130+positionLeft*RAD_TO_DEG);
#endif
#ifdef SERVO_HERKULEX
	/* Escrita dos servos */
	if( (positionRight*RAD_TO_DEG<70) && (positionRight*RAD_TO_DEG>-70) )
		c_io_herkulex_setPosition(idServoRight,positionRight);

	if( (positionLeft*RAD_TO_DEG<70) && (positionLeft*RAD_TO_DEG>-70) )
		c_io_herkulex_setPosition(idServoLeft,positionLeft);
#endif
}

/** \brief Escrita dos servos por Torque o maximo torque permitido é 2 N.m.
  * @param  Torque Servomotor Direito
  * @retval Torque Servomotor Esquerdo
  *
  * Receve o torque em N.m e interpreta ele para um registrador de 0 - 1023
  *
  * Devido as diferenças do modelo matematica com a construção mecanica o sinal do torque do servo esquerdo deve
  * ser adaptado.
  *
  */
void c_io_servos_writeTorque(float torqueRight, float torqueLeft)
{
#ifdef SERVO_RX24F

#endif
#ifdef SERVO_HERKULEX
	int16_t sp_torqR;
	int16_t sp_torqL;

	// Calculo do set point
	sp_torqR=(int16_t)(torqueRight*1023*0.425531915);  // O registrador dos servos precisa de 0 ate 1023 -> dinamica direta
	sp_torqL=(int16_t)(torqueLeft*1023*0.425531915);


	if((alphaRight>0.9*(PI/2) && sp_torqR>0) || (alphaRight<-0.9*(PI/2) && sp_torqR<0))
		c_io_herkulex_setTorque(idServoRight,0);
	else
		c_io_herkulex_setTorque(idServoRight,sp_torqR);

	if((alphaLeft>0.9*(PI/2) && sp_torqL<0) || (alphaLeft<-0.9*(PI/2) && sp_torqL>0))
		c_io_herkulex_setTorque(idServoLeft,0);
	else
		c_io_herkulex_setTorque(idServoLeft,-sp_torqL);

#endif
}
pv_type_servoOutput c_io_servos_getInformation(){
	pv_type_servoOutput information;
	information.servo.alphal=alphaLeft;
	information.servo.alphar=alphaRight;
	information.servo.dotAlphal=dotAlphaLeft;
	information.servo.dotAlphar=dotAlphaRight;
	information.idLeft=idServoLeft;
	information.idRight=idServoRight;
	information.status_detaiLeft=status_detailLeft;
	information.status_errorLeft=status_errorLeft;
	information.status_detaiRight=status_detailRight;
	information.status_errorRight=status_errorRight;
	return information;
}
/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */
