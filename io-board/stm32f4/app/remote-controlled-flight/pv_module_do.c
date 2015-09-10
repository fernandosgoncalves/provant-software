/**
  ******************************************************************************
  * @file    app/remote-controlled-flight/pv_module_do.c
  * @author  Patrick Jose Pereira
  * @version V1.0.0
  * @date    27-August-2014
  * @brief   Implementação do módulo de transmissao de dados para fora do ARM.
  ******************************************************************************/

      /* Includes ------------------------------------------------------------------*/
#include "pv_module_do.h"

/** @addtogroup ProVANT_app
  * @{
  */

/** @addtogroup app_do
  * \brief Módulo responsavel por transmitir dados.
  *
  * Definição do módulo de transmissão de dados.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MODULE_PERIOD	    10//ms
#define USART_BAUDRATE     460800 //921600

#define NONHIL
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
portTickType lastWakeTime;
unsigned int heartBeat=0;
pv_msg_input iInputData;
pv_msg_gps iGpsData;
pv_msg_controlOutput iControlOutputData;
float rpy[3];
float drpy[3];
float position[3];
float velocity[3];
float alpha[2];
float dalpha[2];
int aux[2];
float aux2[3];
float servoTorque[2];
float escForce[2];
GPIOPin LED3;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions definitions --------------------------------------------*/

/** \brief Inicializacao do módulo de data out.
  *
  * Instancia as Queues de comunicação inter-thread.
  * @param  None
  * @retval None
  */
void module_do_init() 
{
  /* Inicia a usart2 */
  c_common_usart2_init(USART_BAUDRATE);

  /* Reserva o local de memoria compartilhado */
  pv_interface_do.iInputData          = xQueueCreate(1, sizeof(pv_msg_input));
 // pv_interface_do.iGpsData           = xQueueCreate(1, sizeof(pv_msg_gps));
 // pv_interface_do.iControlOutputData  = xQueueCreate(1, sizeof(pv_msg_controlOutput));

  /* Pin for debug */
  LED3 = c_common_gpio_init(GPIOD, GPIO_Pin_13, GPIO_Mode_OUT); //LED3

}

/** \brief Função principal do módulo de data out.
  * @param  None
  * @retval None
  *
  */
void module_do_run()
{
	while(1)
	{
		lastWakeTime = xTaskGetTickCount();
		heartBeat++;

		xQueueReceive(pv_interface_do.iInputData, &iInputData, 0);
		//xQueueReceive(pv_interface_do.iGpsData, &iGpsData, 0);
		//xQueueReceive(pv_interface_do.iControlOutputData, &iControlOutputData, 0);

		# ifdef NONHIL
		aux2[0]=iInputData.attitude.roll;
		aux2[1]=iInputData.attitude.pitch;
		aux2[2]=iInputData.attitude.yaw;
		//c_common_datapr_multwii_raw_imu(iInputData.imuOutput.accRaw,iInputData.imuOutput.gyrRaw,iInputData.imuOutput.magRaw);
		c_common_datapr_multwii_attitude(iInputData.attitude.roll*RAD_TO_DEG*10,iInputData.attitude.pitch*RAD_TO_DEG*10,iInputData.attitude.yaw*RAD_TO_DEG*10);
		c_common_datapr_multwii2_sendControldatain(iInputData.imuOutput.accRaw,iInputData.imuOutput.gyrRaw,iInputData.imuOutput.magRaw,aux2);
		//c_common_datapr_multwii_attitude(iGpsData.heartBeat,iGpsData.gpsOutput.lat,iGpsData.gpsOutput.lon);
		//c_common_datapr_multwii2_rcNormalize(channel);
		//c_common_datapr_multwii_altitude(iInputData.position.z,iInputData.position_refrence.z*100);
		//c_common_datapr_multwii_debug(iInputData.servoLeft.servo.alphal*180/PI,iInputData.servoLeft.servo.dotAlphal*180/PI,iInputData.servoRight.servo.alphar*180/PI,iInputData.servoRight.servo.dotAlphar*180/PI);
		//c_common_datapr_multwii_debug(iInputData.servoLeft.torque,iInputData.servoLeft.torque,0,0);
		c_common_datapr_multwii_sendstack(USART2);

//        data1[0]=iControlOutputData.actuation.servoLeft*RAD_TO_DEG;
//        data1[1]=iControlOutputData.actuation.servoRight*RAD_TO_DEG;
//        //data1[0]=iGpsData.gpsOutput.lat;
//        //data1[1]=iGpsData.gpsOutput.lon;
//        data2[0]=iControlOutputData.actuation.escLeftSpeed;
//        data2[1]=iControlOutputData.actuation.escRightSpeed;
//        data3[0]=iInputData.attitude_reference.roll*RAD_TO_DEG;
//        data3[1]=iInputData.attitude_reference.pitch*RAD_TO_DEG;


		//c_common_datapr_multwii2_sendControldatain(iControlOutputData.actuation.servoLeftvantBehavior.rpy, iControlOutputData.vantBehavior.drpy, iControlOutputData.vantBehavior.xyz, iControlOutputData.vantBehavior.dxyz);
		//c_common_datapr_multwii2_sendControldataout(data1,data3,data2);
		//c_common_datapr_multwii_sendstack(USART2);
		#else
		aux[0]=0;
		aux[1]=0;

		aux2[0]=0;
		aux2[1]=0;
		aux2[2]=0;

		rpy[0]=1.1;
		rpy[1]=2.2;
		rpy[2]=3.3;

		drpy[0]=4.4;
		drpy[1]=5.5;
		drpy[2]=6.6;

		position[0]=7.7;
		position[1]=8.8;
		position[2]=9.9;

		velocity[0]=10.10;
		velocity[1]=11.11;
		velocity[2]=12.12;

		alpha[0]=iInputData.servosOutput.servo.alphal*RAD_TO_DEG;
		alpha[1]=iInputData.servosOutput.servo.alphar*RAD_TO_DEG;

		dalpha[0]=iInputData.servosOutput.servo.dotAlphal*RAD_TO_DEG;
		dalpha[1]=iInputData.servosOutput.servo.dotAlphar*RAD_TO_DEG;

		c_common_datapr_multwii2_sendControldatain(rpy,drpy,position,velocity);
		c_common_datapr_multwii2_sendEscdata(aux,alpha,dalpha);
		c_common_datapr_multwii_sendstack(USART2);


		//c_common_datapr_multiwii_receivestack(USART2);
		//pv_type_actuation  actuation=c_common_datapr_multwii_getattitude();

//		servoTorque[0]=actuation.servoLeft;
//		servoTorque[1]=actuation.servoRight;
//		escForce[0]=actuation.escLeftNewtons;
//		escForce[1]=actuation.escRightNewtons;
//		aux2[0]=actuation.escLeftSpeed;
//		aux2[1]=actuation.escRightSpeed;
//
//		c_common_datapr_multwii2_sendControldataout(servoTorque,escForce,aux2);
//		c_common_datapr_multwii_sendstack(USART2);
		#endif
		/* toggle pin for debug */
		c_common_gpio_toggle(LED3);

		vTaskDelayUntil( &lastWakeTime, (MODULE_PERIOD / portTICK_RATE_MS));
	}
}
/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */
