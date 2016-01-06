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
#define MODULE_PERIOD	    6//ms
//#define USART_BAUDRATE     460800  //<-Bluethood
#define USART_BAUDRATE     921600 //<-Beaglebone

//#define NONHIL
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
portTickType lastWakeTime;
unsigned int heartBeat=0;
unsigned int cicleTime=0;
pv_msg_input iInputData;
pv_msg_gps iGpsData;
pv_msg_controlOutput iControlOutputData;
pv_msg_controlOutput oControlBeagleData;
float rpy[3];
float drpy[3];
float position[3];
float velocity[3];
float alpha[2];
float dalpha[2];
float data1[2],data2[2],data3[2];
int aux[2];
float aux2[3];
float servoTorque[2];
float escForce[2];
int channel[7];
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
  /*Data consumed by the thread*/
  pv_interface_do.iInputData          = xQueueCreate(1, sizeof(pv_msg_input));
  pv_interface_do.iControlOutputData  = xQueueCreate(1, sizeof(pv_msg_controlOutput));
  // pv_interface_do.iGpsData           = xQueueCreate(1, sizeof(pv_msg_gps));
  /*Data produced by the thread*/
  pv_interface_do.oControlBeagleData  = xQueueCreate(1, sizeof(pv_msg_controlOutput));

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
	oControlBeagleData.actuation.servoRight = 0;
	oControlBeagleData.actuation.servoLeft  = 0;
	oControlBeagleData.actuation.escRightSpeed = 0;
	oControlBeagleData.actuation.escLeftSpeed  = 0;
	while(1)
	{
		lastWakeTime = xTaskGetTickCount();
		heartBeat++;

		if (uxQueueMessagesWaiting(pv_interface_do.iInputData)!=0){
			xQueueReceive(pv_interface_do.iInputData, &iInputData, 0);
		}
		if (uxQueueMessagesWaiting(pv_interface_do.iControlOutputData)!=0){
			xQueueReceive(pv_interface_do.iControlOutputData, &iControlOutputData, 0);
		}
		//xQueueReceive(pv_interface_do.iGpsData, &iGpsData, 0);


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
		aux[0]=iInputData.securityStop;
		aux[1]=0;

		aux2[0]=0;
		aux2[1]=0;
		aux2[2]=0;

		rpy[0]=iInputData.attitude.roll;
		rpy[1]=iInputData.attitude.pitch;
		rpy[2]=iInputData.attitude.yaw;

		drpy[0]=iInputData.attitude.dotRoll;
		drpy[1]=iInputData.attitude.dotPitch;
		drpy[2]=iInputData.attitude.dotYaw;

		position[0]=0;//iInputData.position.x;
		position[1]=0;//iInputData.position.y;
		position[2]=0;//iInputData.position.z;

		velocity[0]=iInputData.position.dotX;
		velocity[1]=iInputData.position.dotY;
		velocity[2]=iInputData.position.dotZ;

		alpha[0]=iInputData.servosOutput.servo.alphal;
		alpha[1]=iInputData.servosOutput.servo.alphar;

		dalpha[0]=iInputData.servosOutput.servo.dotAlphal;
		dalpha[1]=iInputData.servosOutput.servo.dotAlphar;

		channel[0]=iInputData.receiverOutput.joystick[0];
		channel[1]=iInputData.receiverOutput.joystick[1];
		channel[2]=iInputData.receiverOutput.joystick[2];
		channel[3]=iInputData.receiverOutput.joystick[3];
		channel[4]=iInputData.receiverOutput.aButton;
		channel[5]=iInputData.receiverOutput.bButton;
		channel[6]=0;

		data1[0]= iControlOutputData.actuation.servoLeft;
		data1[1]= iControlOutputData.actuation.servoRight;
		data3[0]= iControlOutputData.actuation.escLeftNewtons;
		data3[1]= iControlOutputData.actuation.escLeftSpeed;
		data2[0]= iControlOutputData.actuation.escRightNewtons;
		data2[1]= iControlOutputData.actuation.escRightSpeed;

		c_common_datapr_multwii2_sendControldatain(rpy,drpy,position,velocity);
		c_common_datapr_multwii2_sendEscdata(aux,alpha,dalpha);
		c_common_datapr_multwii2_sendControldataout(data1,data3,data2);
		c_common_datapr_multwii_debug((float)cicleTime,iControlOutputData.cicleTime,iInputData.cicleTime,(float)(2*heartBeat-iInputData.heartBeat-iControlOutputData.heartBeat));
		//c_common_datapr_multwii2_rcNormalize(channel);
		c_common_datapr_multwii_sendstack(USART2);

		c_common_datapr_multiwii_receivestack(USART2);
		pv_type_actuation  actuation=c_common_datapr_multwii_getattitude();

		oControlBeagleData.actuation=actuation;
		if(pv_interface_do.oControlBeagleData != 0)
			xQueueOverwrite(pv_interface_do.oControlBeagleData, &oControlBeagleData);

		#endif

		unsigned int timeNow=xTaskGetTickCount();
		cicleTime = timeNow - lastWakeTime;

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
