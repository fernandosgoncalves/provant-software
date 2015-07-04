/**
  ******************************************************************************
  * @file    modules/rc/pv_module_rc.c
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    30-November-2013
  * @brief   ...
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "pv_module_co.h"

/** @addtogroup ProVANT_app
  * @{
  */

/** @addtogroup app_co
  * \brief Módulo com as principais funcionalidades para calculo de controle e escrita de atuadores.
  *
  * Definição do módulo.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MODULE_PERIOD	 5//ms
#define ESC_ON           1
#define SERVO_ON         1

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
portTickType lastWakeTime;
pv_msg_input iInputData;
pv_msg_controlOutput oControlOutputData; 
//GPIOPin debugPin;
/* Inboxes buffers */
pv_type_actuation    iActuation;
/* Outboxes buffers*/

/* Private function prototypes -----------------------------------------------*/
unsigned char setPointESC_Forca(float forca);
/* Private functions ---------------------------------------------------------*/
/* Exported functions definitions --------------------------------------------*/

/** \brief Inicializacao do módulo de controle + output.
  *
  * Instancia as Queues de comunicação inter-thread, inicializa a pinagem necessária para
  * os perifericos e aloca o que for necessário para as equações de controle.
  * @param  None
  * @retval None
  */
void module_co_init() 
{

  /* Inicializar os escs*/
  c_common_i2c_init(I2C3);
  c_io_blctrl_init_i2c(I2C3);

  /* Inicializar os servos */
  c_io_rx24f_init(1000000);
  c_common_utils_delayms(2);
  c_io_rx24f_setSpeed(1, 200);
  c_io_rx24f_setSpeed(2, 200);
  c_common_utils_delayms(2);
  /* CCW Compliance Margin e CCW Compliance margin */
  c_io_rx24f_write(1, 0x1A,0x03);
  c_io_rx24f_write(1, 0x1B,0x03);
  c_io_rx24f_write(2, 0x1A,0x03);
  c_io_rx24f_write(2, 0x1B,0x03);
  c_common_utils_delayms(2);
  c_io_rx24f_move(1, 130);
  c_io_rx24f_move(2, 150);
  c_common_utils_delayms(100);

  /*Inicializar o tipo de controlador*/
  c_rc_BS_control_init();

  /* Pin for debug */
  //debugPin = c_common_gpio_init(GPIOE, GPIO_Pin_13, GPIO_Mode_OUT);

  pv_interface_co.iInputData          = xQueueCreate(1, sizeof(pv_msg_input));
  pv_interface_co.oControlOutputData  = xQueueCreate(1, sizeof(pv_msg_controlOutput));
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
void module_co_run() 
{
  unsigned int heartBeat=0;
  /* Inicializa os dados da attitude*/
  oControlOutputData.actuation.servoRight = 0;
  oControlOutputData.actuation.servoLeft  = 0;
  oControlOutputData.actuation.escRightSpeed = 0;
  oControlOutputData.actuation.escLeftSpeed  = 0;

  while(1) 
  {
	/* Variavel para debug */
	heartBeat+=1;

	/* toggle pin for debug */
	//c_common_gpio_toggle(debugPin);

    /* Passa os valores davariavel compartilha para a variavel iInputData */
    xQueueReceive(pv_interface_co.iInputData, &iInputData, 0);

    /* Leitura do numero de ciclos atuais */
	lastWakeTime = xTaskGetTickCount();



	/* Escrita dos servos */

	if (iInputData.securityStop){
		c_io_rx24f_move(1, 130+0);
		c_io_rx24f_move(2, 150+0);
	}
	else{
		// inicializacao
		if (iInputData.init){
			c_io_rx24f_move(1, 130+0);
			c_io_rx24f_move(2, 150+0);
		}
		else{
			if( (iActuation.servoRight*RAD_TO_DEG<70) && (iActuation.servoRight*RAD_TO_DEG>-70) )
				c_io_rx24f_move(2, 150+iActuation.servoRight*RAD_TO_DEG);
			if( (iActuation.servoLeft*RAD_TO_DEG<70) && (iActuation.servoLeft*RAD_TO_DEG>-70) )
				c_io_rx24f_move(1, 130+iActuation.servoLeft*RAD_TO_DEG);
		}
	}

	/* Escrita dos esc */
	unsigned char sp_right;
	unsigned char sp_left;
	sp_right = setPointESC_Forca(iActuation.escRightSpeed);
	sp_left = setPointESC_Forca(iActuation.escLeftSpeed );

	if (iInputData.securityStop){
		c_io_blctrl_setSpeed(1, 0 );//sp_right
		c_common_utils_delayus(10);
		c_io_blctrl_setSpeed(0, 0 );//sp_left
	}
	else{
		//inicializacao
		if (iInputData.init){
			c_io_blctrl_setSpeed(1, ESC_MINIMUM_VELOCITY );
			c_common_utils_delayus(10);
			c_io_blctrl_setSpeed(0, ESC_MINIMUM_VELOCITY );
		}
		else{
			c_io_blctrl_setSpeed(1, sp_right );//sp_right
			c_common_utils_delayus(10);
			c_io_blctrl_setSpeed(0, sp_left );//sp_left
		}
	}


    oControlOutputData.actuation.servoRight = c_io_blctrl_readVoltage(1);
    oControlOutputData.actuation.servoLeft = c_io_blctrl_readSpeed(1);
    oControlOutputData.actuation.escRightSpeed    = 13.3;
    oControlOutputData.actuation.escLeftSpeed    = 13.4;
    oControlOutputData.vantBehavior.rpy[0]        = 7.1;
    oControlOutputData.vantBehavior.rpy[1]        = 7.2;
    oControlOutputData.vantBehavior.rpy[2]        = 7.3;
    oControlOutputData.vantBehavior.drpy[0]       = 8.1;
    oControlOutputData.vantBehavior.drpy[1]       = 8.2;
    oControlOutputData.vantBehavior.drpy[2]       = 8.3;
    oControlOutputData.vantBehavior.xyz[0]        = 3.1;
    oControlOutputData.vantBehavior.xyz[1]        = 3.2;
    oControlOutputData.vantBehavior.xyz[2]        = 3.3;
    oControlOutputData.vantBehavior.dxyz[0]       = 4.1;
    oControlOutputData.vantBehavior.dxyz[1]       = 4.2;
    oControlOutputData.vantBehavior.dxyz[2]       = 4.3;
    oControlOutputData.heartBeat                  = heartBeat;
    unsigned int timeNow=xTaskGetTickCount();
    oControlOutputData.cicleTime                  = timeNow - lastWakeTime;

    /* toggle pin for debug */
    //c_common_gpio_toggle(debugPin);

    if(pv_interface_co.oControlOutputData != 0)
      xQueueOverwrite(pv_interface_co.oControlOutputData, &oControlOutputData);

    /* A thread dorme ate o tempo final ser atingido */
    vTaskDelayUntil( &lastWakeTime, MODULE_PERIOD / portTICK_RATE_MS);
	}
}

/**\ brief Calcula o set point do ESC a partir da forca passada por argumento
 * Curva retirada dos ensaios com os motores brushless no INEP
 */
unsigned char setPointESC_Forca(float forca){
	//	Coefficients:
	float p1 = 0.00088809, p2 = -0.039541, p3 = 0.67084, p4 = -5.2113, p5 = 16.33, p6 = 10.854, p7 = 3.0802, set_point=0;

	if (forca <= 0)
		return (unsigned char) ESC_MINIMUM_VELOCITY;
	else{
		set_point = (p1*pow(forca,6) + p2*pow(forca,5) + p3*pow(forca,4) + p4*pow(forca,3)
								+ p5*pow(forca,2) + p6*forca + p7);
	    if (set_point >= 255)
	    	return (unsigned char)255;
	    else
	    	return (unsigned char)set_point;}
}
/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */
