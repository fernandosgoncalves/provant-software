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
int32_t size;// = sizeof(pv_msg_esc);
GPIOPin LED_builtin_io;
/* Inboxes buffers */
pv_type_actuation    iActuation;
/* Outboxes buffers*/

/* Private function prototypes -----------------------------------------------*/
unsigned char setPointESC_Forca(float forca);
float position_controller(float r, float y);
float velocity_controller(float r, float y);
float velocity_feedforward(float r);
int16_t saturate(float x, const float max);
int16_t get_stepped_pwm(int heartBeat, int16_t pwm);
int check_outlier(int new,int sec);

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
//  c_common_i2c_init(I2C3);
//  c_io_blctrl_init_i2c(I2C3);


  /*Inicializar o tipo de controlador*/


  /* Pin for debug */
  LED_builtin_io = c_common_gpio_init(GPIOD, GPIO_Pin_15, GPIO_Mode_OUT);
  c_common_gpio_toggle( LED_builtin_io);
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
  /* Passa os valores davariavel compartilha para a variavel iInputData */
  //xQueueReceive(pv_interface_co.iInputData, &iInputData, 0);

  /* Inicializa os dados da atuação*/
  oControlOutputData.actuation.servoRight = 0;
  oControlOutputData.actuation.servoLeft  = 0;
  oControlOutputData.actuation.escRightSpeed = 0;
  oControlOutputData.actuation.escLeftSpeed  = 0;

  /* Inicializa os dados da atuação*/
  uint8_t status = 0;
  float torque=0;
  int st =0, el=0;
  //c_io_herkulex_set_torque(servo_id1, pwm);
  //c_common_utils_delayms(100);
  int32_t data_counter=0;
  int32_t queue_data_counter = 0;
  int32_t lost_data_counter = 0;
  uint8_t data_received = 0;


//  c_io_herkulex_read_data(servo_id1);
//  float initial_position = c_io_herkulex_get_position(servo_id1);
//
//  float ref_pos = initial_position + 5*PI/180;
//  portBASE_TYPE xStatus;
//  c_common_utils_delayms(1);

  while(1){
	  lastWakeTime = xTaskGetTickCount();
	  /* Variavel para debug */
	  heartBeat+=1;

	  /* Passa os valores davariavel compartilha para a variavel iInputData */
	  xQueueReceive(pv_interface_co.iInputData, &iInputData, 0);

	  /* Leitura do numero de ciclos atuais */
	  lastWakeTime = xTaskGetTickCount();

	/*-------------------Calculo do controle------------------------------*/


	/*-------------------Escrita dos servos-------------------------------*/
	  oControlOutputData.actuation.servoRight=(iInputData.receiverOutput.joystick[1]/100.0f)*1000.0f;
	  oControlOutputData.actuation.servoLeft=oControlOutputData.actuation.servoRight;
		/**
	 * Leitura de dados
	 */






//
//
//
//
//	#endif
//			//pwm=200;
//
//	#if !SERVO_IN_TEST
//
//			/**
//			 * Envio dos dados para o modulo de comunicação
//			 * Verificação da integridade dos pacotes recebidos
//			 */
//			status = c_io_herkulex_get_status();//indica erros de comunicação
//
//			if (status)
//			{
//				oServoMsg.pwm=pwm;
//				oServoMsg.heartBeat=heartBeat;
//				xStatus = xQueueSend(pv_interface_servo.oServoOutput,
//						&oServoMsg,1/portTICK_RATE_MS);
//				if (!xStatus) xQueueSend(pv_interface_servo.oServoOutput,
//						&oServoMsg,1/portTICK_RATE_MS);
//
//				queue_data_counter++;
//			} else
//			{
//				c_io_herkulex_stat(servo_id);
//				status_error=c_io_herkulex_get_status_error();
//				status_detail=c_io_herkulex_get_status_detail();
//				if (status_error!=0 || status_detail!= 0)
//				{
//					c_io_herkulex_clear(servo_id);
//				}
//				lost_data_counter++;
//			}
//
//
//			/**
//			 * Para utilização em testes finitos dos servos.
//			 * O tamanho da fila(xQueue) indica o numero de pontos acumulados
//			 */
//			//c_common_utils_delayms(1);=1000
//			uint16_t queue_size = uxQueueMessagesWaiting(
//					pv_interface_servo.oServoOutput);
//			if (queue_size<QUEUE_SIZE)
//			{
//	#endif
//				lastWakeTime=wakeTime;
//				vTaskDelayUntil( &lastWakeTime, (MODULE_PERIOD / portTICK_RATE_MS));
//	#if !SERVO_IN_TEST
//			} else
//			{
//				break;
//			}
//	#endif
//		}
//		c_io_herkulex_set_torque(servo_id, 0);
//		c_io_herkulex_set_torque_control(servo_id,TORQUE_BREAK);//set torque free
//	if (iInputData.securityStop){
//		c_io_rx24f_move(1, 130+0);
//		c_io_rx24f_move(2, 150+0);
//	}
//	else{
//		// inicializacao
//		if (iInputData.init){
//			c_io_rx24f_move(1, 130+0);
//			c_io_rx24f_move(2, 150+0);
//		}
//		else{
//			if( (iActuation.servoRight*RAD_TO_DEG<70) && (iActuation.servoRight*RAD_TO_DEG>-70) )
//				c_io_rx24f_move(2, 150+iActuation.servoRight*RAD_TO_DEG);
//			if( (iActuation.servoLeft*RAD_TO_DEG<70) && (iActuation.servoLeft*RAD_TO_DEG>-70) )
//				c_io_rx24f_move(1, 130+iActuation.servoLeft*RAD_TO_DEG);
//		}
//	}

//	/* Escrita dos esc */
//	unsigned char sp_right;
//	unsigned char sp_left;
//	sp_right = setPointESC_Forca(iActuation.escRightSpeed);
//	sp_left = setPointESC_Forca(iActuation.escLeftSpeed );
//
//	if (iInputData.securityStop){
//		c_io_blctrl_setSpeed(1, 0 );//sp_right
//		c_common_utils_delayus(10);
//		c_io_blctrl_setSpeed(0, 0 );//sp_left
//	}
//	else{
//		//inicializacao
//		if (iInputData.init){
//			c_io_blctrl_setSpeed(1, ESC_MINIMUM_VELOCITY );
//			c_common_utils_delayus(10);
//			c_io_blctrl_setSpeed(0, ESC_MINIMUM_VELOCITY );
//		}
//		else{
//			c_io_blctrl_setSpeed(1, sp_right );//sp_right
//			c_common_utils_delayus(10);
//			c_io_blctrl_setSpeed(0, sp_left );//sp_left
//		}
//	}


    unsigned int timeNow=xTaskGetTickCount();
    oControlOutputData.cicleTime                  = timeNow - lastWakeTime;

    /* toggle pin for debug */
//    c_common_gpio_toggle( LED_builtin_io);
//
    if(pv_interface_co.oControlOutputData != 0)
    	xQueueOverwrite(pv_interface_co.oControlOutputData, &oControlOutputData);

    /* A thread dorme ate o tempo final ser atingido */
    vTaskDelayUntil( &lastWakeTime, MODULE_PERIOD / portTICK_RATE_MS);
	}
}

/* Private functions ---------------------------------------------------------*/

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
float position_controller(float r, float y)
{
	static float e_old = 0, u = 0;
	float e = r-y;
	//Tset=200ms PASSOU no teste RP 5/6
	//int K1 = 144.1257, K2 = 92.8029;
	//int K1=619.4, K2=318.4955; //original
	//Passou no teste RP 5x
	//int K1 = 22.0272, K2=20.6867;// slow motherfucker PI
	int K1 = 11.1838, K2 = -10.8214;
	u=u+K1*e-K2*e_old; //intermediario
	e_old=e;
	//saturacao

	//assert(out>=(-1023) && out<=1023);

	return u;
}

float velocity_controller(float r, float y)
{
	static float e_old = 0, u = 0;
	float e = r-y;
	//Tset=200ms PASSOU no teste RP 5/6
	int K1 = 144.1257, K2 = 92.8029;
	//int K1=619.4, K2=318.4955; //original
	//Passou no teste RP 5x20.6867
	//int K1 = 15.966, K2=7.9196;// slow motherfucker PI

	u=u+K1*e-K2*e_old;
	e_old=e;
	//saturacao

	//assert(out>=(-1023) && out<=1023);

	return u;
}

float velocity_feedforward(float r)
{
	static float y = 0;
	//y=0.5142*(y+r); //original
	y=0.6439*y+0.3561*r; //slow

	return y;
}

int16_t saturate(float x, const float max)
{
	if (x>max)
		x=max;
	else if (x<(-max))
		x=-max;
	if (x>0) x+=0.5;
	if (x<0) x-=0.5;


	return (int16_t)x;
}

int16_t get_stepped_pwm(int heartBeat, int16_t pwm)
{
  static int8_t counter = 0, inc=1;

  //parametros da escada
  const int STEP_LENGTH = 50; //tamanho do degraus em ptos de amostragem
  const int STEP_SIZE = 200; //amplitude dos degraus
  const int NUM_STEPS = 5; //numero de degraus

  if ((heartBeat%STEP_LENGTH)==0)
  {
    if (counter == NUM_STEPS) inc=0;
    else if (counter == -NUM_STEPS) inc=1;
    if (inc) counter++;
    else counter--;
    pwm=counter*200;
    //if (pwm) pwm=0;
    //else pwm=1000;
  }

  return pwm;
}
//new value, and secure value
int check_outlier(int new, int sec)
{
	int limite = 0.6;
	return ((sec>=0 && (new>=sec*limite)) ||
			(sec<0 && (new<=sec*limite)));
}
/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */
