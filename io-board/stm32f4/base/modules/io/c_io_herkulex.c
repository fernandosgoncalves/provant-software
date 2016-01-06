/**
  ******************************************************************************
  * @file    modules/io/c_io_herkulex.h
  * @author  Iuro Baptista Pereira Nascimento
  * @version V1.0.0
  * @date    01/12/2014
  * @brief   Implementação da Servo Herkulex DRS-0201
  *****************************************************************************/

#include "c_io_herkulex.h"

#include <math.h>
#include <stdint.h>
#include <string.h>
#include "c_common_uart.h"

/** @addtogroup Module_IO
  * @{
  */

/** @addtogroup Module_IO_Component_Herkulex
  *	\brief Componente para o uso dos servos Herkulex DRS- 0201
  *
  * @{
  */

/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/
#define SIZE_BUFFER 128

/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
USART_TypeDef *usartx;
/* variaveis para decodificação de pacotes */
uint8_t statusError;
uint8_t statusDetail;
float position;
float velocity;
/** informa se pacotes foram recebidos e se estão inteiros
 * status =
 * 1- ok, recebido
 * 0- erro: ou o pacote nao foi recebido, ou o pacote foi corrompido.
 */
uint8_t status;/* informa se pacotes foram recebidos, */
uint8_t BUFFER[SIZE_BUFFER];		// Buffer de dados a serem enviados
uint8_t DATA[20];		// Buffer dados crus recebidos sem cabeçalho ou soma de verificação



/* Private function prototypes -----------------------------------------------*/
void c_io_herkulex_clearBuffer(void);
float c_io_herkulex_raw2pos(uint8_t data[]);
float c_io_herkulex_raw2vel(uint8_t data[]);
uint8_t c_io_herkulex_unpack(void);
uint8_t c_io_herkulex_serializeRw(uint8_t size, uint8_t pid, uint8_t cmd,
		uint8_t dataAddr, uint8_t dataLength);

void c_io_herkulex_serializeSjog(pv_sjog_herkulex sjog[], uint8_t numSjogs,
		uint8_t ptime);

void c_io_herkulex_serializeIjog(pv_ijog_herkulex ijog[], uint8_t numIjogs);
void c_io_herkulex_send(void);
uint8_t c_io_herkulex_receive(void);
uint8_t c_io_herkulex_checksum1(uint8_t *buffer, uint8_t size);
uint8_t c_io_herkulex_checksum2(uint8_t cksum1);
int16_t c_io_herkulex_deg2raw(float position);


/* Private functions ---------------------------------------------------------*/
/** \brief Torna zero os elementos do buffer de saída. */
void c_io_herkulex_clearBuffer(void)
{
	int i;

	for (i = 0; i < SIZE_BUFFER; i++)
		BUFFER[i] = 0;
}

/** \brief Converte os dados enviados pelo servo para radianos.
 *
 *	@param data dados enviados pelo servo.
 *	@return retorna a posição em radianos.
 */
float c_io_herkulex_raw2pos(uint8_t data[])
{
	int16_t rawData = (int16_t)(((data[1] & 0x03) << 8) | data[0]);
	return (((float)rawData) * 0.325 - 166.65) * M_PI / 180.0;
}

/** \brief Converte os dados enviados pelo servo para rad/s.
 *
 *	@param data dados enviados pelo servo.
 *	@return retorna a velocidade em rad/s.
 */
float c_io_herkulex_raw2vel(uint8_t data[])
{
	int16_t rawData = (int16_t)(((data[1] & 0xFF) << 8) | data[0]);
	return ((float)rawData) * 29.09 * M_PI / 180.0;
}


/** \brief Retira os dados do buffer.
 *
 * @return returna 1 se o pacote não foi corrompido, 0 se foi.
 */
uint8_t c_io_herkulex_unpack(void)
{
	uint8_t new_cksum1, new_cksum2, dataAddr, dataLength;

	uint8_t size = (unsigned char)BUFFER[2];
	uint8_t pid = BUFFER[3];
	uint8_t cmd = BUFFER[4];
	uint8_t csum1 = BUFFER[5];
	new_cksum1 = c_io_herkulex_checksum1(BUFFER, size);
	uint8_t csum2 = BUFFER[6];
	new_cksum2 = c_io_herkulex_checksum2(new_cksum1);
	if ((new_cksum1 != csum1) || (new_cksum2 != csum2))
		return 0;

	if (size > 7) {
		if (cmd == ACK_STAT) {
			statusError = BUFFER[7];
			statusDetail = BUFFER[8];
		} else {
			dataAddr = BUFFER[7];
			dataLength = BUFFER[8];
		}
	}

	if (size > 9) {
		memcpy(DATA, BUFFER + 9, dataLength);
		statusError = BUFFER[size - 2];
		statusDetail = BUFFER[size - 1];
	}

	return 1;
}


/** \brief Coloca no buffer pacotes de leitura e escrita de registradores.
 *
 * @param size Tamanho do pacote.
 * @param pid ID do servo, ou o endereço de broadcast 0xFE.
 * @param cmd Comando a ser enviado, XXX_WRITE ou XXX_READ. XXX = EEP | RAM.
 * @param dataAddr Endereço do registrador.
 * @param dataLength Número de bytes lidos ou escritos.
 * @return retorna 0 se o buffer for inconsistente. Caso contrário, 1.
 */
uint8_t c_io_herkulex_serializeRw(uint8_t size, uint8_t pid, uint8_t cmd,
		uint8_t dataAddr, uint8_t dataLength)
{
	if (size <= 0)
		return 0;

	BUFFER[0] = HEADER;
	BUFFER[1] = HEADER;
	BUFFER[2] = size;
	BUFFER[3] = pid;
	BUFFER[4] = cmd;

	if (size > 7) {
		BUFFER[7] = dataAddr;
		BUFFER[8] = dataLength;
	}

	uint8_t i;

	if (size > 9) {
		for (i = 0; i < dataLength; i++)
			BUFFER[9 + i] = DATA[i];
	}


	if ((size-dataLength) > 9) {
		BUFFER[size - 2] = statusError;
		BUFFER[size - 1] = statusDetail;
	}

	BUFFER[5] = c_io_herkulex_checksum1(BUFFER, size);
	BUFFER[6] = c_io_herkulex_checksum2(BUFFER[5]);

	return 1;
}

/** \brief Coloca no buffer pacotes sjog de comandos de torque/posição.
 *
 * @param sjog[] É um array de pv_sjog_herkulex, podendo conter um ou mais elementos.
 * @param numSjogs Número de elementos em sjog[].
 * @param ptime "Play time": tempo estipulado para o servo chegar a posição de destino.
 */
void c_io_herkulex_serializeSjog(pv_sjog_herkulex sjog[], uint8_t numSjogs,
		uint8_t ptime)
{
	int n = sizeof(pv_sjog_herkulex);

	BUFFER[0] = HEADER;
	BUFFER[1] = HEADER;
	/* numero_sjogs*bytes_por_sjog+bytes_de_cabecalho */
	BUFFER[2] = numSjogs * n + 8;
	if (numSjogs>1)
		BUFFER[3] = 0xFE;
	else
		BUFFER[3] = sjog[0].ucID;

	BUFFER[4] = S_JOG;
	BUFFER[7] = ptime;

	memcpy((BUFFER+8), (void *) sjog, n * numSjogs);

	BUFFER[5] = c_io_herkulex_checksum1(BUFFER, BUFFER[2]);
	BUFFER[6] = c_io_herkulex_checksum2(BUFFER[5]);
}


/** \brief Coloca no buffer pacotes sjog de comandos de torque/posição.
 *
 * @param ijog[] Array de pv_ijog_herkulex, podendo conter um ou mais elementos.
 * @param numIjogs Número de elementos em sjog[].
 */
void c_io_herkulex_serializeIjog(pv_ijog_herkulex ijog[], uint8_t numIjogs)
{
	int n = sizeof(pv_ijog_herkulex);

	BUFFER[0] = HEADER;
	BUFFER[1] = HEADER;
	/* numero_sjogs * bytes_por_sjog + bytes_de_cabecalho */
	BUFFER[2] = numIjogs * n + 7;
	if (numIjogs>1)
		BUFFER[3] = 0xFE;
	else
		BUFFER[3] = ijog[0].ucID;

	BUFFER[4] = I_JOG;

	memcpy((BUFFER + 7), (void *) ijog, n * numIjogs);

	/* csum1=checksum1(BUFFER,BUFFER[2]); /* por que usar csum??????? */
	/* csum2=checksum2(csum1); */
	BUFFER[5] = c_io_herkulex_checksum1(BUFFER, BUFFER[2]);
	BUFFER[6] = c_io_herkulex_checksum2(BUFFER[5]);
}

/** \brief Envia os dados do buffer. */
void c_io_herkulex_send(void)
{
	int i;
	for (i = 0; i < BUFFER[2] ; ++i)
		c_common_usart_putchar(usartx, BUFFER[i]);
}

/** \brief Recebe um pacote de dados do servo.
 *
 * @return Booleano que indica se o pacote foi recebido intacto.
 */
uint8_t c_io_herkulex_receive(void)
{
	int i = 0;
	uint8_t lastByte = 0, inByte = 0, ok = 0, size = 30;
	unsigned long now = 0, timeOut;

	now = c_common_utils_millis();
	timeOut = 3 + now;
	taskENTER_CRITICAL();
	while ((long)(now - timeOut) <= 0 && i < size) {
		while (!c_common_usart_available(usartx) && (long)(now - timeOut) <= 0)
			now = c_common_utils_millis();
		lastByte = inByte;
		inByte = c_common_usart_read(usartx);
		if (!ok && inByte == 0xFF && lastByte == 0xFF) {
			BUFFER[0] = 0xFF;
			i = 1;
			ok = 1;
		}
		if (ok) {
			BUFFER[i] = inByte;
			if (i == 2)
				size = BUFFER[2];
			i++;
		}
	}
	if (now >= timeOut){
		taskEXIT_CRITICAL();
		return 0;
	}
	taskEXIT_CRITICAL();
	return 1;
}

/** \brief Faz a primeira soma de verificação dos pacotes.
 *
 * @param buffer Buffer utilizado.
 * @param size Tamanho do buffer em bytes.
 * @return A soma de verificação.
 */
uint8_t c_io_herkulex_checksum1(uint8_t *buffer, uint8_t size)
{
	uint8_t i;
	char chksum = 0;

	for (i = 2; i < 5; i++)
		chksum = chksum ^ buffer[i];

	for (i = 7; i < size; i++)
		chksum = chksum ^ buffer[i];

	return chksum & 0xFE;
}

/** \brief Segunda soma de verificação.
 *
 * @param checksum1 A primeira soma de verificação.
 * @return A segunda soma de verificação.
 */
uint8_t c_io_herkulex_checksum2(uint8_t checksum1)
{
	return (~checksum1) & 0xFE;
}

/** \brief Converte uma posição de graus para inteiro de 16 bits.
 *
 *	O servo DRS0201 entende somente o dado conertido para inteiro de 16 bits.
 *
 * @param position_deg Posição em graus.
 * @return Posição em inteiro de 16 bits.
 */
int16_t c_io_herkulex_deg2raw(float position)
{
	return ((int16_t)(position / 0.325 + 0.5)) + 512; /* 0.5 -> round */
}





/* Exported functions definitions --------------------------------------------*/
/* Direct servo commands */
/** \brief Lê o valor dado um endereço de memoria.
  * Retorna o valor em caso de sucesso.
  *
  * @param mem tipo de memória, ROM ou RAM
  * @param  servoId ID do servo.
  * @param regAddr Endereço do registrador do servo
  * @param dataLength Tamanho dos dados a serem escritos
  * @return ponteiro para os dados lidos.
  */
uint8_t c_io_herkulex_read(uint8_t mem, uint8_t servoId, uint8_t regAddr,
		uint8_t dataLength)
{
	uint8_t size = 9;
	uint8_t cmd;

	if (mem == EEP)
		cmd = EEP_READ;
	else
		cmd = RAM_READ;

	c_io_herkulex_serializeRw(size, servoId, cmd, regAddr, dataLength);
	c_io_herkulex_send();
	c_io_herkulex_clearBuffer();
	status = c_io_herkulex_receive();
	if (status) /* se for corrompido, faz status = 0 */
		status = c_io_herkulex_unpack();

	return status;
}

/** \brief Escreve o valor dado um endereço de memoria, entre 0 e 0x18.
  * Retorna o valor em caso de sucesso.
  *
  * @param mem tipo de memória, ROM ou RAM
  * @param servoId ID do servo.
  * @param regAddr Endereço do registrador do servo
  * @param dataLength Tamanho dos dados a serem escritos
  * @param data Ponteiro para os dados a serem escritos
  * @return Boleano - 1 se dados foram enviados, 0 se não foram.
  */
uint8_t c_io_herkulex_write(uint8_t mem, uint8_t servoId, uint8_t regAddr,
		uint8_t dataLength, uint8_t *data)
{
	uint8_t pid = servoId;
	uint8_t size = 0x09 + dataLength;
	uint8_t cmd;

	if (mem == EEP)
		cmd = EEP_WRITE;
	else
		cmd = RAM_WRITE;

	if (data != NULL) {
		memcpy(DATA, data, dataLength);
		c_io_herkulex_serializeRw(size,servoId,cmd,regAddr,dataLength);
		c_io_herkulex_send();
		return 1;
	} else {
		return 0;
	}
}

/** \brief Envia um comando sjog.
 *
 * \details Um comando sjog pode ser enviado a vários servos ao mesmo tempo,
 * usando 'play time' iguais.
 *
 * @param sjog Array de um ou mais pv_sjog_herkulex.
 * @param numSjog Número de estruturas ijog no array sjog[].
 * @param ptime Play time, é o tempo em que o servo deve executar o comando sjog.
 */
void c_io_herkulex_sendSjog(pv_sjog_herkulex sjog[], uint8_t numSjog,
	uint8_t ptime)
{
	c_io_herkulex_serializeSjog(sjog, numSjog, ptime);
	c_io_herkulex_send();
}

/** \brief Envia um comando ijog.
 *
 * \details Um comando ijog pode ser enviado a vários servos ao mesmo tempo,
 * com 'play time' diferentes.
 *
 * @param ijog[] Array de um ou mais pv_ijog_herkulex.
 * @param numIjog Número de estruturas ijog no array ijog[].
 *
 * TODO Testar envio de pacotes ijog.
 */
void c_io_herkulex_sendIjog(pv_ijog_herkulex ijog[], uint8_t numIjog)
{
	c_io_herkulex_serializeIjog(ijog, numIjog);
	c_io_herkulex_send();
}

/** \brief Envia um comando de pedido de status ao servo.
 *
 * @param servoId ID do servo.
 * @return Booleano que confirma ou não o recebimento do status.
 */
uint8_t c_io_herkulex_readStatus(uint8_t servoId)
{
	c_io_herkulex_serializeRw(7,servoId,STAT,0,0);
	c_io_herkulex_send();

	return c_io_herkulex_receive();
}

/** \brief Reinicia o servo.
 *
 * @param servoId ID do servo.
 */
void c_io_herkulex_reboot(uint8_t servoId)
{
	c_io_herkulex_serializeRw(7,servoId,REBOOT,0,0);
	c_io_herkulex_send();
}

/* Indirect commands */
/** \brief Inicializa o usart para comunicacao serial entre servo e discovery
 *
 * @param usartn USART a ser usada.
 * @param baudrate Baud rate da comunicação serial.
 */
void c_io_herkulex_init(USART_TypeDef *usartn, int baudrate)
{
	usartx = usartn;
	/* Inicia a usart2 */
	if (usartx == USART1)
		c_common_usart1_init(baudrate);
	else if (usartx == USART2)
		c_common_usart2_init(baudrate);
	else if (usartx == USART3)
		c_common_usart3_init(baudrate);
	else if (usartx == USART6)
		c_common_usart6_init(baudrate);
}

/** Configuração inicial do servo.
 *
 * @param servoId ID do servo.
 */
void c_io_herkulex_config(uint8_t servoId)
{
	c_io_herkulex_clear(servoId);
	c_io_herkulex_reboot(servoId);
	c_common_utils_delayms(500);

	/* only reply to read commands */
	c_io_herkulex_configAckPolicy(servoId, 1);

	/* Acceleration Ratio = 0 */
	DATA[0] = 0;
	c_io_herkulex_write(RAM, servoId, REG_ACC_RATIO, 1, DATA);

	/* set no acceleration time */
	DATA[0] = 0;
	c_io_herkulex_write(RAM, servoId, REG_MAX_ACC_TIME, 1, DATA);

	/* zera o offset de pwm */
	DATA[0] = 0;
	c_io_herkulex_write(RAM, servoId, REG_PWM_OFFSET, 1, DATA);

	/* min pwm = 0 */
	DATA[0] = 0;
	c_io_herkulex_write(RAM, servoId, REG_MIN_PWM, 1, DATA);

	/* max pwm >1023 -> no max pwm */
	DATA[1] = 0x03;//little endian 0x03FF sent
	DATA[0] = 0xFF;
	c_io_herkulex_write(RAM, servoId, REG_MAX_PWM, 2, DATA);

	/** set overload pwm register, if overload_pwm>1023, overload is never
	 * activated this is good for data acquisition, but may not be the case for
	 * the tilt-rotor actualy flying.
	 */
	DATA[0] = 0xFF;
	DATA[1] = 0x03;//little endian, 2048 sent
	c_io_herkulex_write(RAM, servoId, REG_OVERLOAD_PWM_THRESHOLD, 1, DATA);

	/** configura Kp, ki,kd and 1st and 2nd feedforward gains
	uint8_t i, n=10;// n is the number of bytes to written
	for(i=0;i<n;i++) {
		DATA[i]=0;
	c_io_herkulex_write(RAM,servo1_id,REG_KP,n,DATA);
	*/
	//c_io_herkulex_setBaudRate(servoId,666666);
	c_io_herkulex_setTorqueControl(servoId, TORQUE_ON);
	c_io_herkulex_clear(servoId);
}

/** \brief Configura a politica de respostas do servo.
 *
 * @param servoId ID do servo.
 * @param policy
 * 				-0: Nenhuma resposta;
 * 				-1: resposta somente a comandos de leitura.
 * 				-2: resposta a todos os comandos.
 * 	O comando stat sempre recebe resposta.
 */
void c_io_herkulex_configAckPolicy(uint8_t servoId, uint8_t policy)
{
	DATA[0] = policy;
	c_io_herkulex_write(RAM, servoId, REG_ACK_POLICY, 1, DATA);
}

/** \brief Configura a poítica de acendimento de leds de alarme.
 *
 * @param servoId ID do servo.
 * @param policy Se '1' os leds piscarão em vermelho quando houver um erro.
 * 				 Se '0', os leds não piscarão quando houver um erro no servo.
 */
void c_io_herkulex_configLedPolicy(uint8_t servoId, uint8_t policy)
{
	DATA[0] = policy;
	c_io_herkulex_write(RAM, servoId, REG_ACK_POLICY, 1, DATA);
}

/** \brief Função de acendimento dos leds.
 *
 * @param servoId ID do servo.
 * @param led Combinação OR de LED_GREEN, LED_BLUE e LED_RED.
 */
void c_io_herkulex_ledControl(uint8_t servoId, uint8_t led)
{
	DATA[0] = led;
	c_io_herkulex_write(RAM, servoId, REG_LED_CONTROL, 1, DATA);
}

/** \brief Limpa os erros do servo, e apaga os leds de alarme.
 *
 * @param servoId ID do servo.
 */
void c_io_herkulex_clear(uint8_t servoId)
{
	DATA[0] = 0;
	DATA[1] = 0;
	c_io_herkulex_write(RAM, servoId, REG_STATUS_ERROR, 2, DATA);
}

/** \brief Configura o status do torque.
 *
 * @param servoId ID do servo.
 * @param control
 * 					-TORQUE_FREE: torque desligado, mas o eixo é livre para se
 * 						mover.
 * 				  	-TORQUE_BREAK: Torque desligado, mas o eixo fica travado.
 * 				  	-TORQUE_ON: Liga o torque. O torque deve ser ligado tanto
 * 				  		para os modos de rotação contínua e de posição.
 */
void c_io_herkulex_setTorqueControl(uint8_t servoId, uint8_t control)
{
	DATA[0] = control;
	c_io_herkulex_write(RAM, servoId, REG_TORQUE_CONTROL, 1, DATA);
}

/** \brief Configura o baudrate do servo.
 *
 * \details É necessário reiniciar o servo para que o baudrate mude.
 *
 * @param servoId ID do servo.
 * @param baudrate Baud rate. O servo só aceita alguns valores, como pode ser
 * 					verificado no 'switch'.
 */
void c_io_herkulex_setBaudRate(uint8_t servoId, int baudrate)
{
	switch (baudrate) {
	case 57600:
		baudrate = 0x22;
		break;
	case 115200:
		baudrate = 0x10;
		break;
	case 200000:
		baudrate = 0x09;
		break;
	case 250000:
		baudrate = 0x07;
		break;
	case 400000:
		baudrate = 0x04;
		break;
	case 500000:
		baudrate = 0x03;
		break;
	case 666666:
		baudrate = 0x02;
		break;
	default:
		baudrate = -1;
	}
	if (baudrate > 0) {
		DATA[0] = baudrate;
		c_io_herkulex_write(EEP, servoId, EEP_BAUD_RATE, 1, DATA);
	}
}


/** Interface de Controle */
/** \brief Le a posição do servo.
 *
 * \details Envia um comando de leitura e espera a resposta do servo.
 *
 * @param servoId Id do servo.
 * @return Booleano que indica se a leitura ocorreu com sucesso.
 */
int8_t c_io_herkulex_readPosition(uint8_t servoId)
{
	if (!c_io_herkulex_read(RAM, servoId, REG_ABSOLUTE_POS, 2))
		return 0;

	position = c_io_herkulex_raw2pos(DATA);


	return 1;
}

/** \brief Le a velocidade angular.
 *
 * \details Envia um comando de leitura da velocidade angular e espera a
 * 			resposta do servo.
 *
 * @param servoId Id do servo.
 * @return Booleano que indica se a leitura ocorreu com sucesso.
 */
int8_t c_io_herkulex_readVelocity(uint8_t servoId)
{
	if (!c_io_herkulex_read(RAM, servoId, REG_DIFFERENTIAL_POS, 2))
		return 0;

	velocity = c_io_herkulex_raw2vel(DATA);

	return 1;
}

/** \brief Le velocidade angular e posição do servo.
 *
 * @param servoId Id do servo.
 * @return booleano que indica se a leitura ocorreu com sucesso.
 */
int8_t c_io_herkulex_readData(uint8_t servoId)
{
	if (!c_io_herkulex_read(RAM, servoId, REG_ABSOLUTE_POS, 4))
			return 0;

	position = c_io_herkulex_raw2pos(DATA);
	velocity = c_io_herkulex_raw2vel(DATA + 2);

	return 1;
}

/** \brief retorna a ultima posição lida por c_io_herkulex_read{Position,Data} */
float c_io_herkulex_getPosition(void)
{
	return position;
}

/** \brief retorna a ultima velocidade lida por c_io_herkulex_read{Velocity,Data} */
float c_io_herkulex_getVelocity(void)
{
	return velocity;
}

/** \brief Envia um comando de torque.
 *
 *	\details Usa sjog.
 *
 * @param servoId
 * @param pwm Valor de PWM do torque entre -1023 e 1023
 */
void c_io_herkulex_setTorque(uint8_t servoId, int16_t pwm)
{
	int8_t sign = 0;

	if (pwm < 0) {
		pwm = pwm * -1;
		sign = 1;
	}
	if (pwm > 1023)
		pwm = 1023;

	pwm |= sign << 14;
	pv_sjog_herkulex sjog =  c_io_herkulex_createSjog(servoId, pwm, 0,
			ROTATION_MODE, 0, 0);
	c_io_herkulex_sendSjog(&sjog, 1, 0);
}

/** \brief Envia 1 pacote de comando de torque a 2 servos.
 *
 * \details Usa sjog.
 *
 * @param servoId1 ID do servo 1.
 * @param pwm1 torque do servo 1, em PWM entre -1023 e 1023.
 * @param servoId2 ID do servo 2.
 * @param pwm2 torque do servo 2, em PWM entre -1023 e 1023.
 */
void c_io_herkulex_setTorque2Servos(uint8_t servoId1, int16_t pwm1,
	uint8_t servoId2, int16_t pwm2)
{
	int8_t sign1 = 0, sign2 = 0;

	if (pwm1 < 0) {
		pwm1 = pwm1 * -1;
		sign1 = 1;
	}
	if (pwm1 > 1023)
		pwm1 = 1023;

	if (pwm2 < 0) {
		pwm2 = pwm2 * -1;
		sign1 = 1;
	}
	if (pwm2 > 1023)
		pwm2 = 1023;

	pwm1 |= sign1 << 14;
	pv_sjog_herkulex sjog[2];

	sjog[0] = c_io_herkulex_createSjog(servoId1, pwm1, 0, ROTATION_MODE,
		0, 0);

	pwm2 |= sign2 << 14;
	sjog[1] =  c_io_herkulex_createSjog(servoId2, pwm2, 0, ROTATION_MODE,
		0, 0);
	c_io_herkulex_sendSjog(sjog, 2, 0);
}

/** \brief Muda a referência de posição.
 *
 * @param servoId ID do servo.
 * @param position posição de referência em radianos.
 */
void c_io_herkulex_setPosition(uint8_t servoId, float position)
{
	int16_t rawPos = c_io_herkulex_deg2raw(rad2deg(position));
	pv_sjog_herkulex sjog = c_io_herkulex_createSjog(servoId, rawPos, 0,
			POSITION_MODE, 0, 0);
	c_io_herkulex_sendSjog(&sjog, 1, 0);
}

/** \brief Muda a referência de posição.
 *
 * @param servoId ID do servo.
 * @param position posição de referência em radianos.
 */
void c_io_herkulex_setPosition2Servos(uint8_t servoId1, float pos1,
		uint8_t servo2Id2, float pos2)
{
	int16_t raw_pos1 = c_io_herkulex_deg2raw(rad2deg(pos1));
	int16_t raw_pos2 = c_io_herkulex_deg2raw(rad2deg(pos2));
	pv_sjog_herkulex sjog[2];

	sjog[0] = c_io_herkulex_createSjog(servoId1, raw_pos1, 0,
		POSITION_MODE, 0, 0);
	sjog[1] = c_io_herkulex_createSjog(servo2Id2, raw_pos2, 0,
		POSITION_MODE, 0, 0);
	c_io_herkulex_sendSjog(sjog, 2, 0);
}


/** status */
/** \brief Retorna o código do 'status error'. */
uint8_t c_io_herkulex_getStatusError(void)
{
	return statusError;
}

/** \brief Retorna o código do 'status detail'. */
uint8_t c_io_herkulex_getStatusDetail(void)
{
	return statusDetail;
}

/** \brief Retorna o status do servo. */
uint8_t c_io_herkulex_getStatus(void)
{
	return status;
}


/** Auxiliary functions */
/** \brief Transforma o código de erro em string.
 *
 * @param dest String de retorno do resultado.
 * @param status_error Código de erro.
 * @param status_detail Código de erro detalhado.
 */
void c_io_herkulex_decodeError(char *dest, int8_t status_error,
		int8_t status_detail)
{
	int n;

	dest[0] = 0;
	if (status_error & 0x01)
		strcat(dest, "Exceed input voltage limit, ");

	if (status_error & 0x02)
		strcat(dest, "Exceed alowed POT limit, ");

	if (status_error & 0x04)
		strcat(dest, "Exceed temperature limit, ");

	if (status_error & 0x08) {
		strcat(dest, "Invalid packet: ");
		if (status_detail & 0x04)
			strcat(dest, "Checksum Error, ");
		if (status_detail & 0x08)
			strcat(dest, "Unknown Command, ");
		if (status_detail & 0x10)
			strcat(dest, "Exceed REG range, ");
		if (status_detail & 0x20)
			strcat(dest, "Garbage detected, ");
	}
	if (status_error & 0x10)
		strcat(dest, "Overload detected, ");

	if (status_error & 0x20)
		strcat(dest, "Driver fault detected, ");

	if (status_error & 0x40)
		strcat(dest, "EEP reg distorted, ");

	n = strlen(dest);

	dest[n - 2] = 0;
}

/** \brief Cria uma struct pv_sjog_herkulex.
 *
 * @param servoId ID do servo.
 * @param data PWM ou posição desejada. Até +/- 1023.
 * @param stop bit de parada do servo:
 * 					-1 para o servo;
 * 					-0 para outros comandos.
 * @param mode modo de operação:
 * 							-ROTATION_MODE: modo de rotação contínua;
 * 							-POSITION_MODE: modo de posição.
 * @param led Indica quais leds deveram estar acessos e apagados.
 * 				Usar as constantes LED_GREEN, LED_BLUE e LED_RED
 *
 * @param invalid Indica que o sjog é inválido.
 * @return A estrutura pv_sjog_herkulex com os parametros desejados.
 */
pv_sjog_herkulex c_io_herkulex_createSjog(uint8_t servoId, int16_t data,
		uint8_t stop, uint8_t mode, uint8_t led, uint8_t invalid)
{
	pv_sjog_herkulex sjog;

	sjog.iJogData = data;
	sjog.uiStop = stop;
	sjog.uiMode = mode;
	sjog.uiLed = led;
	sjog.ucID = servoId;
	sjog.uiReserved1 = 0;
	sjog.uiJogInvalid = invalid;
	sjog.uiReserved2 = 0;

	return sjog;
}

/** \brief Cria uma struct pv_ijog_herkulex.
 *
 * @param servoId ID do servo.
 * @param data PWM ou posição desejada. Até +/- 1023.
 * @param stop bit de parada do servo:
 * 					-1 para o servo;
 * 					-0 para outros comandos.
 * @param mode modo de operação:
 * 							-ROTATION_MODE: modo de rotação contínua;
 * 							-POSITION_MODE: modo de posição.
 * @param led Indica quais leds deveram estar acessos e apagados.
 * 				Usar as constantes LED_GREEN, LED_BLUE e LED_RED
 *
 * @param ptime Play time. É o tempo em que o servo deve chega a posição de destino.
 * @return A estrutura pv_ijog_herkulex com os parametros desejados.
 */
pv_ijog_herkulex c_io_herkulex_createIjog(uint8_t servoId, int16_t data,
		uint8_t stop, uint8_t mode, uint8_t led, uint8_t ptime)
{
	pv_ijog_herkulex ijog;

	ijog.iJogData = data;
	ijog.uiStop = stop;
	ijog.uiMode = mode;
	ijog.uiLed = led;
	ijog.ucID = servoId;
	ijog.uiReserved1 = 0;
	ijog.uiJogInvalid = 0;
	ijog.uiReserved2 = 0;
	ijog.ucJogTime_ms = ptime;


	return ijog;
}

/* @}
 * @}
 */
