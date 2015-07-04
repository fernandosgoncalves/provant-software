/**
  ******************************************************************************
  * @file    modules/io/c_io_herkulex.h
  * @author  Iuro Baptista Pereira Nascimento
  * @version V1.0.0
  * @date    01/12/2014
  * @brief   Implementação do Servo Herkulex DRS-0201
  *****************************************************************************/

#ifndef BASE_MODULES_IO_C_IO_HERKULEX_H_
#define BASE_MODULES_IO_C_IO_HERKULEX_H_

#ifdef __cplusplus
 extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_conf.h"
#include "c_common_uart.h"
#include "c_common_utils.h"
#include "pv_typedefs.h"


/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define KV 0.325*PI/(0.0112*180)
#define TIME_OUT 10
#define INTER_PKG_TIME 0.000105
#define HEADER 0xFF
// status error register, 48
#define EXCEED_INPUT_VOLT_LIMIT 0x01
#define EXCEED_ALLOWED_POT_LIMIT 0x02
#define EXCEED_TEMP_LIMIT 0x04
#define INVALID_PACKET 0x08
#define OVERLOAD 0x10
#define DRIVER_FAULT 0x20
#define EEP_REG_DISTORTED 0x40

// status error register, 49
#define MOVING_FLAG 0x01
#define INPOSITION_FLAG 0x02
//invalid packet errors
#define CHECKSUM_ERR 0x04
#define UNKNOWM_COMMAND 0x08
#define EXCEED_REG_RANGE 0x10
#define GARBAGE_DETECTED 0x20
//end of packet errors
#define MOTOR_ON_FLAG 0x40

//Address of registers in RAM
#define REG_SERVO_ID 0
#define REG_ACK_POLICY 1
#define REG_ALARM_LED_POLICY 2
#define REG_TORQUE_POLICY 3
#define REG_MAX_TEMP 5
#define REG_MIN_VOLT 6
#define REG_MAX_VOLT 7
#define REG_ACC_RATIO 8
#define REG_MAX_ACC_TIME 9
#define REG_DEAD_ZONE 10
#define REG_SATURATOR_OFFSET 11
#define REG_SATURATOR_SLOPE 12
#define REG_PWM_OFFSET 14
#define REG_MIN_PWM 15
#define REG_MAX_PWM 16
#define REG_OVERLOAD_PWM_THRESHOLD 18
#define REG_MIN_POS 20
#define REG_MAX_POS 22
#define REG_KP 24
#define REG_KD 26
#define REG_KI 28
#define REG_KFF1 30
#define REG_KFF2 32

//some other registers from voltatile RAM
#define REG_INPOSITION_MARGIN 44
#define REG_CALIBRATION _DIFF 47
#define REG_STATUS_ERROR 48
#define REG_STATUS_DETAIL 49
#define REG_TORQUE_CONTROL 52
#define REG_LED_CONTROL 53
#define REG_VOLT 54
#define REG_TEMP 55
#define REG_CURRENT_CONTROL_MODE 56
#define REG_TICK 57
#define REG_CALIBRATED_POS 58
#define REG_ABSOLUTE_POS 60
#define REG_DIFFERENTIAL_POS 62
#define REG_PWM 64
#define REG_ABSOLUTE_GOAL_POS 68
#define REG_DESIRED_VELOCITY 70

//commands to servo
#define EEP_WRITE 0x01
#define EEP_READ 0x02
#define RAM_WRITE 0x03
#define RAM_READ 0x04
#define I_JOG 0x05
#define S_JOG 0x06
#define STAT 0x07
#define ROLLBACK 0x08
#define REBOOT 0x09

//ack responses
#define ACK_RAM_WRITE 0x43
#define ACK_RAM_READ 0x44
#define ACK_I_JOG 0x45
#define ACK_S_JOG 0x46
#define ACK_STAT 0x47
#define ACK_ROLLBACK 0x48
#define ACK_REBOOT 0x49

#define EEP_BAUD_RATE 0x04

#define RAM 0
#define EEP 1
#define TORQUE_ON 0x60
#define TORQUE_FREE 0x00
#define TORQUE_BREAK 0x40

#define BROADCAST_ADDR 0xFE;

#define POSITION_MODE 0
#define ROTATION_MODE 1

//Leds
#define LED_GREEN 1
#define LED_BLUE 2
#define LED_RED 4

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */


//Direct servo commands
uint8_t  c_io_herkulex_read(char mem, char servo_id, char reg_addr, unsigned char data_length);
uint8_t c_io_herkulex_write(char mem, char servo_id, char reg_addr, unsigned char datalength, char *data);
void c_io_herkulex_ijog();
void c_io_herkulex_sjog(char size, char servo_id, uint16_t data, char stop, char mode, char led, char ptime);
uint8_t c_io_herkulex_stat(uint8_t servo_id);
void c_io_herkulex_rollback();//not implemented yet
void c_io_herkulex_reboot(uint8_t servo_id);
//uint8_t receive();

//Indirect commands
void c_io_herkulex_init(USART_TypeDef *USART, int baudrate);
void c_io_herkulex_config_ack_policy(char servo_id, char policy);
void c_io_herkulex_config_led_policy(char servo_id, char policy);
void c_io_herkulex_led_control(char servo_id, char led);
void c_io_herkulex_clear(uint8_t servo_id);
void c_io_herkulex_set_torque_control(char servo_id, char control);

/** Control Interface
	 *
	 * Functions for feedback, the 1st read the current absolute
	 * position, the second read the angular velocity. The outputs
	 * are converted to degrees and rad/s respectively
	 */
float c_io_herkulex_read_position(uint8_t servo_id);
float c_io_herkulex_read_position_rad(uint8_t servo_id);
float c_io_herkulex_read_velocity(uint8_t servo_id);
int8_t c_io_herkulex_read_data(uint8_t servo_id);
float c_io_herkulex_get_position(uint8_t servo_id);
float c_io_herkulex_get_velocity(uint8_t servo_id);
//set input toque to servo
void c_io_herkulex_set_torque(uint8_t servo_id, int16_t pwm);
void c_io_herkulex_set_goal_position(uint8_t servo_id, float position_deg);
void c_io_herkulex_set_goal_position_rad(uint8_t servo_id, float position_rad);

//status get functions
uint8_t c_io_herkulex_get_status_error();
uint8_t c_io_herkulex_get_status_detail();
uint8_t c_io_herkulex_get_status();

#ifdef __cplusplus
}
#endif

#endif /* BASE_MODULES_IO_C_IO_HERKULEX_H_ */
