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
#include <math.h>

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
// Request Packet
#define EEP_WRITE_REQ  0x01
#define EEP_READ_REQ   0x02
#define RAM_WRITE_REQ  0x03
#define RAM_READ_REQ   0x04
#define I_JOG_REQ     0x05
#define S_JOG_REQ     0x06
#define STAT_REQ      0x07
#define ROLLBACK_REQ  0x08
#define REBOOT_REQ    0x09
//ACK Packet
#define EEP_WRITE_ACK  0x41
#define EEP_READ_ACK   0x42
#define RAM_WRITE_ACK  0x43
#define RAM_READ_ACK   0x44
#define I_JOG_ACK  0x45
#define S_JOG_ACK  0x46
#define STAT_ACK   0x47
#define ROLLBACK_ACK  0x48
#define REBOOT_ACK    0x49

//Addresses
#define MODEL_NO1_EEP  0
#define MODEL_NO2_EEP  1
#define VERSION1_EEP   2
#define VERSION2_EEP   3
#define BAUD_RATE_EEP  4
#define SERVO_ID_EEP   6
#define SERVO_ID_RAM   0
#define ACK_POLICY_EEP   7
#define ACK_POLICY_RAM   1
#define ALARM_LED_POLICY_EEP   8
#define ALARM_LED_POLICY_RAM   2
#define TORQUE_POLICY_EEP   9
#define TORQUE_POLICY_RAM   3
#define MAX_TEMP_EEP   11
#define MAX_TEMP_RAM   5
#define MIN_VOLTAGE_EEP   12
#define MIN_VOLTAGE_RAM   6
#define MAX_VOLTAGE_EEP   13
#define MAX_VOLTAGE_RAM   7
#define ACCELERATION_RATIO_EEP   14
#define ACCELERATION_RATIO_RAM   8
#define MAX_ACCELERATION_TIME_EEP   15
#define MAX_ACCELERATION_TIME_RAM   9
#define DEAD_ZONE_EEP   16
#define DEAD_ZONE_RAM   10
#define SATURATOR_OFFSET_EEP  17
#define SATURATOR_OFFSET_RAM  11
#define SATURATOR_SLOPE_EEP   18
#define SATURATOR_SLOPE_RAM   12
#define PWM_OFFSET_EEP   20
#define PWM_OFFSET_RAM   14
#define MIN_PWM_EEP   21
#define MIN_PWM_RAM   15
#define MAX_PWM_EEP   22
#define MAX_PWM_RAM   16
#define OVERLOAD_PWM_THRESHOLD_EEP   24
#define OVERLOAD_PWM_THRESHOLD_RAM   18
#define MIN_POSITION_EEP   26
#define MIN_POSITION_RAM   20
#define MAX_POSITION_EEP   28
#define MAX_POSITION_RAM   22
#define POSITION_KP_EEP   30
#define POSITION_KP_RAM   24
#define POSITION_KD_EEP   32
#define POSITION_KD_RAM   26
#define POSITION_KI_EEP   34
#define POSITION_KI_RAM   28
#define POSITION_FEEDFORWARD_GAIN1_EEP   36
#define POSITION_FEEDFORWARD_GAIN1_RAM   30
#define POSITION_FEEDFORWARD_GAIN2_EEP   38
#define POSITION_FEEDFORWARD_GAIN2_RAM   32
#define VELOCITY_KP_EEP   40
#define VELOCITY_KP_RAM   34
#define VELOCITY_KI_EEP   42
#define VELOCITY_KI_RAM   36
#define LED_BLINK_PERIOD_EEP   44
#define LED_BLINK_PERIOD_RAM   38
#define ADC_FAULT_CHECK_PERIOD_EEP   45
#define ADC_FAULT_CHECK_PERIOD_RAM   39
#define PACKET_GARBAGE_CHECK_PERIOD_EEP   46
#define PACKET_GARBAGE_CHECK_PERIOD_RAM   40
#define STOP_DETECTION_PERIOD_EEP   47
#define STOP_DETECTION_PERIOD_RAM   41
#define OVERLOAD_DETECTION_PERIOD_EEP  48
#define OVERLOAD_DETECTION_PERIOD_RAM  42
#define STOP_THRESHOLD_EEP   49
#define STOP_THRESHOLD_RAM   43
#define INPOSITION_MARGIN_EEP   50
#define INPOSITION_MARGIN_RAM   44
#define CALIBRATION_DIFF_EEP   53
#define CALIBRATION_DIFF_RAM   47
#define STATUS_ERROR_RAM   48
#define STATUS_DETAIL_RAM  49
#define TORQUE_CONTROL_RAM  52
#define LED_CONTROL_RAM  53
#define VOLTAGE_RAM  54
#define TEMPERATURE_RAM  55
#define CURRENT_CONTROL_MODE_RAM  56
#define TICK_RAM 57
#define CALIBRATED_POSITION_RAM  58
#define ABSOLUTE_POSITION_RAM  60
#define DIFFERENTIAL_POSITION_RAM  62
#define PWM_RAM  64
#define ABSOLUTE_GOAL_POSITION_RAM  68
#define ABSOLUTE_DESIRED_TRAJECTORY_POSITION  70
#define DESIRED_VELOCITY_RAM  72

#define BROADCAST_ID   0xFE
#define TORQUE_ON      0x60
#define TORQUE_FREE    0x00
#define TORQUE_BREAK   0x40
#define POSITION_MODE  0x00
#define ROTATION_MODE  0x01

//Leds
#define LED_OFF   0
#define LED_GREEN 1
#define LED_BLUE  2
#define LED_RED   4

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
//General Conmmands
void c_io_herkulex_init(USART_TypeDef *usartn, int baudrate);
void c_io_herkulex_initialize();
void  c_io_herkulex_ACK(char valueACK);
uint8_t  c_io_herkulex_model();
void  c_io_herkulex_set_ID(char ID_Old,char ID_New);
void  c_io_herkulex_clearError(char servoID);
void  c_io_herkulex_torqueON(char servoID);
void  c_io_herkulex_torqueOFF(char servoID);
void  c_io_herkulex_setLed(char servoID, char valueLed);

//Moviments for only one servo
void  c_io_herkulex_setAngleOne(char servoID, float angle, char pTime, int iLed);
void  c_io_herkulex_setTorqueOne(char servoID, int Goal, char pTime, int iLed); // move one servo with continous rotation

//Moviments for two or more servos
void  c_io_herkulex_setAngleAll(char servoID, float angle, int iLed);
void  c_io_herkulex_setTorqueAll(char servoID, int Goal, int iLed);

//Read data
float c_io_herkulex_getAngle(char servoID);
float c_io_herkulex_getSpeed(char servoID);


//void  c_io_herkulex_readRegistryEEP(int servoID, int address, int writeByte);
//void  c_io_herkulex_readRegistryRAM(int servoID, int address, int writeByte);
void  c_io_herkulex_write_EEP(char servoID, char address, char* writeBytes,int lenght);
void  c_io_herkulex_write_RAM(char servoID, char address, char* writeBytes,int lenght);
//void  c_io_herkulex_write_I_JOG(int servoID, int address, int writeByte);
void  c_io_herkulex_write_S_JOG(char pTime);
char  c_io_herkulex_status(char servoID);
//void  c_io_herkulex_rollback(int servoID);
void  c_io_herkulex_reboot(char servoID);

#ifdef __cplusplus
}
#endif

#endif /* BASE_MODULES_IO_C_IO_HERKULEX_H_ */
