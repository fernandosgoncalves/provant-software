/**
  ******************************************************************************
  * @file    modules/datapr/c_datapr_MultWii.h
  * @author  Patrick José Pereira
  * @version V1.0.0
  * @date    16-Jul-2014
  * @brief   Funções para envio de dados para a interface de do Multwii
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef C_DATAPR_MULTWII_H
#define C_DATAPR_MULTWII_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_conf.h"
#include "c_common_uart.h"
#include "c_common_utils.h"
#include <math.h>
#include "pv_typedefs.h"

/* FreeRTOS kernel includes */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"


#ifdef __cplusplus
 extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/

/************************************** MultiWii Serial Protocol *******************************************************/
// Multiwii Serial Protocol 0 
#define MSP_VERSION              0

//to multiwii developpers/committers : do not add new MSP messages without a proper argumentation/agreement on the forum
#define MSP_IDENT                100   //out message         multitype + multiwii version + protocol version + capability variable
#define MSP_STATUS               101   //out message         cycletime & errors_count & sensor present & box activation & current setting number
#define MSP_RAW_IMU              102   //out message         9 DOF
#define MSP_SERVO                103   //out message         8 servos
#define MSP_MOTOR                104   //out message         8 motors
#define MSP_RC                   105   //out message         8 rc chan and more
#define MSP_RAW_GPS              106   //out message         fix, numsat, lat, lon, alt, speed, ground course
#define MSP_COMP_GPS             107   //out message         distance home, direction home
#define MSP_ATTITUDE             108   //out message         2 angles 1 heading
#define MSP_ALTITUDE             109   //out message         altitude, variometer
#define MSP_ANALOG               110   //out message         vbat, powermetersum, rssi if available on RX
#define MSP_RC_TUNING            111   //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                  112   //out message         P I D coeff (9 are used currently)
#define MSP_BOX                  113   //out message         BOX setup (number is dependant of your setup)
#define MSP_MISC                 114   //out message         powermeter trig
#define MSP_MOTOR_PINS           115   //out message         which pins are in use for motors & servos, for GUI 
#define MSP_BOXNAMES             116   //out message         the aux switch names
#define MSP_PIDNAMES             117   //out message         the PID names
#define MSP_WP                   118   //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold
#define MSP_BOXIDS               119   //out message         get the permanent IDs associated to BOXes
#define MSP_SERVO_CONF           120   //out message         Servo settings

#define MSP_SET_RAW_RC           200   //in message          8 rc chan
#define MSP_SET_RAW_GPS          201   //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              202   //in message          P I D coeff (9 are used currently)
#define MSP_SET_BOX              203   //in message          BOX setup (number is dependant of your setup)
#define MSP_SET_RC_TUNING        204   //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION      205   //in message          no param
#define MSP_MAG_CALIBRATION      206   //in message          no param
#define MSP_SET_MISC             207   //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208   //in message          no param
#define MSP_SET_WP               209   //in message          sets a given WP (WP#,lat, lon, alt, flags)
#define MSP_SELECT_SETTING       210   //in message          Select Setting Number (0-2)
#define MSP_SET_HEAD             211   //in message          define a new heading hold direction
#define MSP_SET_SERVO_CONF       212   //in message          Servo settings
#define MSP_SET_MOTOR            214   //in message          PropBalance function

#define MSP_BIND                 240   //in message          no param

#define MSP_EEPROM_WRITE         250   //in message          no param

#define MSP_DEBUGMSG             253   //out message         debug string buffer
#define MSP_DEBUG                254   //out message         debug1,debug2,debug3,debug4


#define MSP_ESCDATA              99
#define MSP_CONTROLDATAOUT       98
#define MSP_CONTROLDATAIN        97
#define MSP_RCNORMALIZE          96
#define MSP_DEBUG2               95

/* Private variables ---------------------------------------------------------*/
char multwii_msg[3076];
char multwii_recmsg[3076];
/* Private functions ---------------------------------------------------------*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/************************************************* Serial functions *****************************************************************/
void serialize8(uint8_t a);
char* get_raw_String();
void serialize16(int16_t a);
void serialize32(uint32_t a);
void headSerialResponse(uint8_t size, uint8_t multwii_msg);
void tailSerialReply();
int get_raw_size();
/*********************End of serial comm************************************/
/************************************************* Deserial functions *****************************************************************/
uint16_t deserialize16(int posInit);
uint32_t deserialize32(int posInit);
float decodeFloat(int posInit);
int  c_common_datapr_multiwii_receivestack(USART_TypeDef* USARTx);
void decodeMessage(USART_TypeDef* USARTx,uint8_t tam, uint8_t msg);
bool confirmCheckSum(USART_TypeDef* USARTx,uint8_t tam);
/*********************End of deserial comm************************************/

/* Exported functions ------------------------------------------------------- */
/*********************Begin of ... something************************************/
pv_type_actuation  c_common_datapr_multwii_getactuation();

void c_common_datapr_multwii_attitude(float x,float y,float z);
void c_common_datapr_multwii_raw_imu(float* acc,float* gyr, float* mag);
void c_common_datapr_multwii_altitude(float alt, float vario);
void c_common_datapr_multwii_bicopter_identifier();
void c_common_datapr_multwii_motor_pins();
void c_common_datapr_multwii_motor(float forca_esquerdo,float forca_direito);
void c_common_datapr_multwii_servos(float angle1,float angle2);
void c_common_datapr_multwii_debug(float debug1,float  debug2,float  debug3,float debug4);
void c_common_datapr_multwii2_debug(float debug1,float  debug2,float  debug3,float debug4);
void c_common_datapr_multwii_sendstack(USART_TypeDef* USARTx);
void c_common_datapr_multwii2_sendEscdata(int rpm[2],float current[2],float voltage[2]);
void c_common_datapr_multwii2_rcNormalize(int channel[7]);
void c_common_datapr_multwii2_sendControldatain(float rpy[3], float drpy[3], float position[3], float velocity[3]);
void c_common_datapr_multwii2_sendControldataout(float servo[2], float escTorque[2], float escRpm[2]);

#ifdef __cplusplus
}
#endif

#endif //C_IO_IMU_H
