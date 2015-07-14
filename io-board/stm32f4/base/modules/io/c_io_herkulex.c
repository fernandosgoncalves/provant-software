/*
 * c_io_herkulex.c
 *
 *  Created on: 01/12/2014
 *      Author: iuro
 */

#include "c_io_herkulex.h"

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
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
USART_TypeDef* USARTx;
uint8_t data[30];
uint8_t dataEx[58];
uint8_t moveData[50];
int pSize;
int pID;
int cmd;
int lenghtString;
int ck1;
int ck2;
int conta;
int XOR;
int playTime;
/* Private function prototypes -----------------------------------------------*/
int c_io_herkulex_checksum1(uint8_t* data, int lenghtString);
int c_io_herkulex_checksum2(int XOR);
void c_io_herkulex_addData(int GoalLSB, int GoalMSB, int set, int servoID);
void c_io_herkulex_sendData();
uint8_t c_io_herkulex_readData();
void  c_io_herkulex_write_S_JOG(int pTime);

/* Private functions ---------------------------------------------------------*/
/* Exported functions definitions --------------------------------------------*/
/** \brief Inicializa o usart para comunicacao serial entre servo e discovery
  *
  *
  * @param  None
  * @retval None
  */
void c_io_herkulex_init(USART_TypeDef *usartn, int baudrate){
	USARTx=usartn;
	/* Inicia a usart2 */
	if (USARTx==USART1) {
		c_common_usart1_init(baudrate);
	} else if (USARTx==USART2) {
		c_common_usart2_init(baudrate);
	} else if (USARTx==USART3) {
		c_common_usart3_init(baudrate);
	} else if (USARTx==USART6) {
		c_common_usart6_init(baudrate);
	}
}

// initialize servos
void c_io_herkulex_initialize(){
	uint8_t data_aux[2];
	conta=0;
	lenghtString=0;

	//c_common_utils_delayms(12);
	c_io_herkulex_reboot(BROADCAST_ID);
	c_common_utils_delayus(100);
	c_io_herkulex_torqueON(BROADCAST_ID);//torque free
	c_io_herkulex_ACK(1);
	data_aux[0]=0;
	c_io_herkulex_write_RAM(BROADCAST_ID,ACCELERATION_RATIO_RAM,data_aux,1);
	//set no acceleration time
	data_aux[0]=0;
	c_io_herkulex_write_RAM(BROADCAST_ID,MAX_ACCELERATION_TIME_RAM,data_aux,1);
	//set pwm offset 0
	data_aux[0]=0;
	c_io_herkulex_write_RAM(BROADCAST_ID,PWM_OFFSET_RAM,data_aux,1);
	//min pwm = 0
	data_aux[0]=0;
	c_io_herkulex_write_RAM(BROADCAST_ID,MIN_PWM_RAM,data_aux,1);
    //max pwm >1023 -> no max pwm
	data_aux[1]=0x03;//little endian 0x03FF sent
	data_aux[0]=0xFF;
	c_io_herkulex_write_RAM(BROADCAST_ID,MAX_PWM_RAM,data_aux,2);
	/** set overload pwm register, if overload_pwm>1023, overload is never
	 * activated this is good for data acquisition, but may not be the case for
	 * the tilt-rotor actualy flying.
	 */
	data_aux[0]=0xFF;
	data_aux[1]=0x03;//little endian, 2048 sent
	c_io_herkulex_write_RAM(BROADCAST_ID,OVERLOAD_PWM_THRESHOLD_RAM,data_aux,2);
    data_aux[0]=TORQUE_ON;
    c_io_herkulex_write_RAM(BROADCAST_ID,TORQUE_CONTROL_RAM,data_aux,1);//set torque on
	c_common_utils_delayus(500);
//	c_io_herkulex_set_goal_position(ID,0);

}

// stat
uint8_t c_io_herkulex_status(int servoID){

	pSize = 0x07;			//3.Packet size
	pID   = servoID;			//4.Servo ID - 0XFE=All servos
	cmd   = STAT_REQ;			//5.CMD

	ck1=(pSize^pID^cmd)&0xFE;
    ck2=(~(pSize^pID^cmd))&0xFE ;

	dataEx[0] = (uint8_t)0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header
	dataEx[2] = pSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2

	c_io_herkulex_sendData(dataEx, pSize);
	c_common_utils_delayus(2);
	c_io_herkulex_readData(9); 				// read 9 bytes from serial


	pSize = dataEx[2];           // 3.Packet size 7-58
	pID   = dataEx[3];           // 4. Servo ID
	cmd   = dataEx[4];           // 5. CMD
	data[0]=dataEx[7];
    data[1]=dataEx[8];
    lenghtString=2;


    ck1 = (dataEx[2]^dataEx[3]^dataEx[4]^dataEx[7]^dataEx[8]) & 0xFE;
	ck2=c_io_herkulex_checksum2(ck1);

	if (ck1 != dataEx[5]) return -1; //checksum verify
	if (ck2 != dataEx[6]) return -2;

	return dataEx[7];			// return status
}



// ACK  - 0=No Replay, 1=Only reply to READ CMD, 2=Always reply
void c_io_herkulex_ACK(int valueACK)
{
	pSize = 0x0A;               // 3.Packet size 7-58
	pID   = 0xFE;	            // 4. Servo ID
	cmd   = RAM_WRITE_REQ;          // 5. CMD
	data[0]=0x34;               // 8. Address
	data[1]=0x01;               // 9. Lenght
	data[2]=valueACK;           // 10.Value. 0=No Replay, 1=Only reply to READ CMD, 2=Always reply
	lenghtString=3;             // lenghtData

	ck1=c_io_herkulex_checksum1(data,lenghtString);	//6. Checksum1
	ck2=c_io_herkulex_checksum2(ck1);					//7. Checksum2

	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header
	dataEx[2] = pSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	dataEx[7] = data[0]; 		// Address 52
	dataEx[8] = data[1]; 		// Length
	dataEx[9] = data[2]; 		// Value

	c_io_herkulex_sendData(dataEx, pSize);
}

// model - 1=0101 - 2=0201
uint8_t c_io_herkulex_model()
{
	pSize = 0x09;               // 3.Packet size 7-58
	pID   = 0xFE;	            // 4. Servo ID
	cmd   = EEP_READ_REQ;       // 5. CMD
	data[0]=0x00;               // 8. Address
	data[1]=0x01;               // 9. Lenght
	lenghtString=2;             // lenghtData

	ck1=c_io_herkulex_checksum1(data,lenghtString);	//6. Checksum1
	ck2=c_io_herkulex_checksum2(ck1);					//7. Checksum2

	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header
	dataEx[2] = pSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	dataEx[7] = data[0]; 		// Address
	dataEx[8] = data[1]; 		// Length

	c_io_herkulex_sendData(dataEx, pSize);

	c_common_utils_delayus(100);
	c_io_herkulex_readData(9);

	pSize = dataEx[2];           // 3.Packet size 7-58
	pID   = dataEx[3];           // 4. Servo ID
	cmd   = dataEx[4];           // 5. CMD
	data[0]=dataEx[7];           // 8. 1st byte
	lenghtString=1;              // lenghtData

	ck1=c_io_herkulex_checksum1(data,lenghtString);	//6. Checksum1
	ck2=c_io_herkulex_checksum2(ck1);					//7. Checksum2

	if (ck1 != dataEx[5]) return -1; //checksum verify
	if (ck2 != dataEx[6]) return -2;

	return dataEx[7];			// return status

}

// setID - Need to restart the servo
void c_io_herkulex_set_ID(int ID_Old, int ID_New)
{
	pSize = 0x0A;               // 3.Packet size 7-58
	pID   = ID_Old;		        // 4. Servo ID OLD - original servo ID
	cmd   = EEP_WRITE_REQ;      // 5. CMD
	data[0]=0x06;               // 8. Address
	data[1]=0x01;               // 9. Lenght
	data[2]=ID_New;             // 10. ServoID NEW
	lenghtString=3;             // lenghtData

	ck1=c_io_herkulex_checksum1(data,lenghtString);	//6. Checksum1
	ck2=c_io_herkulex_checksum2(ck1);					//7. Checksum2

	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header
	dataEx[2] = pSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	dataEx[7] = data[0]; 		// Address 52
	dataEx[8] = data[1]; 		// Length
	dataEx[9] = data[2]; 		// Value

	c_io_herkulex_sendData(dataEx, pSize);

}

// clearError
void c_io_herkulex_clearError(int servoID)
{
	pSize = 0x0B;               // 3.Packet size 7-58
	pID   = servoID;     		// 4. Servo ID - 253=all servos
	cmd   = RAM_WRITE_REQ;      // 5. CMD
	data[0]=0x30;               // 8. Address
	data[1]=0x02;               // 9. Lenght
	data[2]=0x00;               // 10. Write error=0
	data[3]=0x00;               // 10. Write detail error=0

	lenghtString=4;             // lenghtData

	ck1=c_io_herkulex_checksum1(data,lenghtString);	//6. Checksum1
	ck2=c_io_herkulex_checksum2(ck1);					//7. Checksum2

	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header
	dataEx[2] = pSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	dataEx[7] = data[0]; 		// Address 52
	dataEx[8] = data[1]; 		// Length
	dataEx[9] = data[2]; 		// Value1
	dataEx[10]= data[3]; 		// Value2

	c_io_herkulex_sendData(dataEx, pSize);
}

// torque on -
void c_io_herkulex_torqueON(int servoID){
	pSize = 0x0A;               // 3.Packet size 7-58
	pID   = servoID;            // 4. Servo ID
	cmd   = RAM_WRITE_REQ;      // 5. CMD
	data[0]=0x34;               // 8. Address
	data[1]=0x01;               // 9. Lenght
	data[2]=0x60;               // 10. 0x60=Torque ON
	lenghtString=3;             // lenghtData

	ck1=c_io_herkulex_checksum1(data,lenghtString);	//6. Checksum1
	ck2=c_io_herkulex_checksum2(ck1);					//7. Checksum2

	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header
	dataEx[2] = pSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	dataEx[7] = data[0]; 		// Address 52
	dataEx[8] = data[1]; 		// Length
	dataEx[9] = data[2]; 		// Torque ON

	c_io_herkulex_sendData(dataEx, pSize);
}

// torque off - the torque is FREE, not Break
void c_io_herkulex_torqueOFF(int servoID)
{
	pSize = 0x0A;               // 3.Packet size 7-58
	pID   = servoID;            // 4. Servo ID
	cmd   = RAM_WRITE_REQ;          // 5. CMD
	data[0]=0x34;               // 8. Address
	data[1]=0x01;               // 9. Lenght
	data[2]=0x00;               // 10. 0x00=Torque Free
	lenghtString=3;             // lenghtData

	ck1=c_io_herkulex_checksum1(data,lenghtString);	//6. Checksum1
	ck2=c_io_herkulex_checksum2(ck1);					//7. Checksum2

	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header
	dataEx[2] = pSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	dataEx[7] = data[0]; 		// Address 52
	dataEx[8] = data[1]; 		// Length
	dataEx[9] = data[2]; 		// Torque Free

	c_io_herkulex_sendData(dataEx, pSize);

}

// reboot single servo - pay attention 253 - all servos doesn't work!
void c_io_herkulex_reboot(int servoID) {

    pSize = 0x07;               // 3.Packet size 7-58
	pID   = servoID;     	    // 4. Servo ID - 253=all servos
	cmd   = REBOOT_REQ;            // 5. CMD
    ck1=(pSize^pID^cmd)&0xFE;
    ck2=(~(pSize^pID^cmd))&0xFE ; ;

	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header
	dataEx[2] = pSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2

	c_io_herkulex_sendData(dataEx, pSize);

}

// LED  - see table of colors
void c_io_herkulex_setLed(int servoID, int valueLed)
{
	pSize   = 0x0A;               // 3.Packet size 7-58
	pID     = servoID;            // 4. Servo ID
	cmd     = RAM_WRITE_REQ;      // 5. CMD
	data[0] = 0x35;               // 8. Address 53
    data[1] = 0x01;               // 9. Lenght
	data[2] = valueLed;           // 10.LedValue
	lenghtString=3;               // lenghtData

	ck1=c_io_herkulex_checksum1(data,lenghtString);	//6. Checksum1
	ck2=c_io_herkulex_checksum2(ck1);					//7. Checksum2

	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header
	dataEx[2] = pSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	dataEx[7] = data[0];        // Address
	dataEx[8] = data[1];       	// Length
	dataEx[9] = data[2];        // Value

	c_io_herkulex_sendData(dataEx, pSize);
}

// move one servo at goal position 0 - 1024
void c_io_herkulex_setAngleOne(int servoID, float angle, int pTime, int iLed)
{
	if (angle > 160.0|| angle < -160.0)
		return;
	int Goal = (int)((angle*180)/(0.325*PI)) + 512;

	if (Goal > 1023 || Goal < 0)
		return;              // speed (goal) non correct

	if ((pTime <0) || (pTime > 2856))
		return;

	// Position definition
	int posLSB=Goal & 0X00FF;								// MSB Pos
	int posMSB=(Goal & 0XFF00) >> 8;						// LSB Pos

	//led
	int iBlue=0;
	int iGreen=0;
	int iRed=0;
	switch (iLed) {
	case 1:
		iGreen=1;
		break;
	case 2:
		iBlue=1;
		break;
	case 3:
		iRed=1;
		break;
	}
	int SetValue=iGreen*4+iBlue*8+iRed*16;	//assign led value

	playTime=(int)((float)pTime/11.2);		// 8. Execution time

	pSize = 0x0C;          			    	// 3.Packet size 7-58
	cmd   = S_JOG_REQ;              				// 5. CMD

	data[0]=posLSB;               			// 8. speedLSB
	data[1]=posMSB;               			// 9. speedMSB
	data[2]=SetValue;                       // 10. Mode=0;
	data[3]=servoID;                    	// 11. ServoID

	pID=servoID^playTime;

	lenghtString=4;             			// lenghtData

	ck1=c_io_herkulex_checksum1(data,lenghtString);		//6. Checksum1
	ck2=c_io_herkulex_checksum2(ck1);						//7. Checksum2

	pID=servoID;

	dataEx[0] = 0xFF;				// Packet Header
	dataEx[1] = 0xFF;				// Packet Header
	dataEx[2] = pSize;	 		    // Packet Size
	dataEx[3] = pID;				// Servo ID
	dataEx[4] = cmd;				// Command Ram Write
	dataEx[5] = ck1;				// Checksum 1
	dataEx[6] = ck2;				// Checksum 2
	dataEx[7] = playTime;  		    // Execution time
	dataEx[8] = data[0];
	dataEx[9] = data[1];
	dataEx[10] = data[2];
	dataEx[11] = data[3];

	c_io_herkulex_sendData(dataEx, pSize);
}

// move one servo with continous rotation
void c_io_herkulex_setTorqueOne(int servoID, long int Goal, int pTime, int iLed)
{
  if (Goal > 1023 || Goal < -1023) return;              // speed (goal) non correct
  if ((pTime <0) || (pTime > 2856)) return;

  long int GoalSpeedSign;
  if (Goal < 0) {
    GoalSpeedSign = (-1)* Goal ;
    GoalSpeedSign |= 0x4000;  //bit n\B014
  }
  else {
    GoalSpeedSign = Goal;
  }
  int speedGoalLSB=GoalSpeedSign & 0X00FF; 		       // MSB speedGoal
  int speedGoalMSB=(GoalSpeedSign & 0xFF00) >> 8;      // LSB speedGoal

  //led
  int iBlue=0;
  int iGreen=0;
  int iRed=0;
  switch (iLed) {
  case 1:
    iGreen=1;
    break;
  case 2:
    iBlue=1;
    break;
  case 3:
    iRed=1;
    break;
  }
  int SetValue=2+iGreen*4+iBlue*8+iRed*16;		//assign led value

  playTime=(int)((float)pTime/11.2);				// 8. Execution time

  pSize = 0x0C;              					// 3.Packet size 7-58
  cmd   = S_JOG_REQ;      					        // 5. CMD
  pID=servoID;

  data[0]=(uint8_t)speedGoalLSB;            			    // 8. speedLSB
  data[1]=(uint8_t)speedGoalMSB;              			// 9. speedMSB
  data[2]=(uint8_t)SetValue;                          	// 10. Mode=0;
  data[3]=(uint8_t)servoID;                    			// 11. ServoID

  pID=servoID^playTime;

  lenghtString=4;             					// lenghtData

  ck1=c_io_herkulex_checksum1(data,lenghtString);				//6. Checksum1
  ck2=c_io_herkulex_checksum2(ck1);							//7. Checksum2



  dataEx[0] = 0xFF;				// Packet Header
  dataEx[1] = 0xFF;				// Packet Header
  dataEx[2] = pSize;	 		// Packet Size
  dataEx[3] = pID;				// Servo ID
  dataEx[4] = cmd;				// Command Ram Write
  dataEx[5] = ck1;				// Checksum 1
  dataEx[6] = ck2;				// Checksum 2
  dataEx[7] = playTime;  		// Execution time
  dataEx[8] = data[0];
  dataEx[9] = data[1];
  dataEx[10] = data[2];
  dataEx[11] = data[3];

  c_io_herkulex_sendData(dataEx, pSize);

}

// move all servo at the same time to a position: servo list building
void c_io_herkulex_setAngleAll(int servoID, float angle, int iLed)
{
	if (angle > 160.0|| angle < -160.0)
		return; // out of the range
	int Goal = (int)((angle*180)/(0.325*PI)) + 512;

	int iMode=0;                   //mode=position
	int iStop=0;                   //stop=0

	// Position definition
	int posLSB=Goal & 0X00FF;					// MSB Pos
	int posMSB=(Goal & 0XFF00) >> 8;			// LSB Pos

	//led
	int iBlue=0;
	int iGreen=0;
	int iRed=0;
	switch (iLed) {
	case 1:
		iGreen=1;
		break;
	case 2:
		iBlue=1;
		break;
	case 3:
		iRed=1;
		break;
	}

	int SetValue=iStop+iMode*2+iGreen*4+iBlue*8+iRed*16;	//assign led value

	c_io_herkulex_addData(posLSB, posMSB, SetValue, servoID);	//add servo data to list, pos mode
}

// move all servo at the same time with different torques: servo list building
void c_io_herkulex_setTorqueAll(int servoID, int Goal, int iLed)
{
	  if (Goal > 1023 || Goal < -1023)
		return;								 //-1023 <--> 1023 range

	  int iMode=1;                  		// mode=continous rotation
	  int iStop=0;                  		// Stop=0

	  // Speed definition
	  int GoalSpeedSign;
	  if (Goal < 0) {
		GoalSpeedSign = (-1)* Goal ;
		GoalSpeedSign |= 0x4000;  //bit n\B014
	  }
	  else {
		GoalSpeedSign = Goal;
	  }

	  int speedGoalLSB=GoalSpeedSign & 0X00FF; 	      		 // MSB speedGoal
	  int speedGoalMSB=(GoalSpeedSign & 0xFF00) >> 8;        // LSB speedGoal

	  //led
	  int iBlue=0;
	  int iGreen=0;
	  int iRed=0;
	  switch (iLed) {
	  case 1:
		iGreen=1;
		break;
	  case 2:
		iBlue=1;
		break;
	  case 3:
		iRed=1;
		break;
	  }

	  int SetValue=iStop+iMode*2+iGreen*4+iBlue*8+iRed*16;	//assign led value

	  c_io_herkulex_addData(speedGoalLSB, speedGoalMSB, SetValue, servoID);		//add servo data to list, speed mode
}



// get Position in rad
float c_io_herkulex_getAngle(int servoID) {
	int Position  = 0;

    pSize = 0x09;               // 3.Packet size 7-58
	pID   = servoID;     	    // 4. Servo ID - 253=all servos
	cmd   = RAM_READ_REQ;           // 5. CMD
	data[0]=0x3A;               // 8. Address
	data[1]=0x02;               // 9. Lenght

	lenghtString=2;             // lenghtData

	ck1=c_io_herkulex_checksum1(data,lenghtString);	//6. Checksum1
	ck2=c_io_herkulex_checksum2(ck1);					//7. Checksum2

	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header
	dataEx[2] = pSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	dataEx[7] = data[0];      	// Address
	dataEx[8] = data[1]; 		// Length

	c_io_herkulex_sendData(dataEx, pSize);

	c_common_utils_delayus(100);
    c_io_herkulex_readData(13);

	pSize = dataEx[2];           // 3.Packet size 7-58
	pID   = dataEx[3];           // 4. Servo ID
	cmd   = dataEx[4];           // 5. CMD
	data[0]=dataEx[7];
    data[1]=dataEx[8];
    data[2]=dataEx[9];
    data[3]=dataEx[10];
    data[4]=dataEx[11];
    data[5]=dataEx[12];
    lenghtString=6;

    ck1=c_io_herkulex_checksum1(data,lenghtString);	//6. Checksum1
	ck2=c_io_herkulex_checksum2(ck1);					//7. Checksum2

    if (ck1 != dataEx[5]) return -1;
	if (ck2 != dataEx[6]) return -1;

	Position = ((dataEx[10]&0x03)<<8) | dataEx[9];

	return (float)(Position-512)*(0.325*PI)/180;
}

// get Position in rad/seg
float c_io_herkulex_getSpeed(int servoID){
	int speed  = 0;

	pSize = 0x09;               // 3.Packet size 7-58
	pID   = servoID;     	   	  // 4. Servo ID
	cmd   = RAM_READ_REQ;           // 5. CMD
	data[0]=0x40;               // 8. Address
	data[1]=0x02;               // 9. Lenght

	lenghtString=2;             // lenghtData

	ck1=c_io_herkulex_checksum1(data,lenghtString);		//6. Checksum1
	ck2=c_io_herkulex_checksum2(ck1);					//7. Checksum2

	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header
	dataEx[2] = pSize;		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	dataEx[7] = data[0]; 	    // Address
	dataEx[8] = data[1]; 		// Length

	c_io_herkulex_sendData(dataEx, pSize);

	c_common_utils_delayus(100);
	c_io_herkulex_readData(13);

	pSize = dataEx[2];           // 3.Packet size 7-58
	pID   = dataEx[3];           // 4. Servo ID
	cmd   = dataEx[4];           // 5. CMD
	data[0]=dataEx[7];
	data[1]=dataEx[8];
	data[2]=dataEx[9];
	data[3]=dataEx[10];
	data[4]=dataEx[11];
	data[5]=dataEx[12];
	lenghtString=6;

	ck1=c_io_herkulex_checksum1(data,lenghtString);	//6. Checksum1
	ck2=c_io_herkulex_checksum2(ck1);				//7. Checksum2

	if (ck1 != dataEx[5]) return -1;
	if (ck2 != dataEx[6]) return -1;

	speed = ((dataEx[10]&0xFF)<<8) | dataEx[9];

	return ((float)speed*29.09)*PI/180;
}

// write registry in the RAM: one byte
void c_io_herkulex_write_RAM(int servoID, int address,uint8_t* writeBytes,int lenght)
{
  pSize = 0x0A;               	// 3.Packet size 7-58
  pID   = servoID;     			// 4. Servo ID - 253=all servos
  cmd   = RAM_WRITE_REQ;          	// 5. CMD
  data[0]=address;              // 8. Address
  data[1]=0x01;               	// 9. Lenght
  lenghtString=2;
  for (int i=0; i < lenght; i++){
	  data[i+2]=writeBytes[i];	 // 10. Write error=0
  	  lenghtString++;            // lenghtData
   }

  ck1=c_io_herkulex_checksum1(data,lenghtString);	//6. Checksum1
  ck2=c_io_herkulex_checksum2(ck1);				//7. Checksum2

  dataEx[0] = 0xFF;			// Packet Header
  dataEx[1] = 0xFF;			// Packet Header
  dataEx[2] = pSize;	 	// Packet Size
  dataEx[3] = pID;			// Servo ID
  dataEx[4] = cmd;			// Command Ram Write
  dataEx[5] = ck1;			// Checksum 1
  dataEx[6] = ck2;			// Checksum 2
  dataEx[7] = data[0]; 		// Address 52
  dataEx[8] = data[1]; 		// Length
  dataEx[9] = data[2]; 		// Value1
  dataEx[10]= data[3]; 		// Value2

  c_io_herkulex_sendData(dataEx, pSize);

}

// write registry in the EEP memory (ROM): one byte
void c_io_herkulex_write_EEP(int servoID, int address, uint8_t* writeBytes,int lenght)
{
  pSize = 0x0A;                  // 3.Packet size 7-58
  pID   = servoID;     	         // 4. Servo ID - 253=all servos
  cmd   = EEP_WRITE_REQ;         // 5. CMD
  data[0]=address;               // 8. Address
  data[1]=0x01;                  // 9. Lenght
  lenghtString=2;
  for (int i=0; i < lenght; i++){
	  data[i+2]=writeBytes[i];	 // 10. Write error=0
	  lenghtString++;            // lenghtData
  }

  ck1=c_io_herkulex_checksum1(data,lenghtString);	//6. Checksum1
  ck2=c_io_herkulex_checksum2(ck1);				//7. Checksum2

  dataEx[0] = 0xFF;			// Packet Header
  dataEx[1] = 0xFF;			// Packet Header
  dataEx[2] = pSize;		// Packet Size
  dataEx[3] = pID;			// Servo ID
  dataEx[4] = cmd;			// Command Ram Write
  dataEx[5] = ck1;			// Checksum 1
  dataEx[6] = ck2;			// Checksum 2
  dataEx[7] = data[0]; 		// Address 52
  dataEx[8] = data[1]; 		// Length
  dataEx[9] = data[2]; 		// Value1
  dataEx[10]= data[3]; 		// Value2

  c_io_herkulex_sendData(dataEx, pSize);

}

/* Private functions ---------------------------------------------------------*/

// checksum1
int c_io_herkulex_checksum1(uint8_t* data, int lenghtString){
	XOR = 0;
	  XOR = XOR ^ pSize;
	  XOR = XOR ^ pID;
	  XOR = XOR ^ cmd;
	  for (int i = 0; i < lenghtString; i++)
	  {
	    XOR = XOR ^ data[i];
	  }
	  return XOR&0xFE;
}

// checksum2
int c_io_herkulex_checksum2(int XOR){
  return (~XOR)&0xFE;
}

void c_io_herkulex_addData(int GoalLSB, int GoalMSB, int set, int servoID){
  moveData[conta++]=GoalLSB;
  moveData[conta++]=GoalMSB;
  moveData[conta++]=set;
  moveData[conta++]=servoID;
}


void c_io_herkulex_sendData(uint8_t* buffer,int lenght) {
	for (int i=0; i < lenght ; ++i)
		c_common_usart_putchar(USARTx,buffer[i]);
}

uint8_t c_io_herkulex_readData(int lenght) {
	long now = c_common_utils_micros();
	long timeOut = now + 100;
//	//Time out para recever os dados
	while (!c_common_usart_available(USARTx) && now<timeOut)
			now=c_common_utils_micros();
	for(int i=0;c_common_usart_available(USARTx)==1 && i<lenght;i++){
			dataEx[i] = c_common_usart_read(USARTx);
	}
	c_common_usart_flush(USARTx);
}

void  c_io_herkulex_write_S_JOG(int pTime){
	if ((pTime <0) || (pTime > 2856))
		return;

    pSize = 0x08 + conta;     	        // 3.Packet size 7-58
	cmd   = S_JOG_REQ;		 			// 5. CMD SJOG Write n servo with same execution time
	playTime=(int)((float)pTime/11.2);  // 8. Execution time

    pID=0xFE^playTime;
    ck1=c_io_herkulex_checksum1(moveData,conta);	    //6. Checksum1
	ck2=c_io_herkulex_checksum2(ck1);				    //7. Checksum2

    pID=0xFE;
	dataEx[0] = 0xFF;				    // Packet Header
	dataEx[1] = 0xFF;				    // Packet Header
	dataEx[2] = pSize;	 			    // Packet Size
	dataEx[3] = pID;				    // Servo ID
	dataEx[4] = cmd;				    // Command Ram Write
	dataEx[5] = ck1;				    // Checksum 1
	dataEx[6] = ck2;				    // Checksum 2
	dataEx[7] = playTime;			    // Execution time

	for (int i=0; i < conta; i++)
		dataEx[i+8]=moveData[i];	    // Variable servo data

	c_io_herkulex_sendData(dataEx, pSize);
	conta=0; 						    //reset counter
}
