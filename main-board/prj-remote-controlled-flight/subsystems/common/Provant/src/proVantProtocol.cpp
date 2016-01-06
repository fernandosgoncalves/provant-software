/*
 *
 *-----------------------------------------------------------------------------
 *  Filename:    proVantProtocol.cpp
 *  Implementation of ProVant communication protocol
 *  Created on: 24/09/2014
 *  Author: Fernando S. Gonçalves
 *-----------------------------------------------------------------------------
 *     ___                           _
 *    / _ \_ __ ___/\   /\__ _ _ __ | |_
 *   / /_)/ '__/ _ \ \ / / _` | '_ \| __|
 *  / ___/| | | (_) \ V / (_| | | | | |_
 *  \/    |_|  \___/ \_/ \__,_|_| |_|\__|
 *
 *-----------------------------------------------------------------------------
 *
 */

#include "proVantProtocol.h"
#include "multwii_const.h"    
#include "provant_const.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
using namespace std;

typedef union _FLOATCONV{
	float f;
	unsigned char b[4];
} _FLOATCONV;

proVantProtocol::proVantProtocol() {
}

proVantProtocol::~proVantProtocol() {
}

void proVantProtocol::init(char port[256], int baudRate) {
	UARTX.setPort(port);
	UARTX.setBaudRate(baudRate);
	UARTX.init();
}

int proVantProtocol::sendMessage(const char* str){
	return UARTX.writeMsg(str);
}

void proVantProtocol::sendControlData(){
}

int proVantProtocol::updateData(){
	unsigned char byte;
	uint8_t tam, msg;

	UARTX.readByte(&byte);
	while(byte != '$' && UARTX.bytesAvailable() > 0){
		UARTX.readByte(&byte);
	}
	if(UARTX.bytesAvailable() == 0){
		return -1;
	}
	else{
		UARTX.readByte(&byte);
		if(byte == 'M'){
			UARTX.readByte(&byte);
			if(byte == '>'){
				//cout << "Inicio da mensagem!" << endl;
				UARTX.readByte(&byte);
				buffer[0] = byte;
				tam = (uint8_t) byte;
				UARTX.readByte(&byte);
				buffer[1] = byte;
				msg = (uint8_t) byte;
				//printf("%d\n%d\n", tam, msg);
				decodeMessage(tam, msg);
				if(UARTX.bytesAvailable() > 0)
					updateData();
			}
			else{
				return -1;
			}
		}
		else{
			return -1;
		}
		return 0;
	}
}

void proVantProtocol::decodeMessage(uint8_t tam, uint8_t msg){
	uint8_t aux;
	unsigned char auxChar;
	if(confirmCheckSum(tam)){
		switch(msg){
			case MSP_RAW_GPS:
				//printf("GPS Raw message with %d bytes \n", tam);
				vantData.setGpsFix((uint8_t) buffer[2]); 	//GPS_FIX	UINT 8	0 or 1
				vantData.setGpsNumSat((uint8_t) buffer[3]);	//GPS_numSat	UINT 8	
				vantData.setGpsCoordLat(deserialize32(4));	//GPS_coord[LAT]	UINT 32	1 / 10 000 000 deg
				vantData.setGpsCoordLong(deserialize32(8));	//GPS_coord[LON]	UINT 32	1 / 10 000 000 deg
				vantData.setGpsAltitude(deserialize16(12));	//GPS_altitude	UINT 16	meter
				vantData.setGpsSpeed(deserialize16(14));	//GPS_speed	UINT 16	cm/s
				vantData.setGpsGroundCourse(deserialize16(16));	//GPS_ground_course	UINT 16	unit: degree*10
				//printf("Fix %d \nNumSat %d \nLat %d\nLong %d\nAlt %d\nSpeed %d\nGC %d\n", buffer[2], buffer[3], vantData.getGpsCoordLat(), vantData.getGpsCoordLong(), vantData.getGpsAltitude(), vantData.getGpsSpeed(), vantData.getGpsGroundCourse());
			break;
			case MSP_COMP_GPS:
				//printf("GPS Comp message with %d bytes \n", tam);
				vantData.setGpsDistanceToHome(deserialize16(2)); //GPS_distanceToHome	UINT 16	unit: meter
				vantData.setGpsDirectionToHome(deserialize16(4)); //GPS_directionToHome	UINT 16	unit: degree (range [-180;+180])
				vantData.setGpsUpdate((uint8_t) buffer[6]); //GPS_update 		UINT 8	a flag to indicate when a new GPS frame is received (the GPS fix is not dependent of this)
				//printf("DisToHome %d \nDirToHome %d \nUpdate %d\n", vantData.getGpsDistanceToHome(), vantData.getGpsDirectionToHome(), vantData.getGpsUpdate());
			break;
			case MSP_ATTITUDE:
				//printf("Attitude message with %d bytes \n", tam);
				vantData.setRoll(((int16_t)deserialize16(2))/10); //angx	INT 16	Range [-1800;1800] (unit: 1/10 degree)
				vantData.setPitch(((int16_t)deserialize16(4))/10); //angy	INT 16	Range [-900;900] (unit: 1/10 degree)
				vantData.setYaw(((int16_t)deserialize16(6))); //heading	INT 16	Range [-180;180]
				//printf("Roll %f \nPitch %f \nYaw %f\n", vantData.getRoll(), vantData.getPitch(), vantData.getYaw());
			break;
			case MSP_ANALOG:
				//printf("Analog message with %d bytes \n", tam);
				vantData.setVbat((uint8_t) buffer[2]); 		//vbat	UINT 8	unit: 1/10 volt
				vantData.setIntPowerMeterSum(deserialize16(3));	//intPowerMeterSum	UINT 16	
				vantData.setRssi(deserialize16(5)); 		//rssi	UINT 16	range: [0;1023]
				vantData.setAmperage(deserialize16(7)); 	//amperage	UINT 16	
				//printf("Vbat %d\nPowerMeterSum %d\nRssi %d\nAmperage %d\n", vantData.getVbat(), vantData.getIntPowerMeterSum(), vantData.getRssi(), vantData.getAmperage());
			break;
			case MSP_ALTITUDE:
				//printf("Altitude message with %d bytes \n", tam);
				vantData.setEstAlt((int32_t)deserialize32(2));	//EstAlt	INT 32	cm
				vantData.setVario((int16_t)deserialize16(6));	//vario	INT 16	cm/s
				//printf("EstAlt %d\nVario %d\n", vantData.getEstAlt(), vantData.getVario());
			break;
			case MSP_STATUS:
				//printf("Status message with %d bytes \n", tam);
				vantData.setCycleTime(deserialize16(2));	//cycleTime	UINT 16	unit: microseconds
				vantData.setI2cErrorsCount(deserialize16(4));	//i2c_errors_count	UINT 16	
				vantData.setSensor(deserialize16(6));		//sensor	UINT 16	BARO<<1|MAG<<2|GPS<<3|SONAR<<4
				vantData.setFlag(deserialize32(8));		//flag	UINT 32	a bit variable to indicate which BOX are active, the bit position depends on the BOX 	which are configured
				//printf("CycleTime %d\nI2CError %d\nSensor %d\nFlag %d\n", vantData.getCycleTime(), vantData.getI2cErrorsCount(), vantData.getSensor(), vantData.getFlag());
			break;
			case MSP_RC:
				//printf("RC message with %d bytes \n", tam);
				vantData.setChannel(1, deserialize16(2));//rcData[RC_CHANS]	16 x UINT 16	Range [1000;2000] ROLL/PITCH/YAW/THROTTLE/AUX1/AUX2/AUX3/AUX4
				vantData.setChannel(2, deserialize16(4));
				vantData.setChannel(3, deserialize16(6));
				vantData.setChannel(4, deserialize16(8));
				vantData.setChannel(5, deserialize16(10));
				vantData.setChannel(6, deserialize16(12));
				vantData.setChannel(7, deserialize16(14));
				vantData.setChannel(8, deserialize16(16));
				vantData.setChannel(9, deserialize16(18));
				vantData.setChannel(10, deserialize16(20));
				vantData.setChannel(11, deserialize16(22));
				vantData.setChannel(12, deserialize16(24));

				//printf("Ch1 %d\nCh2 %d\nCh3 %d\nCh4 %d\nCh5 %d\nCh6 %d\nCh7 %d\nCh8 %d\nCh9 %d\nCh10 %d\nCh11 %d\nCh12 %d\n", vantData.getChannel(1), vantData.getChannel(2), vantData.getChannel(3), vantData.getChannel(4), vantData.getChannel(5), vantData.getChannel(6), vantData.getChannel(7), vantData.getChannel(8), vantData.getChannel(9), vantData.getChannel(10), vantData.getChannel(11), vantData.getChannel(12));
			break;
			case MSP_RCNORMALIZE:
				//printf("RC Normalize message with %d bytes \n", tam);
				vantData.setNormChannel(1, (int16_t)deserialize16(2));//rcData[RC_CHANS]	16 x UINT 16	Range [1000;2000] ROLL/PITCH/YAW/THROTTLE/AUX1/AUX2/AUX3/AUX4
				vantData.setNormChannel(2, (int16_t)deserialize16(4));
				vantData.setNormChannel(3, (int16_t)deserialize16(6));
				vantData.setNormChannel(4, (int16_t)deserialize16(8));
				vantData.setNormChannel(5, (int16_t)deserialize16(10));
				vantData.setNormChannel(6, (int16_t)deserialize16(12));
				vantData.setNormChannel(7, (int16_t)deserialize16(14));
//				vantData.setNormChannel(8, (int16_t)deserialize16(16));
//				vantData.setNormChannel(9, (int16_t)deserialize16(18));
//				vantData.setNormChannel(10, (int16_t)deserialize16(20));
//				vantData.setNormChannel(11, (int16_t)deserialize16(22));
//				vantData.setNormChannel(12, (int16_t)deserialize16(24));

				//printf("Ch1 %d\nCh2 %d\nCh3 %d\nCh4 %d\nCh5 %d\nCh6 %d\nCh7 %d\nCh8 %d\nCh9 %d\nCh10 %d\nCh11 %d\nCh12 %d\n", vantData.getNormChannel(1), vantData.getNormChannel(2), vantData.getNormChannel(3), vantData.getNormChannel(4), vantData.getNormChannel(5), vantData.getNormChannel(6), vantData.getNormChannel(7), vantData.getNormChannel(8), vantData.getNormChannel(9), vantData.getNormChannel(10), vantData.getNormChannel(11), vantData.getNormChannel(12));
			break;
			case MSP_IDENT:
				//printf("Ident message with %d bytes \n", tam);
				vantData.setVersion(buffer[2]);//VERSION	UINT 8	version of MultiWii
				vantData.setMultitype(buffer[3]);//MULTITYPE	UINT 8	type of multi: /TRI/QUADP/QUADX/BI/GIMBAL/Y6/HEX6/FLYING_WING/Y4/HEX6X/OCTOX8/ OCTOFLATP/OCTOFLATX/AIRPLANE/HELI_120/HELI_90/VTAIL4/HEX6H/SINGLECOPTER/DUALCOPTER
				vantData.setMspVersion(buffer[4]);//MSP_VERSION	UINT 8	not used currently
				vantData.setCapability(deserialize32(5));//capability	UINT 32	A 32 bit variable to indicate capability of FC board. Currently, BIND button is used on first bit, DYNBAL on second, FLAP on third
				
				//printf("Version %d\nMultitype %d\nMspVersion %d\nCapability %d\n", vantData.getVersion(), vantData.getMultitype(), vantData.getMspVersion(), vantData.getCapability());
			break;
			case MSP_MOTOR_PINS:
				//printf("Motor Pins message with %d bytes \n", tam);
				vantData.setPwmPin(1, buffer[2]);	//8*PWM_PIN	8 x UNIT 8	motor pin indication
				vantData.setPwmPin(2, buffer[3]);
				vantData.setPwmPin(3, buffer[4]);
				vantData.setPwmPin(4, buffer[5]);
				vantData.setPwmPin(5, buffer[6]);
				vantData.setPwmPin(6, buffer[7]);
				vantData.setPwmPin(7, buffer[8]);
				vantData.setPwmPin(8, buffer[9]);

				//printf("Ch1 %d\nCh2 %d\nCh3 %d\nCh4 %d\nCh5 %d\nCh6 %d\nCh7 %d\nCh8 %d\n", vantData.getPwmPin(1), vantData.getPwmPin(2), vantData.getPwmPin(3), vantData.getPwmPin(4), vantData.getPwmPin(5), vantData.getPwmPin(6), vantData.getPwmPin(7), vantData.getPwmPin(8));
			break;
			case MSP_MOTOR:
				//printf("Motor Speed message with %d bytes \n", tam);
				vantData.setEscLeftSpeed((int16_t) deserialize16(2));	//Motor*8	16 x UINT 16	Range [1000;2000]
				vantData.setEscRightSpeed((int16_t) deserialize16(4));
				vantData.setMotorSpeed(3, deserialize16(6));
				vantData.setMotorSpeed(4, deserialize16(8));
				vantData.setMotorSpeed(5, deserialize16(10));
				vantData.setMotorSpeed(6, deserialize16(12));
				vantData.setMotorSpeed(7, deserialize16(14));
				vantData.setMotorSpeed(8, deserialize16(16));

				//printf("M1 %f\nM2 %f\nM3 %d\nM4 %d\nM5 %d\nM6 %d\nM7 %d\nM8 %d\n", vantData.getEscLeftSpeed(), vantData.getEscRightSpeed(), vantData.getMotorSpeed(3), vantData.getMotorSpeed(4), vantData.getMotorSpeed(5), vantData.getMotorSpeed(6), vantData.getMotorSpeed(7), vantData.getMotorSpeed(8));
			break;
			case MSP_SERVO:
				//printf("Servo message with %d bytes \n", tam);
				vantData.setServo(1, (int16_t)deserialize16(2)); //Servo*8	16 x UINT 16	Range [1000;2000] The servo order depends on multi type
				vantData.setServo(2, (int16_t)deserialize16(4));
				vantData.setServo(3, (int16_t)deserialize16(6));
				vantData.setServo(4, (int16_t)deserialize16(8));
				vantData.setServoLeft((int16_t)deserialize16(10));
				vantData.setServoRight((int16_t)deserialize16(12));
				vantData.setServo(5, (int16_t)deserialize16(14));
				vantData.setServo(6, (int16_t)deserialize16(16));
				
				//printf("S1 %d\nS2 %d\nS3 %d\nS4 %d\nS5 %f\nS6 %f\nS7 %d\nS8 %d\n", vantData.getServo(1), vantData.getServo(2), vantData.getServo(3), vantData.getServo(4), vantData.getServoLeft(), vantData.getServoRight(), vantData.getServo(5), vantData.getServo(6));
			break;
			case MSP_DEBUG:
				//printf("Debug message with %d bytes \n", tam);
				vantData.setDebug(1, (int16_t)deserialize16(2));
				vantData.setDebug(2, (int16_t)deserialize16(4));
				vantData.setDebug(3, (int16_t)deserialize16(6));
				vantData.setDebug(4, (int16_t)deserialize16(8));
				//printf("D1 %d\nD2 %d\nD3 %d\nD4 %d\n", vantData.getDebug(1), vantData.getDebug(2), vantData.getDebug(3), vantData.getDebug(4));
			break;
			case MSP_RAW_IMU:
				//printf("Raw IMU message with %d bytes \n", tam);
				vantData.setGyro(1, (int16_t)deserialize16(2)); //gyrx	INT 16	unit: it depends on GYRO sensor. For MPU6050, 1 unit = 1/4.096 deg/s
				vantData.setGyro(2, (int16_t)deserialize16(4)); //gyry	INT 16	
				vantData.setGyro(3, (int16_t)deserialize16(6)); //gyrz	INT 16	
				vantData.setAcc(1, (int16_t)deserialize16(8)); //accx	INT 16	unit: it depends on ACC sensor and is based on ACC_1G definition MMA7455 64 / MMA8451Q 512 / ADXL345 265 / BMA180 255 / BMA020 63 / NUNCHUCK 200 / LIS3LV02 256 / LSM303DLx_ACC 256 / MPU6050 512 / LSM330 256
				vantData.setAcc(2, (int16_t)deserialize16(10)); //accy	INT 16	
				vantData.setAcc(3, (int16_t)deserialize16(12)); //accz	INT 16	
				vantData.setMag(1, (int16_t)deserialize16(14)); //magx	INT 16	unit: it depends on MAG sensor.
				vantData.setMag(2, (int16_t)deserialize16(16)); //magy	INT 16	
				vantData.setMag(3, (int16_t)deserialize16(18)); //magz	INT 16	
	
				//printf("G1 %d\nG2 %d\nG3 %d\nA1 %d\nA2 %d\nA3 %d\nM1 %d\nM2 %d\nM3 %d\n", vantData.getGyro(1), vantData.getGyro(2), vantData.getGyro(3), vantData.getAcc(1), vantData.getAcc(2), vantData.getAcc(3), vantData.getMag(1), vantData.getMag(2), vantData.getMag(3));
			break;
			case MSP_CONTROLDATAIN:
				//printf("Control Data in message with %d bytes \n", tam);
				vantData.setRoll(decodeFloat(2));
				vantData.setPitch(decodeFloat(6));
				vantData.setYaw(decodeFloat(10));

				vantData.setDotRoll(decodeFloat(14));
				vantData.setDotPitch(decodeFloat(18));
				vantData.setDotYaw(decodeFloat(22));

				vantData.setX(decodeFloat(26));
				vantData.setY(decodeFloat(30));
				vantData.setZ(decodeFloat(34));

				vantData.setDotX(decodeFloat(38));
				vantData.setDotY(decodeFloat(42));
				vantData.setDotZ(decodeFloat(46));

				//printf("Roll %f\nPitch %f\nYaw %f\nDRoll %f\nDPitch %f\nDYaw %f\nX %f\nY %f\nZ %f\nDX %f\nDY %f\nDZ %f\n", vantData.getRoll(), vantData.getPitch(), vantData.getYaw(), vantData.getDotRoll(), vantData.getDotPitch(), vantData.getDotYaw(), vantData.getX(), vantData.getY(), vantData.getZ(), vantData.getDotX(), vantData.getDotY(), vantData.getDotZ());
			break;
			case MSP_CONTROLDATAOUT:
				//printf("Control Data Out message with %d bytes \n", tam);
				vantData.setServoLeft(decodeFloat(2));
				vantData.setEscLeftNewtons(decodeFloat(6));
				vantData.setEscLeftSpeed(decodeFloat(10));
				vantData.setServoRight(decodeFloat(14));
				vantData.setEscRightNewtons(decodeFloat(18));
				vantData.setEscRightSpeed(decodeFloat(22));

				//printf("ServoLeft %f\nEscLeftNewton %f\nEscLeftSpeed %f\nServoRight %f\nEscRightNewton %f\nEscRightSpeed %f\n", vantData.getServoLeft(), vantData.getEscLeftNewtons(), vantData.getEscLeftSpeed(), vantData.getServoRight(), vantData.getEscRightNewtons(), vantData.getEscRightSpeed());
			break;
			case MSP_ESCDATA:
				//printf("ESC Data message with %d bytes \n", tam);
				vantData.setRpm(0, (int)deserialize16(2));
				vantData.setCurrent(0, decodeFloat(4));
				vantData.setVoltage(0, decodeFloat(8));
				vantData.setRpm(1, (int)deserialize16(12));
				vantData.setCurrent(1, decodeFloat(14));
				vantData.setVoltage(1, decodeFloat(18));
				//printf("RPM1 %f\nCurrent1 %f\nVoltage1 %f\nRPM2 %f\nCurrent2 %f\nVoltage2 %f\n", vantData.getRpm(0), vantData.getCurrent(0), vantData.getVoltage(0), vantData.getRpm(1), vantData.getCurrent(1), vantData.getVoltage(1));
			break;
			case MSP_DEBUG2:
				//printf("ESC Data message with %d bytes \n", tam);
				vantData.setDebug2(0, decodeFloat(2));
				vantData.setDebug2(1, decodeFloat(6));
				vantData.setDebug2(2, decodeFloat(10));
				vantData.setDebug2(3, decodeFloat(14));
				std::cout<<"d1"<<vantData.getDebug(1);
				//printf("RPM1 %f\nCurrent1 %f\nVoltage1 %f\nRPM2 %f\nCurrent2 %f\nVoltage2 %f\n", vantData.getRpm(0), vantData.getCurrent(0), vantData.getVoltage(0), vantData.getRpm(1), vantData.getCurrent(1), vantData.getVoltage(1));
			break;
		}
	}
}

bool proVantProtocol::confirmCheckSum(uint8_t tam){
	int i;
	unsigned char checksum, auxChar;
	checksum = 0;
	checksum ^= buffer[0];
	checksum ^= buffer[1];	
	for(i = 2; i < tam+2; i++){
		UARTX.readByte(&auxChar);
		buffer[i] = auxChar;
		checksum ^= buffer[i];
		//printf("%d - %u\n", i, auxChar);
	}
	UARTX.readByte(&auxChar);
	//printf("%u, %d, %u, %d\n", checksum, checksum, auxChar, auxChar);
	return (checksum == auxChar);
}

uint16_t proVantProtocol::deserialize16(int posInit){
	uint16_t value;

	value = (buffer[posInit] & 0xff) + ((buffer[posInit+1] & 0xff) << 8);

	//printf("%x -- %x -- %d\n", buffer[posInit], buffer[posInit+1], value);	

	return value;
}

uint32_t proVantProtocol::deserialize32(int posInit){
	uint32_t value;

	value = (buffer[posInit] & 0xff) + ((buffer[posInit+1] & 0xff) << 8) + ((buffer[posInit+2] & 0xff) << 16) + ((buffer[posInit+3] & 0xff) << 24);

	//printf("%x -- %x -- %x -- %x -- %d\n", buffer[posInit], buffer[posInit+1], buffer[posInit+2], buffer[posInit+3], value);	

	return value;	
}

float proVantProtocol::decodeFloat(int posInit){
	_FLOATCONV _float;
	for(int i=0; i<4; i++)
		_float.b[i]=buffer[posInit+i];
	
	//printf("%f", _float.f);
	return _float.f;
}

vant proVantProtocol::getVantData(){
	return this->vantData;
}
/**
 * \brief Funcao para serializar os bytes.
 * @param a byte de entrada
 */
void proVantProtocol::serialize8(uint8_t a)
{
  //Serial.write(a);
  multwii_msg[multwii_msgindex++] = a;
  multwii_msg[multwii_msgindex] = '\0';
  multwii_checksum ^= a &0x00FF;
  if (multwii_msgindex >= _multwiisize)
  {
    multwii_msgindex = 0;
  }
}
/**
 * Retorna o tamanho da pilha
 * @return Retorna o tamanho da pilha
 */
int proVantProtocol::get_raw_size(){
  return multwii_msgindex;
}

/**
 * Retorna a pilha
 * @return Retorna a pilha
 */
char* proVantProtocol::get_raw_String()
{
  multwii_msgindex = 0;
  return multwii_msg;
}

/**
 * Serializa dois bytes na pilha
 * @param a Qualquer representação de 2 bytes.
 */
void proVantProtocol::serialize16(int16_t a)
{
  serialize8((a   ) & 0xFF);
  serialize8((a>>8) & 0xFF);
}

/**
 * Serializa 4 bytes como se fosse 2.
 * @param a Qualquer representação de 4 bytes.
 */
void proVantProtocol::serialize32_as16(int32_t a)
{
  serialize8(((int16_t)a   ) & 0xFF);
  serialize8(((int16_t)a>>8) & 0xFF);
}

/**
 * Serializa 4 bytes na pilha.
 * @param a Qualquer representação de 4 bytes.
 */
void proVantProtocol::serialize32(uint32_t a)
{
  serialize8((a    ) & 0xFF);
  serialize8((a>> 8) & 0xFF);
  serialize8((a>>16) & 0xFF);
  serialize8((a>>24) & 0xFF);
}

void proVantProtocol::serializeFloat(float x)
{
  char * b = (char *) &x;
  for (int i = 0; i < 4; ++i)
    serialize8(b[i]);
}

/**
 * Monta o header da mensagem.
 * @param size        tamanho da mensagem em bytes.
 * @param multwii_msg tipo da mensagem
 */
void proVantProtocol::headSerialResponse(uint8_t size, uint8_t multwii_msg)
{
  serialize8('$');
  serialize8('M');
  serialize8('>');
  multwii_checksum = 0; // start calculating a new multwii_checksum
  serialize8(size);
  serialize8(multwii_msg);
}

/**
 * Finaliza o pacote a ser enviado montando o checksum.
 */
void proVantProtocol::tailSerialReply()
{
  serialize8(multwii_checksum);
}

/*********************End of serial comm************************************/

/* Exported functions ------------------------------------------------------- */
/*********************Begin of ... something************************************/

/**
 * Envia a attitude para a pilha.
 * @param x Roll
 * @param y Pitch
 * @param z Yaw
 */
void proVantProtocol::multwii_attitude(float x,float y,float z)
{
  headSerialResponse(6,MSP_ATTITUDE);
  serialize32_as16(x*10);
  serialize32_as16(y*10);
  serialize32_as16(z);
  tailSerialReply();
}

/**
 * Envia os dados da imu para a pilha.
 * @param acc x,y,z do acelerometro
 * @param gyr x,y,y do giroscopio
 * @param mag x,y,z do magnetometro
 */
void proVantProtocol::multwii_raw_imu(float* acc,float* gyr, float* mag)
{
  headSerialResponse(18,MSP_RAW_IMU);
  serialize32_as16(acc[0]);
  serialize32_as16(acc[1]);
  serialize32_as16(acc[2]);
  serialize32_as16(gyr[0]);

  serialize32_as16(gyr[1]);
  serialize32_as16(gyr[2]);
  serialize32_as16(mag[0]);
  serialize32_as16(mag[1]);
  serialize32_as16(mag[2]);
  tailSerialReply();
}

/**
 * Envia a altitude para a pilha.
 * @param alt   altitude
 * @param vario a derivada da altitude, usado em planadores como feedback pra saber se esta numa termica.
 */
void proVantProtocol::multwii_altitude(float alt, float vario)
{
  headSerialResponse(6,MSP_ALTITUDE);
  serialize32(alt);
  serialize16(vario);
  tailSerialReply();
}

/**
 * Manda para a pilha a identificação do vant como tiltrotor.
 */
void proVantProtocol::multwii_bicopter_identifier()
{
  headSerialResponse(7,MSP_IDENT);
  serialize8(32);
  serialize8(4); //codigo do bicoptero
  serialize8(0); //not used
  serialize32(0);
  tailSerialReply();
}

/**
 * Envia a posição dos pinos para saber a posição dos motores na imagem do multwii
 */
void proVantProtocol::multwii_motor_pins()
{
  headSerialResponse(8, MSP_MOTOR_PINS);
  serialize8(1);
  serialize8(2);
  serialize8(0);
  serialize8(0);

  serialize8(0);
  serialize8(0);
  serialize8(0);
  serialize8(0);
  tailSerialReply();
}

/**
 * Envia os angulos dos motores para a pilha
 * @param angle1 Angulo do servo esquerdo
 * @param angle2 Angulo do servo direito
 * /Todo existe um problema no angle1
 */
void proVantProtocol::multwii_servos(float angle1,float angle2)
{
  headSerialResponse(16,MSP_SERVO);
  serialize16(0);
  serialize16(0);
  serialize16(0);
  serialize16(0);

  serialize16((int)(1500+8.333*angle1));
  serialize16((int)(1500+8.333*angle2));
  serialize16(0);
  serialize16(0);
  tailSerialReply();
}

/**
 * Envia a forca dos motores para a pilha.
 * @param forca_esquerdo forca do motor esquerdo
 * @param forca_direito  forca do motor direito
 */
void proVantProtocol::multwii_motor(float forca_esquerdo,float forca_direito)
{
  headSerialResponse(16, MSP_MOTOR);
  serialize32_as16((int)((forca_esquerdo*100)));
  serialize32_as16((int)((forca_direito*100)));
  serialize32_as16(0);
  serialize32_as16(0);

  serialize32_as16(0);
  serialize32_as16(0);
  serialize32_as16(0);
  serialize32_as16(0);
  tailSerialReply();
}

/**
 * @brief Envia os dados de debug para a pilha.
 * @param debug1 debug1
 * @param debug2 debug2
 * @param debug3 debug3
 * @param debug4 debug4
 */
void proVantProtocol::multwii_debug(float debug1,float  debug2,float  debug3,float debug4)
{
  headSerialResponse(8, MSP_DEBUG);
  serialize32_as16((int)((debug1)));
  serialize32_as16((int)((debug2)));
  serialize32_as16((int)((debug3)));
  serialize32_as16((int)((debug4)));
  tailSerialReply();
}

/**
 * @brief Envia os dados dos escs para a pilha
 * @details Tipo de mensagem provant
 *
 * @param rpm rom do motor
 * @param current corrente medida pelos escs
 * @param voltage tensao medida pelos escs
 */
void proVantProtocol::multwii2_sendEscdata(int rpm[2],float current[2],float voltage[2])
{
  headSerialResponse(20, MSP_ESCDATA);
  for (int i = 0; i < 2; ++i)
  {
        serialize16(rpm[i]);
        serializeFloat(current[i]);
        serializeFloat(voltage[i]);
  }
  tailSerialReply();
}

/**
 * @brief Envia os dados de entrada do controle para a pilha
 * @details Tipo de mensagemm provant para debug do controle
 *
 * @param channel Dados das medidas dos canais realizados pelo receiver
 */
void proVantProtocol::multwii2_rcNormalize(int16_t channel[7])
{
  headSerialResponse(2*7, MSP_RCNORMALIZE);
  for (int i = 0; i < 7; ++i)
  {
        serialize16(channel[i]);
  }
  tailSerialReply();
}

void proVantProtocol::multwii2_sendControldatain(float rpy[3], float drpy[3], float position[3], float velocity[3]){
  headSerialResponse(48, MSP_CONTROLDATAIN);
  for (int i = 0; i < 3; ++i)
    serializeFloat(rpy[i]);
  for (int i = 0; i < 3; ++i)
    serializeFloat(drpy[i]);
  for (int i = 0; i < 3; ++i)
    serializeFloat(position[i]);
  for (int i = 0; i < 3; ++i)
    serializeFloat(velocity[i]);
  tailSerialReply();
}

void proVantProtocol::multwii2_sendControldataout(float servo[2],float escTorque[2], float escRpm[2])
{
  headSerialResponse(24, MSP_CONTROLDATAOUT);
  serializeFloat(servo[0]);
  serializeFloat(escTorque[0]);
  serializeFloat(escTorque[1]);
  serializeFloat(servo[1]);
  serializeFloat(escRpm[0]);
  serializeFloat(escRpm[1]);
  tailSerialReply();
}

void proVantProtocol::multwii_sendstack()
{
  for (int i = 0; i <  get_raw_size(); ++i)
//    c_common_usart_putchar(USARTx,multwii_msg[i]);
	  UARTX.writeByte((uint8_t)multwii_msg[i]);
  get_raw_String(); // limpa a pilha;
}
