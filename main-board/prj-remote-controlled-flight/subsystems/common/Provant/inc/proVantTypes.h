/*
 *-----------------------------------------------------------------------------
 *  Filename:    proVantTypes.h
 *  Typedefs of messages on ProVant
 *  Created on: 13/10/2014
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

#ifndef PROVANTTYPES_H_
#define PROVANTTYPES_H_

#include <stdint.h>   /* Standard types */

namespace proVant{
	typedef struct {
		float roll;
		float pitch;
		float yaw;
		float dotRoll;
		float dotPitch;
		float dotYaw;
	} atitude;

	typedef struct {
		float x;
		float y;
		float z;
		float dotX;
		float dotY;
		float dotZ;
	} position;

	typedef struct {
		float alphar;
		float alphal;
		float dotAlphar;
		float dotAlphal;
	} servos_state;

	typedef struct {
		//Control output:
		float servoLeft;
		float escLeftNewtons;
		float escLeftSpeed;
		float servoRight;
		float escRightNewtons;
		float escRightSpeed;
	} controlOutput;

	typedef struct {
		//Esc data:
		float rpm[2];
		float current[2];
		float voltage[2];
	} escData;

	typedef struct {
		//GPS Raw Data
		uint8_t gpsFix;
		uint8_t gpsNumSat;
		uint32_t gpsCoordLat;
		uint32_t gpsCoordLong; 
		uint16_t gpsAltitude;
		uint16_t gpsSpeed;
		uint16_t gpsGroundCourse;
	} gpsRawData;


	typedef struct {
		//GPS Comp Data
		uint16_t gpsDistanceToHome;
		uint16_t gpsDirectionToHome;
		uint8_t gpsUpdate;
	} gpsCompData;

	typedef struct {
		//Analog
		uint8_t vbat;
		uint16_t intPowerMeterSum;
		uint16_t rssi;
		uint16_t amperage;
	} analog;

	typedef struct {
		//Altitude
		int32_t estAlt;
		int16_t vario;
	} altitude;

	typedef struct {
		//Status
		uint16_t cycleTime;
		uint16_t i2cErrorsCount;
		uint16_t sensor;
		uint32_t flag;
		bool stop;
	} status;

	typedef struct {
		//RC
		uint16_t channels[7];
	} rc;

	typedef struct {
		//RC Normalize
		int16_t normChannels[7];
	} rcNormalize;

	typedef struct {
		//IDENT
		uint8_t version; 
		uint8_t multitype;
		uint8_t mspVersion;
		uint32_t capability;
	} ident;

	typedef struct {
		//Motor Pins
		uint8_t pwmPin[8];
	} motorPins;

	typedef struct {
		//Motor Speed
		uint16_t motorSpeed[6];
	} motorSpeed;

	typedef struct {
		//Servos
		int16_t servos[6];
	} servos;

	typedef struct {
		//Debug
		int16_t debug[4];
	} debug;

	typedef struct {
		//Debug
		float debug[4];
	} debug2;

	typedef struct {
		//Raw Imu Data
		int16_t gyro[3];
		int16_t mag[3];
		int16_t acc[3];
	} rawImuData;

	typedef struct{
		atitude atitudeVant;
		position positionVant;
		servos_state servos_stateVant;
	} states;
}
#endif /* PROVANTTYPES_H_ */
