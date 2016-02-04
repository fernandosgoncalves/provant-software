/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef C_RC_COMMONS_H
#define C_RC_COMMONS_H

//#include "arm_math.h"
#include "pv_typedefs.h"

#ifdef __cplusplus
 extern "C" {
#endif
/* Exported macros ------------------------------------------------------------*/

// Choose the controller
//#define LQR_ATTITUDE_HEIGHT_CONTROL
#define TORQUE_CONTROL
//#define BACKSTEPPING_ATTITUDE_HEIGHT_CONTROL //Based on Chowdhurry's article
//#define LQR_PATHTRACK_CONTROL  //To be implemented
//#define HINF_PATHTRACK_CONTROL //To be implemented
//#define HMIX_PATHTRACK_CONTROL //To be implemented


#if defined LQR_ATTITUDE_HEIGHT_CONTROL || defined BACKSTEPPING_ATTITUDE_HEIGHT_CONTROL
 #define STABILITY_CONTROL
#elif defined LQR_PATHTRACK_CONTROL || defined HINF_PATHTRACK_CONTROL || defined HMIX_PATHTRACK_CONTROL || defined TORQUE_CONTROL
 #define PATH_TRACKING_CONTROL
#endif


// PID parameters for the backstepping control
//#ifdef BACKSTEPPING_ATTITUDE_HEIGHT_CONTROL
//Valido
// KP=5
// Kv=1.5

#if 1
	 #define KPPHI    	80.0f
	 #define KVPHI    	8.0f
	 #define KIPHI	 	6.0f
	 #define KPTHETA  	40.0f
	 #define KVTHETA  	5.0f
	 #define KITHETA	6.0f
	 #define KPPSI    	18.0f
	 #define KVPSI    	6.0f
	 #define KIPSI	 	0.0f
	 #define KVZ		3.0f
	 #define KPZ		4.5f
	 #define KIZ		0.5f
  #else
	 #define KPPHI    	50.0f
	 #define KVPHI    	5.0f
	 #define KIPHI	 	10.0f
	 #define KPTHETA  	40.0f
	 #define KVTHETA  	5.0f
	 #define KITHETA	10.0f
	 #define KPPSI    	18.0f
	 #define KVPSI    	6.0f
	 #define KIPSI	 	0.0f
	 #define KVZ		0.0f
	 #define KPZ		10.0f
	 #define KIZ		0.5f
#endif
//#endif

 //keeping here to keep it practical, move it to sensors after test
#define SONAR_FILTER_1_ORDER_10HZ
//#define SONAR_FILTER_2_ORDER_10HZ
#define FILTER_SONAR_100ms //average filter of 20 samples. Can be used in conjuction with the discrete filters

#define LIMIT_SONAR_VAR
#define SONAR_MAX_VAR		0.5
#define SONAR_AVERAGE_WINDOW 20

//#define CONTROL_BEAGLE

//Maximum value for the thrust when in manual height mode
#define THRUST_ROTOR_MAX			14.2f   // Newtons. It is the maximum thrust that can be appliesd to a rotor
#define THRUST_FLIGHT_THRESHOLD		1.0f//(M*G-14.0f)	// Newtons. The thrust needed for the aircraft to almost take flight
#define THRUST_MAX_MANUAL			(2.0f*THRUST_ROTOR_MAX-THRUST_FLIGHT_THRESHOLD)	// Newtons. It is the total thrust applied by the brushless motors combined

#define ENABLE_RC_HEIGHT_REFERENCE
#define HEIGHT_REFERENCE_MAX	0.5 //meters
//#define ENABLE_TAKEOFF_PROCEDURE

 //keeping here to keep it practical, move it to sensors after test

#define ATTITUDE_MINIMUM_STEP	0.01// Radians. Minimum change in angle that is passed to the controller
//#define ATTITUDE_MINIMUM_STEP	0.0035f// Radians. Minimum change in angle that is passed to the controller

 // Fixed Sample Time
 #define CONTROL_SAMPLE_TIME 	0.006f

 // Saturation limits for the anti-windup
#define INT_ROLL_LOWER_ER_LIMIT		-0.01
#define INT_ROLL_UPPER_ER_LIMIT		 0.01
#define INT_PITCH_LOWER_ER_LIMIT	-0.01
#define INT_PITCH_UPPER_ER_LIMIT	 0.01
#define INT_YAW_LOWER_ER_LIMIT		-0.01
#define INT_YAW_UPPER_ER_LIMIT		 0.01
#define INT_Z_LOWER_ER_LIMIT		-0.03
#define INT_Z_UPPER_ER_LIMIT		 0.03


#define ATTITUDE_REF_CONTINOUS
//#define ATTITUDE_REF_INCREMENTAL


 // Reference limits for the radio controller
#ifdef ATTITUDE_REF_CONTINOUS
//	#define REF_ROLL_BIAS		0.0 //radians
//	#define REF_ROLL_BIAS		-0.0087 //radians
	#define REF_ROLL_BIAS		0.0175//-0.0087 //radians
	#define REF_ROLL_MAX		0.20 //radians
//	#define REF_PITCH_MAX		0.2 //radians
    #define REF_PITCH_MAX		0.20 //radians
//	#define REF_PITCH_BIAS		0.148 //radians //funciona no chowdhurry
//	#define REF_PITCH_BIAS		-0.0576
	#define REF_PITCH_BIAS		 -0.0349//-0.1169
	#define REF_YAW_MAX			0.0 //radians
	#define REF_Z_MAX			0.5 //meters
	#define REF_Z_INCREMENT     0.05 //meters
#elif defined ATTITUDE_REF_INCREMENTAL
	#define REF_ROLL_INCREMENT	0.5*DEG_TO_RAD
	#define REF_PITCH_INCREMENT	0.5*DEG_TO_RAD
	#define REF_YAW_INCREMENT	0.5*DEG_TO_RAD
#endif
 //Enable the integration in relation to time
//#define ENABLE_INT_Z
 #define ENABLE_INT_ROLL
// #define ENABLE_INT_PITCH
// #define ENABLE_INT_YAW


#ifdef STABILITY_CONTROL
 #define STATE_Z		0
 #define STATE_ROLL		1
 #define STATE_PITCH	2
 #define STATE_YAW		3
 #define STATE_DZ		4
 #define STATE_DROLL	5
 #define STATE_DPITCH	6
 #define STATE_DYAW		7
#elif defined PATH_TRACKING_CONTROL
 #define STATE_X			0
 #define STATE_Y			1
 #define STATE_Z			2
 #define STATE_ROLL			3
 #define STATE_PITCH		4
 #define STATE_YAW			5
 #define STATE_ALPHA_R		6
 #define STATE_ALPHA_L		7
 #define STATE_DX			8
 #define STATE_DY			9
 #define STATE_DZ			10
 #define STATE_DROLL		11
 #define STATE_DPITCH		12
 #define STATE_DYAW			13
 #define STATE_DALPHA_R		14
 #define STATE_DALPHA_L		15
 #define STATE_INT_Z		16
 #define STATE_INT_YAW		17
#endif




//Environment parameters
#define G	9.81f
// Aircraft parameters - B coordinate system origin in IMU center
#define M    2.3f   // mass inf kg of entire aircraft (including the fiber-glass protection
#define L    0.24737f // aircraft's arm length. Y-axis distance between center of rotation(B) and rotor center of mass.
#define H    0.05394f // center of mass displacement in Z-axis
// Aircraft's Moments of Inertia km*m²
#define IXX  0.04794375f
#define IYY  0.01872182f
#define IZZ  0.03666388f
//antigo
//#define IXX  0.00982678f
//#define IYY  0.01008018f
//#define IZZ  0.00979193f


/* Exported types ------------------------------------------------------------*/

/* Exported variables ------------------------------------------------------------*/
bool c_rc_commons_manual_height_control; //If 1 the thrust is controlled with the remote control

/* Exported functions ------------------------------------------------------- */
void c_rc_commons_init(bool manual_height_control);
void set_manual_height_control(bool manual_height_control);
bool get_manual_height_control();

float c_rc_integrate_trapezoidal(float last_integration, float current_value, float last_value, float sample_time);
float c_rc_saturation(float value, float lower_limit, float upper_limit);



#ifdef __cplusplus
}
#endif

#endif
