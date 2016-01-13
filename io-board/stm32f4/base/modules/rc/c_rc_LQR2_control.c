/**
  ******************************************************************************
  * @file    modules/rc/c_rc_SFC.c
  * @author  Rodrigo Donadel
  * @version V1.0.0
  * @date    08-December-2014
  * @brief   Controle de estabilizacao por realimentacao de estados.
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "c_rc_LQR2_control.h"

/** @addtogroup Module_RC
  * @{
  */

/** @addtogroup Module_RC_Component_State_Feedback_Control
  * \brief Controle de estabilização para o modo de operação RC por realimentacao de estados.
  *
   * @{
  */

		//---------------------------------------------------------------------------------------------

/* Exported functions definitions --------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

// Matrizes de ganho
float32_t c_rc_LQR2_Ke_f32[4][18]={
{0.000000,0.000000,-1.371853,0.000000,1.000000,-39.724509,0.000000,2.000000,-97957.294882,0.000000,3.000000,42419.810496,0.000000,4.000000,-2223.822151,0.000000,5.000000,111.253440,0.000000,6.000000,-591.354120,0.000000,7.000000,17.287557,0.000000,8.000000,-39.772924,0.000000,9.000000,-887.448510,0.000000,10.000000,-19103.732754,0.000000,11.000000,3596.243849,0.000000,12.000000,-138.477699,0.000000,13.000000,56.538385,0.000000,14.000000,-6.705868,0.000000,15.000000,-1.069242,0.000000,16.000000,-186607.947238,0.000000,17.000000,54.908655},
{1.000000,0.000000,1.390616,1.000000,1.000000,39.943153,1.000000,2.000000,-97820.941581,1.000000,3.000000,-42644.649130,1.000000,4.000000,2255.209356,1.000000,5.000000,-104.580089,1.000000,6.000000,595.160410,1.000000,7.000000,-18.043551,1.000000,8.000000,40.323236,1.000000,9.000000,892.356777,1.000000,10.000000,-19071.153561,1.000000,11.000000,-3601.176470,1.000000,12.000000,142.959295,1.000000,13.000000,-55.393502,1.000000,14.000000,6.744088,1.000000,15.000000,1.066547,1.000000,16.000000,-186362.313304,1.000000,17.000000,-39.655903},
{2.000000,0.000000,-0.321012,2.000000,1.000000,0.019130,2.000000,2.000000,-0.040316,2.000000,3.000000,-21.731850,2.000000,4.000000,-548.677335,2.000000,5.000000,-468.350476,2.000000,6.000000,-451.937095,2.000000,7.000000,-72.868155,2.000000,8.000000,-9.395540,2.000000,9.000000,0.427515,2.000000,10.000000,-0.003020,2.000000,11.000000,-3.527660,2.000000,12.000000,-64.321808,2.000000,13.000000,-65.701801,2.000000,14.000000,-13.100294,2.000000,15.000000,-1.611704,2.000000,16.000000,-0.098615,2.000000,17.000000,-1299.009124},
{3.000000,0.000000,-0.323779,3.000000,1.000000,0.006324,3.000000,2.000000,-0.015845,3.000000,3.000000,-6.518958,3.000000,4.000000,-553.719846,3.000000,5.000000,469.178604,3.000000,6.000000,-72.801156,3.000000,7.000000,-455.193901,3.000000,8.000000,-9.472686,3.000000,9.000000,0.144623,3.000000,10.000000,-0.001896,3.000000,11.000000,-0.321562,3.000000,12.000000,-64.739571,3.000000,13.000000,65.722430,3.000000,14.000000,-1.597260,3.000000,15.000000,-13.151226,3.000000,16.000000,-0.022637,3.000000,17.000000,1303.824425}};

float32_t c_rc_LQR2_equilibrium_control_f32[4]={9857.54, 9837.48, 0, 0};

float32_t c_rc_LQR2_state_vector_f32[16]={0};
float32_t c_rc_LQR2_state_vector_reference_f32[16]={0};
float32_t c_rc_LQR2_error_state_vector_f32[16]={0};

float32_t c_rc_LQR2_state_vector_I_f32[18]={0};
float32_t c_rc_LQR2_state_vector_reference_I_f32[18]={0};
float32_t c_rc_LQR2_error_state_vector_I_f32[18]={0};

float32_t c_rc_LQR2_control_output_f32[4]={0};

arm_matrix_instance_f32 c_rc_LQR2_equilibrium_control;
arm_matrix_instance_f32 c_rc_LQR2_Ke;

//Vector integration of error(Trapezoidal method)
float32_t c_rc_LQR2_deltaxsi[2]={0};
float32_t c_rc_LQR2_deltaxsiant[2]={0};
float32_t c_rc_LQR2_xsi[2]={0};
float32_t c_rc_LQR2_xsiant[2]={0};

/* Private function prototypes -----------------------------------------------*/
arm_matrix_instance_f32 c_rc_LQR2_errorStateVector_I(pv_type_datapr_attitude attitude,
														pv_type_datapr_attitude attitude_reference,
														pv_type_datapr_position position,
														pv_type_datapr_position position_reference,
														pv_type_datapr_servos servo,
														pv_type_datapr_servos servo_reference,
														float sample_time);
arm_matrix_instance_f32 c_rc_LQR2_errorStateVector(pv_type_datapr_attitude attitude,
													  pv_type_datapr_attitude attitude_reference,
													  pv_type_datapr_position position,
													  pv_type_datapr_position position_reference,
													  pv_type_datapr_servos servo,
													  pv_type_datapr_servos servo_reference);
/* Private functions ---------------------------------------------------------*/

float32_t c_rc_LQR_saturation(float32_t value, float32_t lower_limit, float32_t upper_limit){
	if (value <= lower_limit)
		return lower_limit;
	else if (value >= upper_limit)
		return upper_limit;
	else
		return value;
}


arm_matrix_instance_f32 c_rc_LQR2_errorStateVector_I(pv_type_datapr_attitude attitude,
													 pv_type_datapr_attitude attitude_reference,
													 pv_type_datapr_position position,
													 pv_type_datapr_position position_reference,
													 pv_type_datapr_servos servo,
													 pv_type_datapr_servos servo_reference,
													 float sample_time){

	arm_matrix_instance_f32 c_rc_LQR2_error_state_vector, c_rc_LQR2_state_vector,  c_rc_LQR2_reference_state_vector;
	//Integradores
	//Vector integration of error(Trapezoidal method)
	c_rc_LQR2_deltaxsi[0]=position.z-position_reference.z;
	c_rc_LQR2_xsi[0]=c_rc_LQR2_xsiant[0]+(((float32_t)sample_time)*(c_rc_LQR2_deltaxsi[0]+c_rc_LQR2_deltaxsiant[0])*0.5);
	c_rc_LQR2_deltaxsi[1]=attitude.yaw-attitude_reference.yaw;
	c_rc_LQR2_xsi[1]=c_rc_LQR2_xsiant[1]+(((float32_t)sample_time)*(c_rc_LQR2_deltaxsi[1]+c_rc_LQR2_deltaxsiant[1])*0.5);
	// anti_windup
	c_rc_LQR2_xsi[0]=c_rc_LQR_saturation(c_rc_LQR2_xsi[0],-0.2,0.2);
	c_rc_LQR2_xsi[1]=c_rc_LQR_saturation(c_rc_LQR2_xsi[1],-0.2,0.2);

	//State Vector
	c_rc_LQR2_state_vector_I_f32[STATE_X]=position.x;
	c_rc_LQR2_state_vector_I_f32[STATE_Y]=position.y;
	c_rc_LQR2_state_vector_I_f32[STATE_Z]=position.z;
	c_rc_LQR2_state_vector_I_f32[STATE_ROLL]=attitude.roll;
	c_rc_LQR2_state_vector_I_f32[STATE_PITCH]=attitude.pitch;
	c_rc_LQR2_state_vector_I_f32[STATE_YAW]=attitude.yaw;
	c_rc_LQR2_state_vector_I_f32[STATE_ALPHA_R]=servo.alphar;
	c_rc_LQR2_state_vector_I_f32[STATE_ALPHA_L]=servo.alphal;
	c_rc_LQR2_state_vector_I_f32[STATE_DX]=position.dotX;
	c_rc_LQR2_state_vector_I_f32[STATE_DY]=position.dotY;
	c_rc_LQR2_state_vector_I_f32[STATE_DZ]=position.dotZ;
	c_rc_LQR2_state_vector_I_f32[STATE_DROLL]=attitude.dotRoll;
	c_rc_LQR2_state_vector_I_f32[STATE_DPITCH]=attitude.dotPitch;
	c_rc_LQR2_state_vector_I_f32[STATE_DYAW]=attitude.dotYaw;
	c_rc_LQR2_state_vector_I_f32[STATE_DALPHA_R]=servo.dotAlphar;
	c_rc_LQR2_state_vector_I_f32[STATE_DALPHA_L]=servo.dotAlphal;
	c_rc_LQR2_state_vector_I_f32[STATE_INT_Z]=c_rc_LQR2_xsi[0];
	c_rc_LQR2_state_vector_I_f32[STATE_INT_YAW]=c_rc_LQR2_xsi[1];


	//Updates the height equilibrium point according to the reference
	c_rc_LQR2_state_vector_reference_I_f32[STATE_X]=position_reference.x;
	c_rc_LQR2_state_vector_reference_I_f32[STATE_Y]=position_reference.y;
	c_rc_LQR2_state_vector_reference_I_f32[STATE_Z]=position_reference.z;
	c_rc_LQR2_state_vector_reference_I_f32[STATE_ROLL]=attitude_reference.roll;
	c_rc_LQR2_state_vector_reference_I_f32[STATE_PITCH]=attitude_reference.pitch;
	c_rc_LQR2_state_vector_reference_I_f32[STATE_YAW]=attitude_reference.yaw;
	c_rc_LQR2_state_vector_reference_I_f32[STATE_ALPHA_R]=servo_reference.alphar;
	c_rc_LQR2_state_vector_reference_I_f32[STATE_ALPHA_L]=servo_reference.alphal;
	c_rc_LQR2_state_vector_reference_I_f32[STATE_DX]=position_reference.dotX;
	c_rc_LQR2_state_vector_reference_I_f32[STATE_DY]=position_reference.dotY;
	c_rc_LQR2_state_vector_reference_I_f32[STATE_DZ]=position_reference.dotZ;
	c_rc_LQR2_state_vector_reference_I_f32[STATE_DROLL]=attitude_reference.dotRoll;
	c_rc_LQR2_state_vector_reference_I_f32[STATE_DPITCH]=attitude_reference.dotPitch;
	c_rc_LQR2_state_vector_reference_I_f32[STATE_DYAW]=attitude_reference.dotYaw;
	c_rc_LQR2_state_vector_reference_I_f32[STATE_DALPHA_R]=servo_reference.dotAlphar;
	c_rc_LQR2_state_vector_reference_I_f32[STATE_DALPHA_L]=servo_reference.dotAlphal;
	c_rc_LQR2_state_vector_reference_I_f32[STATE_INT_Z]=0;
	c_rc_LQR2_state_vector_reference_I_f32[STATE_INT_YAW]=0;

	//Initializes the matrices
	arm_mat_init_f32(&c_rc_LQR2_state_vector, 18, 1, (float32_t *)c_rc_LQR2_state_vector_f32);
	arm_mat_init_f32(&c_rc_LQR2_reference_state_vector, 18, 1, (float32_t *)c_rc_LQR2_state_vector_reference_f32);
	arm_mat_init_f32(&c_rc_LQR2_error_state_vector, 18, 1, (float32_t *)c_rc_LQR2_error_state_vector_f32);
	//e(t)=x(k)- xr(k)
	arm_mat_sub_f32(&c_rc_LQR2_state_vector, &c_rc_LQR2_reference_state_vector, &c_rc_LQR2_error_state_vector);

	return c_rc_LQR2_error_state_vector;
}

arm_matrix_instance_f32 c_rc_LQR2_errorStateVector(pv_type_datapr_attitude attitude,
												   pv_type_datapr_attitude attitude_reference,
												   pv_type_datapr_position position,
												   pv_type_datapr_position position_reference,
												   pv_type_datapr_servos servo,
												   pv_type_datapr_servos servo_reference){

	arm_matrix_instance_f32 c_rc_LQR2_error_state_vector, c_rc_LQR2_state_vector,  c_rc_LQR2_reference_state_vector;

		//State Vector
		c_rc_LQR2_state_vector_f32[STATE_X]=position.x;
		c_rc_LQR2_state_vector_f32[STATE_Y]=position.y;
		c_rc_LQR2_state_vector_f32[STATE_Z]=position.z;
		c_rc_LQR2_state_vector_f32[STATE_ROLL]=attitude.roll;
		c_rc_LQR2_state_vector_f32[STATE_PITCH]=attitude.pitch;
		c_rc_LQR2_state_vector_f32[STATE_YAW]=attitude.yaw;
		c_rc_LQR2_state_vector_f32[STATE_ALPHA_R]=servo.alphar;
		c_rc_LQR2_state_vector_f32[STATE_ALPHA_L]=servo.alphal;
		c_rc_LQR2_state_vector_f32[STATE_DX]=position.dotX;
		c_rc_LQR2_state_vector_f32[STATE_DY]=position.dotY;
		c_rc_LQR2_state_vector_f32[STATE_DZ]=position.dotZ;
		c_rc_LQR2_state_vector_f32[STATE_DROLL]=attitude.dotRoll;
		c_rc_LQR2_state_vector_f32[STATE_DPITCH]=attitude.dotPitch;
		c_rc_LQR2_state_vector_f32[STATE_DYAW]=attitude.dotYaw;
		c_rc_LQR2_state_vector_f32[STATE_DALPHA_R]=servo.dotAlphar;
		c_rc_LQR2_state_vector_f32[STATE_DALPHA_L]=servo.dotAlphal;

		//Updates the height equilibrium point according to the reference
		c_rc_LQR2_state_vector_reference_f32[STATE_X]=position_reference.x;
		c_rc_LQR2_state_vector_reference_f32[STATE_Y]=position_reference.y;
		c_rc_LQR2_state_vector_reference_f32[STATE_Z]=position_reference.z;
		c_rc_LQR2_state_vector_reference_f32[STATE_ROLL]=attitude_reference.roll;
		c_rc_LQR2_state_vector_reference_f32[STATE_PITCH]=attitude_reference.pitch;
		c_rc_LQR2_state_vector_reference_f32[STATE_YAW]=attitude_reference.yaw;
		c_rc_LQR2_state_vector_reference_f32[STATE_ALPHA_R]=servo_reference.alphar;
		c_rc_LQR2_state_vector_reference_f32[STATE_ALPHA_L]=servo_reference.alphal;
		c_rc_LQR2_state_vector_reference_f32[STATE_DX]=position_reference.dotX;
		c_rc_LQR2_state_vector_reference_f32[STATE_DY]=position_reference.dotY;
		c_rc_LQR2_state_vector_reference_f32[STATE_DZ]=position_reference.dotZ;
		c_rc_LQR2_state_vector_reference_f32[STATE_DROLL]=attitude_reference.dotRoll;
		c_rc_LQR2_state_vector_reference_f32[STATE_DPITCH]=attitude_reference.dotPitch;
		c_rc_LQR2_state_vector_reference_f32[STATE_DYAW]=attitude_reference.dotYaw;
		c_rc_LQR2_state_vector_reference_f32[STATE_DALPHA_R]=servo_reference.dotAlphar;
		c_rc_LQR2_state_vector_reference_f32[STATE_DALPHA_L]=servo_reference.dotAlphal;

		//Initializes the matrices
		arm_mat_init_f32(&c_rc_LQR2_state_vector, 16, 1, (float32_t *)c_rc_LQR2_state_vector_f32);
		arm_mat_init_f32(&c_rc_LQR2_reference_state_vector, 16, 1, (float32_t *)c_rc_LQR2_state_vector_reference_f32);
		arm_mat_init_f32(&c_rc_LQR2_error_state_vector, 16, 1, (float32_t *)error_state_vector_f32);
		//e(t)=x(k)- xr(k)
		arm_mat_sub_f32(&c_rc_LQR2_state_vector, &c_rc_LQR2_reference_state_vector, &c_rc_LQR2_error_state_vector);

	return c_rc_LQR2_error_state_vector;
}

/* Exported functions definitions --------------------------------------------*/

/** \brief Inicilização do controle de estabilidade.
 *
 * O controlador utiliza a API de DSP da CMSIS, e portanto se baseia fortemente no uso do
 * tipo arm_matrix_instance_f32. Esta \b struct contêm os valores de número de linhas e
 * colunas de matriz, além de um ponteiro para seus elementos (na forma de array).
 * Estes arrays são prealocados globalmente (ver código fonte), para evitar overhead
 * de alocação dinâmica em cada chamada e para evitar que, a cada alocação em uma função, a memória para
 * a qual o ponteiro aponta saia de escopo e seja deletada. Uma vez que as funções são privadas e chamadas
 * em ordem determinística, mutexes não são implementadas (por simplicidade apenas)
 */
void c_rc_LQR2_control_init() {

	// Inicializa as matrizes estaticas
	arm_mat_init_f32(&c_rc_LQR2_equilibrium_control, 4, 1, (float32_t *)c_rc_LQR2_equilibrium_control_f32);
	arm_mat_init_f32(&c_rc_LQR2_Ke, 4, 8, (float32_t *)c_rc_LQR2_Ke_f32);

}



/* \brief LQR Controller.
 *
 * Implemented based on the article "Back-stepping Control Strategy for Stabilization of a Tilt-rotor UAV" by Chowdhury, A. B., with some modifications.
 * It implements an estabilization controller and an altitude controller. It is meant to be used with the radio controller.
 * The struct pv_msg_io_attitude includes the angular velocity.
 */

pv_type_actuation c_rc_LQR2_controller(pv_type_datapr_attitude attitude,
				  pv_type_datapr_attitude attitude_reference,
				  pv_type_datapr_position position,
				  pv_type_datapr_position position_reference,
				  pv_type_datapr_servos servo,
				  pv_type_datapr_servos servo_reference,
				  float sample_time){

	pv_type_actuation actuation_signals;

	arm_matrix_instance_f32 c_rc_LQR2_error_state_vector, c_rc_LQR2_control_output, c_rc_LQR2_i_control_output;

	//Initialize result matrices
	arm_mat_init_f32(&c_rc_LQR2_error_state_vector, 4, 1, (float32_t *)c_rc_LQR2_error_state_vector_f32);
	arm_mat_init_f32(&c_rc_LQR2_control_output, 4, 1, (float32_t *)c_rc_LQR2_control_output_f32);
	arm_mat_init_f32(&c_rc_LQR2_i_control_output, 4, 1, (float32_t *)c_rc_LQR2_i_control_output_f32);

	//e(k)=xs_a(k)-xr_a(k)
	c_rc_LQR2_error_state_vector = c_rc_LQR_AH_errorStateVector_I(attitude, attitude_reference, position, position_reference,servo, servo_reference,sample_time);
	//u=Ke*e(t)
	arm_mat_mult_f32(&c_rc_LQR2_Ke, &c_rc_LQR2_error_state_vector, &c_rc_LQR2_i_control_output);

	arm_mat_add_f32(&c_rc_LQR2_equilibrium_control, &c_rc_LQR2_control_output, &c_rc_LQR2_control_output);
	//The result must be in a struct pv_msg_io_actuation
	actuation_signals.escRightSpeed= (float)c_rc_LQR2_control_output.pData[0]/1000;
	actuation_signals.escLeftSpeed=	 (float)c_rc_LQR2_control_output.pData[1]/1000;
	actuation_signals.servoRight=	 (float)c_rc_LQR2_control_output.pData[2]/1000;
	actuation_signals.servoLeft=	 (float)c_rc_LQR2_control_output.pData[3]/1000;

	return actuation_signals;
}

/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */

