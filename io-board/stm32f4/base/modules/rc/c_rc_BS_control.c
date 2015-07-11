/**
  ******************************************************************************
  * @file    modules/rc/c_rc_receiver.c
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    30-November-2013
  * @brief   Controle de estabilizacao para vôo com usando controle remoto manual.
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "c_rc_BS_control.h"

/** @addtogroup Module_RC
  * @{
  */

/** @addtogroup Module_RC_Component_Control
  * \brief Controle de estabilização para o modo de operação RC.
  *
  * O controlador pode ser testado fazendo-se, por exemplo:
  * \code
  * GPIOPin test = c_common_gpio_init(GPIOB, GPIO_Pin_11, GPIO_Mode_OUT);
  *
  * pv_msg_io_actuation   actuation = {0,0.0f,0.0f,0.0f,0.0f};
  * pv_msg_datapr_attitude attitude = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
  * pv_msg_datapr_attitude attitude_reference = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
  * pv_msg_datapr_position position = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
  * pv_msg_datapr_position position_reference = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
  *
  * attitude_reference.roll = 1.0f;
  *
  * while(1) {
  *   	actuation = RC_controller(attitude,
  * 					attitude_reference,
  * 					position,
  * 					position_reference);
  * 	    c_common_gpio_toggle(test);
  * }
  * \endcode
  *
  * A frequência de cálculo pode ser vista então se olhando o pino comutado no osciloscópio.
  * Para o exemplo acima, gira em torno de 80us.
  * @{
  */

		//---------------------------------------------------------------------------------------------

/* Exported functions definitions --------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
float32_t gamma_f32[3]={0};
float32_t tau_f32[3]={0};
float32_t attitudeVector_f32[3]={0};
float32_t Mq_f32[3][3]={{0}};
float32_t Cqq_f32[3][3]={{0}};

// Variáveis criadas para guardar o erro e a integral do erro
// São globais porque precisamos do erro anterior e integracao anterior.
pv_type_stability_error BS_AH_integrated_error={0};
pv_type_stability_error BS_AH_last_error={0};


/* Private function prototypes -----------------------------------------------*/
float32_t c_rc_BS_AH_altitude_controller_step(float altitude, float altitude_reference, float rateOfClimb, float rateOfClimb_reference,
									pv_type_datapr_attitude attitude);
arm_matrix_instance_f32 c_rc_BS_AH_PD_gains_step(pv_type_datapr_attitude attitude, pv_type_datapr_attitude attitude_reference, bool enable_integrators);
arm_matrix_instance_f32 c_rc_BS_AH_torque_calculation_step(pv_type_datapr_attitude attitude, arm_matrix_instance_f32 gamma);
pv_type_actuation c_rc_BS_AH_actuators_signals_step(arm_matrix_instance_f32 tau, float Fzb);
arm_matrix_instance_f32 c_rc_BS_AH_inertia_matrix(pv_type_datapr_attitude attitude);
arm_matrix_instance_f32 c_rc_BS_AH_coriolis_matrix(pv_type_datapr_attitude attitude);
void c_rc_BS_AH_integrate_error(pv_type_stability_error current_error, float sample_time);
float c_rc_BS_AH_Fzb_RC(float signal_RC);

//pv_msg_io_actuation LQR(arm_matrix_instance_f32 state_vector);

/* Private functions ---------------------------------------------------------*/

// rateOfClimb (RoC) = aircraft's vertical speed (velocity in Z-axis)
float c_rc_BS_AH_altitude_controller_step(float altitude, float altitude_reference, float rateOfClimb, float rateOfClimb_reference, pv_type_datapr_attitude attitude)
{
	float Zeta3=0;

	Zeta3 = -KPZ*(altitude - altitude_reference) - KVZ*(rateOfClimb - rateOfClimb_reference) -KIZ*BS_AH_integrated_error.z;

	return (M * G + M * Zeta3) / (cos(attitude.roll) * cos(attitude.pitch));
}

/** \brief Calcula o Fzb de acordo com o sinal do radio controle */
float c_rc_BS_AH_Fzb_RC(float throttle_control){
	return THRUST_FLIGHT_THRESHOLD+THRUST_MAX_MANUAL*throttle_control;
}

///** \brief Integral numérica utilizando o método trapezoidal (Tustin) */
//float integrateTrapezoidal(float last_integration, float current_value, float last_value, float sample_time){
//
//	return last_integration + sample_time * 0.5* (current_value + last_value);
//}

/** \brief Integra o erro, para ser usado no controle.
 * @param current_error erro calculado nesta iteracão
 * @param sample_time tempo entre uma medicão e outra - Periodo de amostragem variável.
 *
 * A resposta é guardada na variável global BS_AH_integrated_error. Foi utilizada uma variável globla pois precisamos guardar o valor da integral do erro
 * da última iteracão.
 * Outra vairavel global foic riada para guardar o valor do erro atual para a proxima iteracao
 */
void c_rc_BS_AH_integrate_error(pv_type_stability_error error, float sample_time){

	#ifdef ENABLE_INT_ROLL
//		BS_AH_integrated_error.roll = c_rc_saturation( c_rc_integrate_trapezoidal(BS_AH_integrated_error.roll, error.roll, BS_AH_last_error.roll, sample_time), INT_ROLL_LOWER_ER_LIMIT, INT_ROLL_UPPER_ER_LIMIT);
	BS_AH_integrated_error.roll = c_rc_integrate_trapezoidal(BS_AH_integrated_error.roll, error.roll, BS_AH_last_error.roll, sample_time);
		BS_AH_last_error.roll  = error.roll;
	#endif
	#ifdef ENABLE_INT_PITCH
//		BS_AH_integrated_error.pitch = c_rc_saturation( c_rc_integrate_trapezoidal(BS_AH_integrated_error.pitch, error.pitch, BS_AH_last_error.pitch, sample_time), INT_PITCH_LOWER_ER_LIMIT, INT_PITCH_UPPER_ER_LIMIT);
		BS_AH_integrated_error.pitch = c_rc_integrate_trapezoidal(BS_AH_integrated_error.pitch, error.pitch, BS_AH_last_error.pitch, sample_time);
		BS_AH_last_error.pitch = error.pitch;
	#endif
	#ifdef ENABLE_INT_YAW
		BS_AH_integrated_error.yaw = c_rc_saturation( c_rc_integrate_trapezoidal(BS_AH_integrated_error.yaw, error.yaw, BS_AH_last_error.yaw, sample_time), INT_YAW_LOWER_ER_LIMIT, INT_YAW_UPPER_ER_LIMIT);
		BS_AH_last_error.yaw   = error.yaw;
	#endif
	#ifdef ENABLE_INT_Z
//		BS_AH_integrated_error.z = c_rc_saturation( c_rc_integrate_trapezoidal(BS_AH_integrated_error.z, error.z, BS_AH_last_error.z, sample_time), INT_Z_LOWER_ER_LIMIT, INT_Z_UPPER_ER_LIMIT);
		BS_AH_integrated_error.z = c_rc_integrate_trapezoidal(BS_AH_integrated_error.z, error.z, BS_AH_last_error.z, sample_time);
		BS_AH_last_error.z   = error.z;
	#endif
}

arm_matrix_instance_f32 c_rc_BS_AH_PD_gains_step(pv_type_datapr_attitude attitude, pv_type_datapr_attitude attitude_reference, bool enable_integrators){
	arm_matrix_instance_f32 gamma;
	pv_type_stability_error current_error={0};

	current_error.roll = attitude.roll - attitude_reference.roll;
	current_error.pitch = attitude.pitch - attitude_reference.pitch;
	current_error.yaw = attitude.yaw - attitude_reference.yaw;

	/* Integra o erro, utiliza as variaveis globais:
	 * BS_AH_integrated_error;
	 * BS_AH_last_error
	 * Já guarda o valor do erro para a proxima iteracao
	 */
	if (enable_integrators)
		c_rc_BS_AH_integrate_error(current_error, CONTROL_SAMPLE_TIME);
	else{
		BS_AH_integrated_error.roll=0;
		BS_AH_integrated_error.pitch=0;
		BS_AH_integrated_error.yaw=0;
		BS_AH_integrated_error.z=0;
	}

	//gamma1
	gamma_f32[0] = -KPPHI * (attitude.roll - attitude_reference.roll)
						- KVPHI * (attitude.dotRoll - attitude_reference.dotRoll)
						- KIPHI * BS_AH_integrated_error.roll;
	//gamma2
	gamma_f32[1] = -KPTHETA * (attitude.pitch - attitude_reference.pitch)
						- KVTHETA * (attitude.dotPitch - attitude_reference.dotPitch)
						- KITHETA * BS_AH_integrated_error.pitch;

	//gamma3
	gamma_f32[2] = -KPPSI * (attitude.yaw - attitude_reference.yaw)
						- KVPSI * (attitude.dotYaw - attitude_reference.dotYaw)
						- KIPSI * BS_AH_integrated_error.yaw;

	arm_mat_init_f32(&gamma, 3, 1, (float32_t *)gamma_f32);

	return gamma;
}

arm_matrix_instance_f32 c_rc_BS_AH_torque_calculation_step(pv_type_datapr_attitude attitude, arm_matrix_instance_f32 gamma)
{
	float32_t r1_f32[3]={0}, r2_f32[3]={0}; // used as intermediate variable for matrices operations
	arm_matrix_instance_f32 tau, attitudeVector, r1, r2, cor_att, inr_att;

	arm_mat_init_f32(&r1, 3, 1, (float32_t *)r1_f32);
	arm_mat_init_f32(&r2, 3, 1, (float32_t *)r2_f32);

	attitudeVector_f32[0]= attitude.dotRoll;
	attitudeVector_f32[1]= attitude.dotPitch;
	attitudeVector_f32[2]= attitude.dotYaw;

	arm_mat_init_f32(&tau, 3, 1, (float32_t *)tau_f32);
	arm_mat_init_f32(&attitudeVector, 3, 1, (float32_t *)attitudeVector_f32);

	/*tau = ( inertia_matrix(pv_msg_datapr_attitude attitude) * gamma )
	+ ( coriolis_matrix(pv_msg_datapr_attitude attitude) * dotTheta );*/
	inr_att = c_rc_BS_AH_inertia_matrix(attitude);
	cor_att = c_rc_BS_AH_coriolis_matrix(attitude);

	arm_mat_mult_f32(&inr_att, &gamma, &r1);
	arm_mat_mult_f32(&cor_att, &attitudeVector, &r2);
	arm_mat_add_f32(&r1, &r2, &tau);

	return tau;
	/* tau[0] = tauRoll
	   tau[1] = tauPitch
	   tau[2] = tauYaw */
}

pv_type_actuation c_rc_BS_AH_actuators_signals_step(arm_matrix_instance_f32 tau, float Fzb)
{
	/* tau[0] = tauRoll
	   tau[1] = tauPitch
	   tau[2] = tauYaw */

	pv_type_actuation actuation;
	float32_t * tauData = tau.pData;

	actuation.escRightSpeed = 0.5 * sqrt(pow((tauData[1] / H + tauData[2] / L), 2) + pow((-Fzb + tauData[0] / L), 2));

	actuation.escLeftSpeed = 0.5 * sqrt(pow((-tauData[1] / H + tauData[2] / L), 2) + pow((Fzb + tauData[0] / L), 2));

	actuation.servoRight = atan2((tauData[1] / H + tauData[2] / L) , (Fzb - tauData[0] / L));

	actuation.servoLeft = atan2((tauData[1] / H - tauData[2] / L) , (Fzb + tauData[0] / L));

	// Declares that the servos will use angle control, rather than torque control
	actuation.servoTorqueControlEnable = 0;

	return actuation;
}

// Inertia matrix
arm_matrix_instance_f32 c_rc_BS_AH_inertia_matrix(pv_type_datapr_attitude attitude)
{
	arm_matrix_instance_f32 Mq;

	Mq_f32[0][0] = IXX;
	Mq_f32[0][1] = 0;
	Mq_f32[0][2] = -IXX * sin(attitude.pitch);
	Mq_f32[1][0] = 0;
	//Mq_f32[1][1] = IYY * pow(cos(attitude.roll), 2) + IZZ * pow(sin(attitude.roll), 2);
	Mq_f32[1][1] = IYY * (cos(attitude.roll)*cos(attitude.roll)) + IZZ * (sin(attitude.roll)*sin(attitude.roll));
	Mq_f32[1][2] = (IYY - IZZ) * cos(attitude.roll) * sin(attitude.roll)* cos(attitude.pitch);
	Mq_f32[2][0] = - IXX * sin(attitude.pitch);
	Mq_f32[2][1] = (IYY - IZZ) * cos(attitude.roll) * sin(attitude.roll)* cos(attitude.pitch);
	Mq_f32[2][2] = IXX * pow(sin(attitude.pitch), 2) + IYY * pow(sin(attitude.roll), 2)* pow(cos(attitude.pitch), 2) + IZZ * pow(cos(attitude.roll), 2)* pow(cos(attitude.pitch), 2);

	arm_mat_init_f32(&Mq, 3, 3, (float32_t *)Mq_f32);

	return Mq;
}

/*	Coriolis and Centrifugal Matrix = C(THETA,dotTHETA) - Cqq - obtained by
*  christoffel symbols of the first kind page 40 of "ROBUST CONTROL STRATEGIES
*  FOR A QUADROTOR HELICOPTER - An Underactuated Mechanical System" by Raffo G.
*  */
arm_matrix_instance_f32 c_rc_BS_AH_coriolis_matrix(pv_type_datapr_attitude attitude)
{
	arm_matrix_instance_f32 Cqq;

	Cqq_f32[0][0] = 0;

	Cqq_f32[0][1] = (IYY - IZZ)* (attitude.dotPitch * cos(attitude.roll)* sin(attitude.roll)+ 0.5 * attitude.dotYaw* pow(sin(attitude.roll), 2)
			* cos(attitude.pitch))+ 0.5 * (IZZ - IYY) * attitude.dotYaw* pow(cos(attitude.roll), 2)* cos(attitude.pitch)- 0.5 * IXX * attitude.dotYaw * cos(attitude.pitch);

	Cqq_f32[0][2] = (IZZ - IYY)* (attitude.dotYaw * cos(attitude.roll)* sin(attitude.roll)* pow(cos(attitude.pitch), 2)+ 0.5 * attitude.dotPitch * cos(attitude.pitch)
			* pow(cos(attitude.roll), 2))+ 0.5 * (IYY - IZZ) * attitude.dotPitch* cos(attitude.pitch)* pow(sin(attitude.roll), 2)- 0.5 * IXX * attitude.dotPitch * cos(attitude.pitch);

	Cqq_f32[1][0] = (IZZ - IYY)* (attitude.dotPitch * cos(attitude.roll)* sin(attitude.roll)+ 0.5 * attitude.dotYaw* pow(sin(attitude.roll), 2)
			* cos(attitude.pitch))+ 0.5 * (IYY - IZZ)* (attitude.dotYaw * pow(cos(attitude.roll), 2)* cos(attitude.pitch))+ 0.5 * IXX * attitude.dotYaw * cos(attitude.pitch);

	Cqq_f32[1][1] = (IZZ - IYY) * attitude.dotRoll * cos(attitude.roll)* sin(attitude.roll);

	Cqq_f32[1][2] = -IXX * attitude.dotYaw * sin(attitude.pitch)* cos(attitude.pitch)+ IYY * attitude.dotYaw * pow(sin(attitude.roll), 2)
			* cos(attitude.pitch) * sin(attitude.pitch)+ IZZ * attitude.dotYaw * pow(cos(attitude.roll), 2)* sin(attitude.pitch) * cos(attitude.pitch)
			+ 0.5 * (IYY - IZZ) * attitude.dotRoll* cos(attitude.pitch)* pow(cos(attitude.roll), 2)+ 0.5 * (IZZ - IYY) * attitude.dotRoll
			* cos(attitude.pitch)* pow(sin(attitude.roll), 2)+ 0.5 * IXX * attitude.dotRoll * cos(attitude.pitch);

	Cqq_f32[2][0] = (IYY - IZZ)* (attitude.dotYaw * pow(cos(attitude.pitch), 2)* sin(attitude.roll) * cos(attitude.roll)
			+ 0.5 * attitude.dotPitch * cos(attitude.pitch)* pow(cos(attitude.roll), 2))+ 0.5 * (IZZ - IYY) * attitude.dotPitch
			* cos(attitude.pitch)* pow(sin(attitude.roll), 2)- 0.5 * IXX * attitude.dotPitch * cos(attitude.pitch);

	Cqq_f32[2][1] = (IZZ - IYY)* (attitude.dotPitch * cos(attitude.roll)* sin(attitude.roll) * sin(attitude.pitch)+ 0.5 * attitude.dotRoll
			* pow(sin(attitude.roll), 2)* cos(attitude.pitch))+ 0.5 * (IYY - IZZ) * attitude.dotRoll* pow(cos(attitude.roll), 2)* cos(attitude.pitch)
			- 0.5 * IXX * attitude.dotRoll * cos(attitude.pitch)+ IXX * attitude.dotYaw * sin(attitude.pitch)* cos(attitude.pitch)- IYY * attitude.dotYaw * pow(sin(attitude.roll), 2)
			* sin(attitude.pitch) * cos(attitude.pitch)- IZZ * attitude.dotYaw * pow(cos(attitude.roll), 2)* sin(attitude.pitch) * cos(attitude.pitch);

	Cqq_f32[2][2] = (IYY - IZZ) * attitude.dotRoll * cos(attitude.roll)* sin(attitude.roll) * pow(cos(attitude.pitch), 2)- IYY * attitude.dotPitch * pow(sin(attitude.roll), 2)
			* cos(attitude.pitch) * sin(attitude.pitch)- IZZ * attitude.dotPitch * pow(cos(attitude.roll), 2)* cos(attitude.pitch) * sin(attitude.pitch)
			+ IXX * attitude.dotPitch * cos(attitude.pitch)	* sin(attitude.pitch);

	arm_mat_init_f32(&Cqq, 3, 3, (float32_t *)Cqq_f32);

	return Cqq;
}

/* Exported functions definitions --------------------------------------------*/

/** \brief Inicilização do controle de estabilidade.
 *
 * O controlador utilza a API de DSP da CMSIS, e portanto se baseia fortemente no uso do
 * tipo arm_matrix_instance_f32. Esta \b struct contêm os valores de número de linhas e
 * colunas de matriz, além de um ponteiro para seus elementos (na forma de array).
 * Estes arrays são prealocados globalmente (ver código fonte), para evitar overhead
 * de alocação dinâmica em cada chamada e para evitar que, a cada alocação em uma função, a memória para
 * a qual o ponteiro aponta saia de escopo e seja deletada. Uma vez que as funções são privadas e chamadas
 * em ordem determinística, mutexes não são implementadas (por simplicidade apenas)
 */
void c_rc_BS_control_init() {

}

/* \brief Controlador.
 *
 * Implemented based on the article "Back-stepping Control Strategy for Stabilization of a Tilt-rotor UAV" by Chowdhury, A. B., with some modifications.
 * It implements an estabilization controller and an altitude controller. It is meant to be used with the radio controller.
 * The struct pv_msg_io_attitude includes the angular velocity.
 */
pv_type_actuation c_rc_BS_AH_controller(pv_type_datapr_attitude attitude,
		          pv_type_datapr_attitude attitude_reference,
		          pv_type_datapr_position position,
		          pv_type_datapr_position position_reference,
				  float throttle_control,
				  bool manual_height_control,
				  bool enable_integration)
{
	arm_matrix_instance_f32 gamma, tau;
	pv_type_actuation actuation_signals;
	float Fzb;

	if (enable_integration)
		gamma = c_rc_BS_AH_PD_gains_step(attitude, attitude_reference, true);
	else
		gamma = c_rc_BS_AH_PD_gains_step(attitude, attitude_reference, false);

	tau = c_rc_BS_AH_torque_calculation_step(attitude, gamma);

	if (manual_height_control)
		Fzb = c_rc_BS_AH_Fzb_RC(throttle_control);
	else{
		#ifdef ENABLE_RC_HEIGHT_REFERENCE
			Fzb = c_rc_BS_AH_altitude_controller_step(position.z, throttle_control*HEIGHT_REFERENCE_MAX, position.dotZ, position_reference.dotZ, attitude);
		#else
			Fzb = c_rc_BS_AH_altitude_controller_step(position.z, position_reference.refz, position.dotZ, position_reference.refdotZ, attitude);
		#endif
		}
	actuation_signals = c_rc_BS_AH_actuators_signals_step(tau, Fzb);

	return actuation_signals;
}

/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */

