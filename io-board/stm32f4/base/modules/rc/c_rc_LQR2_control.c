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


//float32_t c_rc_LQR2_Ki_f32[4][18]={{-0.032676,-1.790518,-8081725.334234,696830.399469,-4146.625541,-34534.421452,-761.899198,541.913196,-7.452760,-686.289624,-153199.442447,16274.387485,-98.848807,-862.814380,-1.435292,-1.321221,-136806541.283952,-129699.627045},
//		{0.032182,1.788570,-8065326.258753,-696072.172277,4082.905223,34506.257776,721.716586,-574.306637,7.342892,685.565463,-152888.165516,-16233.556962,92.564448,859.573036,0.967616,0.912050,-136529203.513908,129657.795451},
//		{-0.021746,0.000723,-0.002675,-287.224561,-2815.987131,-7520.001218,-1642.220577,24.079836,-4.867426,0.276976,-0.000489,-26.105755,-272.836678,-585.903174,-25.837197,-3.080271,0.075500,-22259.949641},
//		{-0.021764,-0.000376,0.009507,142.951442,-2821.601545,7506.655927,23.812396,-1644.831535,-4.871445,-0.143968,0.000400,8.140278,-272.668557,583.909121,-3.091596,-25.861362,0.139020,22231.427994}};

//float32_t c_rc_LQR2_Ki_f32[4][18]={{-1.256674,-2.201734,-41543.712334,84504.582366,-62463.079009,-9183.876367,6025.607383,7287.203688,-292.742759,-398.216230,-14464.697215,6452.282831,-3255.095210,-1011.299071,81.582718,85.592247,-0.059668,-0.015833},
//		{-2.053109,2.323370,-43287.445605,-88845.115329,-100763.815246,-7554.521798,-7434.032576,-10438.861956,-479.106121,421.385480,-15637.386848,-6668.707067,-8180.369974,-343.701662,-103.451292,-119.211192,-0.061992,-0.013083},
//		{-0.118757,0.004957,388.996693,-172.201755,-5860.918425,-891.903819,-1317.420273,-661.078821,-27.745312,0.930606,-33.779173,-16.077561,-419.636227,-126.622290,-24.767335,-8.999617,0.000579,-0.001539},
//		{-0.078068,0.000517,-305.513156,-6.920118,-3848.701244,1118.802166,-469.582974,-1201.241388,-18.021566,0.112057,-122.234845,-9.971906,-327.671938,168.055242,-6.766891,-22.799016,-0.000428,0.001925}};

float32_t c_rc_LQR2_Ke_f32[4][16]={{4.595096,-3.648554,-225560.517585,5641.409283,8800.976615,6343.722123,23349.015200,8602.247176,130.528632,-81.424854,-28495.811755,1854.181491,2498.815939,2057.100323,641.604881,221.964903},
		{0.638235,91.018965,-20150.549552,-92861.764565,452.987444,-1861.298158,-18037.837251,-4013.755831,13.883092,1862.381674,-7271.662356,-14920.634862,-555.917886,-1034.364919,-738.788271,-135.083696},
		{-4.683997,0.155372,-991.397712,-219.059786,-8831.907188,-10878.286140,-31117.663995,-1552.748123,-132.589964,3.363241,-89.687947,-110.015146,-2267.028318,-3373.210454,-1415.219123,-107.309469},
		{-9.346997,0.090078,43.987653,-125.853346,-17673.347803,7922.028338,-12656.653293,-35739.171770,-264.479845,2.274596,9.556440,-49.620342,-4322.616717,2444.668960,-261.419647,-1453.375605}};
float32_t c_rc_LQR2_equilibrium_control_f32[4]={9857.54, 9837.48, 0, 0};

float32_t c_rc_LQR2_state_vector_f32[16]={0};
float32_t c_rc_LQR2_state_vector_reference_f32[16]={0};
float32_t c_rc_LQR2_error_state_vector_f32[16]={0};

float32_t c_rc_LQR2_state_vector_I_f32[18]={0};
float32_t c_rc_LQR2_state_vector_reference_I_f32[18]={0};
float32_t c_rc_LQR2_error_state_vector_I_f32[18]={0};

float32_t c_rc_LQR2_control_output_f32[4]={0};
float32_t c_rc_LQR2_i_control_output_f32[4]={0};

arm_matrix_instance_f32 c_rc_LQR2_equilibrium_control;
arm_matrix_instance_f32 c_rc_LQR2_Ke;
arm_matrix_instance_f32 c_rc_LQR2_Ki;

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
														float sample_time, bool stop);
arm_matrix_instance_f32 c_rc_LQR2_errorStateVector(pv_type_datapr_attitude attitude,
													  pv_type_datapr_attitude attitude_reference,
													  pv_type_datapr_position position,
													  pv_type_datapr_position position_reference,
													  pv_type_datapr_servos servo,
													  pv_type_datapr_servos servo_reference);
/* Private functions ---------------------------------------------------------*/

float32_t c_rc_LQR2_saturation(float32_t value, float32_t lower_limit, float32_t upper_limit){
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
													 float sample_time, bool stop){

	arm_matrix_instance_f32 c_rc_LQR2_error_state_vector, c_rc_LQR2_state_vector,  c_rc_LQR2_reference_state_vector;
	//Integradores
	//Vector integration of error(Trapezoidal method)
	if (stop){
		c_rc_LQR2_deltaxsi[0]=0;
		c_rc_LQR2_deltaxsiant[0]=0;
		c_rc_LQR2_xsi[0]=0;
		c_rc_LQR2_xsiant[0]=0;
		c_rc_LQR2_xsi[1]=0;
		c_rc_LQR2_xsiant[1]=0;
		c_rc_LQR2_deltaxsi[1]=0;
		c_rc_LQR2_deltaxsiant[1]=0;
	}else{
		c_rc_LQR2_deltaxsi[0]=position.z-position_reference.z;
		c_rc_LQR2_xsi[0]=c_rc_LQR2_xsiant[0]+(((float32_t)sample_time)*(c_rc_LQR2_deltaxsi[0]+c_rc_LQR2_deltaxsiant[0])*0.5);
		c_rc_LQR2_deltaxsi[1]=attitude.yaw-attitude_reference.yaw;
		c_rc_LQR2_xsi[1]=c_rc_LQR2_xsiant[1]+(((float32_t)sample_time)*(c_rc_LQR2_deltaxsi[1]+c_rc_LQR2_deltaxsiant[1])*0.5);
	}
	// anti_windup
	c_rc_LQR2_xsi[0]=c_rc_LQR2_saturation(c_rc_LQR2_xsi[0],-0.2,0.2);
	c_rc_LQR2_xsi[1]=c_rc_LQR2_saturation(c_rc_LQR2_xsi[1],-0.2,0.2);

	//update
	c_rc_LQR2_deltaxsiant[0]=c_rc_LQR2_deltaxsi[0]=0;
	c_rc_LQR2_deltaxsiant[1]=c_rc_LQR2_deltaxsi[1]=0;
	c_rc_LQR2_xsiant[1]=c_rc_LQR2_xsi[0];
	c_rc_LQR2_xsiant[1]=c_rc_LQR2_xsi[1];

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
	arm_mat_init_f32(&c_rc_LQR2_state_vector, 18, 1, (float32_t *)c_rc_LQR2_state_vector_I_f32);
	arm_mat_init_f32(&c_rc_LQR2_reference_state_vector, 18, 1, (float32_t *)c_rc_LQR2_state_vector_reference_I_f32);
	arm_mat_init_f32(&c_rc_LQR2_error_state_vector, 18, 1, (float32_t *)c_rc_LQR2_error_state_vector_I_f32);
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
		arm_mat_init_f32(&c_rc_LQR2_error_state_vector, 16, 1, (float32_t *)c_rc_LQR2_error_state_vector_f32);
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
	#ifdef LQR_INTEGRATOR
		arm_mat_init_f32(&c_rc_LQR2_Ki, 4, 18, (float32_t *)c_rc_LQR2_Ki_f32);
	#else
		arm_mat_init_f32(&c_rc_LQR2_Ke, 4, 16, (float32_t *)c_rc_LQR2_Ke_f32);
	#endif
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
				  float sample_time, bool stop){

	pv_type_actuation actuation_signals;
#ifdef LQR_INTEGRATOR
	arm_matrix_instance_f32 c_rc_LQR2_error_state_vector, c_rc_LQR2_control_output, c_rc_LQR2_i_control_output;

	//Initialize result matrices
	arm_mat_init_f32(&c_rc_LQR2_control_output, 4, 1, (float32_t *)c_rc_LQR2_control_output_f32);
	arm_mat_init_f32(&c_rc_LQR2_i_control_output, 4, 1, (float32_t *)c_rc_LQR2_i_control_output_f32);

	//e(k)=xs_a(k)-xr_a(k)
	c_rc_LQR2_error_state_vector = c_rc_LQR2_errorStateVector_I(attitude, attitude_reference, position, position_reference,servo, servo_reference,sample_time, stop);
	//u=Ke*e(t)
	arm_mat_mult_f32(&c_rc_LQR2_Ki, &c_rc_LQR2_error_state_vector, &c_rc_LQR2_i_control_output);

	arm_mat_add_f32(&c_rc_LQR2_equilibrium_control, &c_rc_LQR2_i_control_output, &c_rc_LQR2_control_output);
#else
	arm_matrix_instance_f32 c_rc_LQR2_error_state_vector, c_rc_LQR2_control_output, c_rc_LQR2_i_control_output;

	//Initialize result matrices
	arm_mat_init_f32(&c_rc_LQR2_control_output, 4, 1, (float32_t *)c_rc_LQR2_control_output_f32);
	arm_mat_init_f32(&c_rc_LQR2_i_control_output, 4, 1, (float32_t *)c_rc_LQR2_i_control_output_f32);

	//e(k)=xs_a(k)-xr_a(k)
	c_rc_LQR2_error_state_vector = c_rc_LQR2_errorStateVector(attitude, attitude_reference, position, position_reference,servo, servo_reference);
	//u=Ke*e(t)
	arm_mat_mult_f32(&c_rc_LQR2_Ke, &c_rc_LQR2_error_state_vector, &c_rc_LQR2_i_control_output);

	arm_mat_add_f32(&c_rc_LQR2_equilibrium_control, &c_rc_LQR2_i_control_output, &c_rc_LQR2_control_output);
#endif

	//The result must be in a struct pv_msg_io_actuation
	actuation_signals.escRightNewtons=c_rc_LQR2_saturation((float)c_rc_LQR2_control_output.pData[0]/1000,0,15);
	actuation_signals.escLeftNewtons=c_rc_LQR2_saturation((float)c_rc_LQR2_control_output.pData[1]/1000,0,15);
	actuation_signals.servoRight=c_rc_LQR2_saturation((float)c_rc_LQR2_control_output.pData[2]/1000,-2,2);
	actuation_signals.servoLeft=c_rc_LQR2_saturation((float)c_rc_LQR2_control_output.pData[3]/1000,-2,2);
	actuation_signals.escRightSpeed=0;
	actuation_signals.escLeftSpeed=0;
	return actuation_signals;
}

/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */

