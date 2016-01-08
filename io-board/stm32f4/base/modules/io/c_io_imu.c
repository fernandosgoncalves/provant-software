/**
  ******************************************************************************
  * @file    modules/io/c_io_imu.c
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    11-February-2014
  * @brief   Funções para IMUs (incialmente baseadas no CIs ITG3205 e ADXL345).
  *****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "c_io_imu.h"

/** @addtogroup Module_IO
  * @{
  */

/** @addtogroup Module_IO_Component_IMU
  * \brief Componente para IMU.
  *
  * Este componente é projetado para implementar funções da IMU da aeronave - leitura
  * e pré-processamento. A IMU suportada é a baseada nos CIs ITG3205 e ADXL345, mas
  * outros modelos podem ser incorporados via #define.
  *
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
//#define I2Cx_imu      I2C2 // i2c of imu

#ifdef C_IO_IMU_USE_ITG_ADXL_HMC
  #define GYRO_ADDR   0x68 // The address of ITG3205
  #define ACCL_ADDR   0x53 // The address of ADXL345
  #define MAGN_ADDR   0x1E // The address of HMC5883
  #define GYRO_X_ADDR 0x1D // Start address for x-axis
  #define ACCL_X_ADDR 0x32 // Start address for x-axis
  #define MAGN_X_ADDR 0x03 // Start address for x-axis
#elif defined C_IO_IMU_USE_MPU6050_HMC5883
  #include "c_io_imu_MPU6050.h"

  #define HMC58X3_ADDR 0x1E // 7 bit address of the HMC58X3
  #define HMC_POS_BIAS 1
  #define HMC_NEG_BIAS 2

  // HMC58X3 register map. For details see HMC58X3 datasheet
  #define HMC58X3_R_CONFA 0
  #define HMC58X3_R_CONFB 1
  #define HMC58X3_R_MODE 2
  #define HMC58X3_R_XM 3
  #define HMC58X3_R_XL 4
  #define HMC58X3_R_STATUS 9
  #define HMC58X3_R_IDA 10
  #define HMC58X3_R_IDB 11
  #define HMC58X3_R_IDC 12
#elif defined C_IO_IMU_USE_GY_87
  #include "c_io_imu_MPU6050.h"
  #include "c_io_imu_bmp180.h"
  // HMC58X3 register map. For details see HMC58X3 datasheet
  #define HMC5883L_ADDRESS       0x1E
  #define HMC5883L_REG_CONFIG_A  0x00
  #define HMC5883L_REG_CONFIG_B  0x01
  #define HMC5883L_REG_MODE      0x02
  #define HMC5883L_REG_OUT_X_M   0x03
  #define HMC5883L_REG_OUT_X_L   0x04
  #define HMC5883L_REG_OUT_Z_M   0x05
  #define HMC5883L_REG_OUT_Z_L   0x06
  #define HMC5883L_REG_OUT_Y_M   0x07
  #define HMC5883L_REG_OUT_Y_L   0x08
  #define HMC5883L_REG_STATUS    0x09
  #define HMC5883L_REG_IDENT_A   0x0A
  #define HMC5883L_REG_IDENT_B   0x0B
  #define HMC5883L_REG_IDENT_C   0x0C
  /*Bias*/
  #define HMC_POS_BIAS 1
  #define HMC_NEG_BIAS 2
  /*Sample rate*/
  #define HMC5883L_SAMPLES_8  0b11
  #define HMC5883L_SAMPLES_4  0b10
  #define HMC5883L_SAMPLES_2  0b01
  #define HMC5883L_SAMPLES_1  0b00
  /*Operation range*/
  #define HMC5883L_RANGE_8_1GA   0b111
  #define HMC5883L_RANGE_5_6GA   0b110
  #define HMC5883L_RANGE_4_7GA   0b101
  #define HMC5883L_RANGE_4GA     0b100
  #define HMC5883L_RANGE_2_5GA   0b011
  #define HMC5883L_RANGE_1_9GA   0b010
  #define HMC5883L_RANGE_1_3GA   0b001
  #define HMC5883L_RANGE_0_88GA  0b000
  /*Data Rate*/
  #define HMC5883L_DATARATE_75HZ     0b110
  #define HMC5883L_DATARATE_30HZ     0b101
  #define HMC5883L_DATARATE_15HZ     0b100
  #define HMC5883L_DATARATE_7_5HZ    0b011
  #define HMC5883L_DATARATE_3HZ      0b010
  #define HMC5883L_DATARATE_1_5HZ    0b001
  #define HMC5883L_DATARATE_0_75_HZ  0b000
  /*Operation mode*/
  #define HMC5883L_IDLE       0b10
  #define HMC5883L_SINGLE     0b01
  #define HMC5883L_CONTINOUS  0b00

#else
  #error "Define an IMU type in `c_io_imu.h`! C_IO_USE_ITG_ADXL, or other!"
#endif


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
I2C_TypeDef* I2Cx_imu;
uint8_t imuBuffer[16];
unsigned char ACCL_ID = 0;
unsigned char GYRO_ID = 0;
unsigned char MAGN_ID = 0;

const float mag_ellipsoid_center[3] = {79.8977, -113.117, -136.064};
const float mag_ellipsoid_transform[3][3] = {{0.792428, -0.00418974, 0.00504922}, {-0.00418974, 0.841005, -0.0430735}, {0.00504922, -0.0430735, 0.988147}};

//calibration_matrix[3][3] is the transformation matrix
double mag_calibration_matrix[3][3] = {{M11, M12, M13},{M21, M22, M23},{M31, M32, M33} };
//bias[3] is the bias
double mag_bias[3] = {Bx,By,Bz};
float acc_filtered[3]={0}, acc_filtered_k_minus_1[3]={0}, acc_filtered_k_minus_2[3]={0}, acc_raw_k_minus_1[3]={0}, acc_raw_k_minus_2[3]={0};
float mag_filtered[3]={0}, mag_filtered_k_minus_1[3]={0}, mag_filtered_k_minus_2[3]={0}, mag_raw_k_minus_1[3]={0}, mag_raw_k_minus_2[3]={0};
/* Private function prototypes -----------------------------------------------*/
float abs2(float num);
void c_io_imu_calibrate();
void c_io_imu_EulerMatrix(float * rpy, float * velAngular);
void c_io_imu_Quaternion2Euler(float * q, float * rpy);
void c_io_imu_Quaternion2EulerMadgwick(float * q, float * rpy);

/* Private functions ---------------------------------------------------------*/

/* Exported functions definitions --------------------------------------------*/


/** \brief Inicializa a IMU.
 *
 * Seta sensibilidade do acelerômetro e liga o girscópio.
 */
void c_io_imu_init(I2C_TypeDef* I2Cx)
{
  uint8_t byteBuffer;
  I2Cx_imu=I2Cx;

#ifdef C_IO_IMU_USE_ITG_ADXL_HMC // Inicialização para a IMU selecionada
  /*-----------------------Acelerometer------------------------------*/
  // Get Accelerometer ID
  c_common_i2c_readBytes(I2Cx_imu, ACCL_ADDR, 0x00, 1, &ACCL_ID);
  //  ADXL345 (Accel) pow_CTL - Set to measurement mode
  c_common_i2c_writeByte(I2Cx_imu, ACCL_ADDR, 0x2D, 8);

  // Accelerometer increase G-range
  //0x0B -> (+/- 16G)
  //8    -> (+/- 2G)
  //9    -> (+/- 4G)
  c_common_i2c_writeByte(I2Cx_imu, ACCL_ADDR, 0x31, 9);

  // Accelerometer output data rate(Hz)
  //11 -> 200Hz
  //12 -> 400Hz
  c_common_i2c_writeByte(I2Cx_imu, ACCL_ADDR, 0x2C, 11);

  /*-----------------------Gyroscope------------------------------*/
  // Gyro ID and setup
  c_common_i2c_readBytes(I2Cx_imu, GYRO_ADDR, 0x00, 1, &GYRO_ID);

  // Low Pass Filter
  // 0x1E -> fc=5Hz
  // 0x1B -> fc=42Hz
  // 0x1A -> fc=98Hz
  // 25   -> fc=188Hz
  // 0x03) -> fc=256Hz
  c_common_i2c_writeByte(I2Cx_imu, GYRO_ADDR, 0X16, 0x1E);

  // Output data frequency (must change if the value in register 0x16 changes to fc=256Hz)
  // 4 -> f=200Hz
  // 1 -> f=500Hz
  // 9 -> f=800Hz
  c_common_i2c_writeByte(I2Cx_imu, GYRO_ADDR, 0X15, 1);

  // Clock selection
  c_common_i2c_writeByte(I2Cx_imu, GYRO_ADDR, 0x3E, 3); //clock to PLL gyro z axis

  /*-----------------------Magnetometer------------------------------*/

#endif

#ifdef C_IO_IMU_USE_MPU6050_HMC5883 //Inicialização para a IMU baseada na MPU6050
  // Clear the 'sleep' bit to start the sensor.
  c_common_i2c_writeByte(I2Cx_imu, MPU6050_I2C_ADDRESS, MPU6050_PWR_MGMT_1, 0);

  // Alocar o sub i2c -> desligar o I2C Master da MPU, habilitar I2C bypass
  c_common_i2c_writeBit(I2Cx_imu, MPU6050_I2C_ADDRESS, MPU6050_USER_CTRL, MPU6050_I2C_MST_EN, 0);
  c_common_i2c_writeBit(I2Cx_imu, MPU6050_I2C_ADDRESS, MPU6050_INT_PIN_CFG, MPU6050_I2C_BYPASS_EN, 1);

  /** \todo Implementar e testar o enabling do bus secundário da MPU, para leitura do HMC.*/
  //c_common_i2c_writeByte(0x1E, 0x02, 0x00);
  //uint8_t hmcid[3];
  //c_common_i2c_readBytes(HMC58X3_ADDR, HMC58X3_R_IDA, 3, hmcid);
#endif
#ifdef C_IO_IMU_USE_GY_87
  	  /*-----------------------Acelerometer------------------------------*/
  	  c_io_imu_MPU6050_int(I2Cx_imu);

  	  /*-------Cria as condiçoes necessarias para conectar o HMC5883L----*/
  	  c_io_imu_MPU6050_setI2CMasterModeEnabled(false);
  	  c_io_imu_MPU6050_setI2CBypassEnabled(true) ;
  	  c_io_imu_MPU6050_setSleepEnabled(false);

      /*-----------------------Magnetometer------------------------------*/
      // HMC5883 (Magn) Run in continuous mode
      c_common_i2c_writeByte(I2Cx_imu, HMC5883L_ADDRESS, HMC5883L_REG_MODE,0x00);

      // configure the B register to default value of Sensor Input Field Range: 1.2Ga
      // +/- 1.3Ga
      c_common_i2c_writeByte(I2Cx_imu, HMC5883L_ADDRESS, HMC5883L_REG_CONFIG_B, 0x20);

      //75Hz (maximum)
      c_common_i2c_writeByte(I2Cx_imu, HMC5883L_ADDRESS, HMC5883L_REG_CONFIG_A, 0x18);

      /*-----------------------Barometer------------------------------*/
      c_io_imu_bmp180_int(I2Cx_imu);
#endif
}

/** \brief Calcula o periodo de amostragem
 *
 *
 */
long t1=0;
long c_io_imu_sample_time_us(){
	long t2, sample_time;

	t2 = c_common_utils_micros();
	if (t1==0)
		sample_time = 0;
	else if (t2 < t1)
  		sample_time = t2-t1+UINT32_MAX/168;
  	else
  		sample_time = t2-t1;
	t1=t2;

	return sample_time;
}

/** \brief Obtem as leituras raw do acelerômetro, giro e magnetômetro.
 *
 * O output de cada um dos sensores é escrito nos buffers passados para a função. Os buffers passados
 * devem ter tamanho mínimo 3 (tipo float), e serão escritos sempre com valores dos eixos X, Y e Z
 * de cada sensor lido.
 * Os valores retornados para cada eixo de cada sensor estão em  \f$ g \f$ para o \b acelerômetro,
 * \f$ rad/s \f$ para o \b giroscópio, e \f$ rad \f$ para o \b magnetômetro.
 *
 * @param accRaw Buffer onde serão escritos os dados do acelerômetro.
 * @param gyrRaw Buffer onde serão escritos os dados do giroscópio.
 * @param magRaw Buffer onde serão escritos os dados do magnetômetro.
 */
void c_io_imu_getRaw(float  * accRaw, float * gyrRaw, float * magRaw, long * sample_time_gyro_us){
float mag_tmp[3]={0};
#ifdef C_IO_IMU_USE_ITG_ADXL_HMC
   	// Read x, y, z acceleration, pack the data.
   	uint8_t  buffer[14]={};
   	float accScale =0.00390625f; // 1/256
   	/*All g-ranges, full resolution - typical sensitivity = 256 LSB/g*/

   	c_common_i2c_readBytes(I2Cx_imu, ACCL_ADDR, ACCL_X_ADDR, 6, imuBuffer);

   	// Para transformar em valores no SI -> acc/256 *G m/s^2
   	accRaw[0] = (int16_t)(imuBuffer[0] | (imuBuffer[1] << 8))*accScale;
   	accRaw[1] = (int16_t)(imuBuffer[2] | (imuBuffer[3] << 8))*accScale;
   	accRaw[2] = (int16_t)(imuBuffer[4] | (imuBuffer[5] << 8))*accScale;

   	//2nd order filter with fc=5Hz
   	#ifdef ACC_FILTER_2OD_5HZ
   		float k1_2o_5Hz=-1.778631777824585, k2_2o_5Hz=0.800802646665708, k3_2o_5Hz=0.005542717210281, k4_2o_5Hz=0.011085434420561, k5_2o_5Hz=0.005542717210281;

   		for (int i=0; i<3; i++){
			acc_filtered[i] = -k1_2o_5Hz*acc_filtered_k_minus_1[i] - k2_2o_5Hz*acc_filtered_k_minus_2[i] + k3_2o_5Hz*accRaw[i] + k4_2o_5Hz*acc_raw_k_minus_1[i] + k5_2o_5Hz*acc_raw_k_minus_2[i];
			// Filter memory
			acc_raw_k_minus_2[i] = acc_raw_k_minus_1[i];
			acc_raw_k_minus_1[i] = accRaw[i];
			acc_filtered_k_minus_2[i] = acc_filtered_k_minus_1[i];
			acc_filtered_k_minus_1[i] = acc_filtered[i];
		}
	#endif

	// Read x, y, z from gyro, pack the data
    /** A sensitividade do giroscopio da ITG3200 é dada pela tabela (extraída do datasheet):
     FS_SEL | Full Scale Range | LSB Sensitivity
    --------|------------------|----------------
    0       | Reservado        | Reservado
    1       | Reservado        | Reservado
    2       | Reservado        | Reservado
    3       | 2,000°/seg       | 14.375 LSBs °/S
    ***********************************************/
    
    float gyrScale =14.375f;
    gyrScale = 0.0174532925f/gyrScale;//0.0174532925 = PI/180

    c_common_i2c_readBytes(I2Cx_imu, GYRO_ADDR, GYRO_X_ADDR, 6, imuBuffer);
    sample_time_gyro_us[0] = c_io_imu_sample_time_us();

    gyrRaw[0] =  (int16_t)((imuBuffer[1] | (imuBuffer[0] << 8)))*gyrScale;
    gyrRaw[1] =  (int16_t)((imuBuffer[3] | (imuBuffer[2] << 8)))*gyrScale;
    gyrRaw[2] =  (int16_t)((imuBuffer[5] | (imuBuffer[4] << 8)))*gyrScale;

    // Read x, y, z from magnetometer;

    c_common_i2c_readBytes(I2Cx_imu, MAGN_ADDR, MAGN_X_ADDR, 6, imuBuffer);

    magRaw[0] =  (int16_t)((imuBuffer[1] | (imuBuffer[0] << 8)));// X
    magRaw[1] =  (int16_t)((imuBuffer[5] | (imuBuffer[4] << 8)));// Y
    magRaw[2] =  (int16_t)((imuBuffer[3] | (imuBuffer[2] << 8)));// Z
    
    //2nd order filter with fc=5Hz
    #ifdef MAG_FILTER_2OD_5HZ
    //float k1_2o_5Hz=-1.778631777824585, k2_2o_5Hz=0.800802646665708, k3_2o_5Hz=0.005542717210281, k4_2o_5Hz=0.011085434420561, k5_2o_5Hz=0.005542717210281;
    	for (int i=0; i<3; i++){
    		mag_filtered[i] = -k1_2o_5Hz*mag_filtered_k_minus_1[i] - k2_2o_5Hz*mag_filtered_k_minus_2[i] + k3_2o_5Hz*magRaw[i] + k4_2o_5Hz*mag_raw_k_minus_1[i] + k5_2o_5Hz*mag_raw_k_minus_2[i];
    		// Filter memory
    		mag_raw_k_minus_2[i] = mag_raw_k_minus_1[i];
    		mag_raw_k_minus_1[i] = magRaw[i];
    		mag_filtered_k_minus_2[i] = mag_filtered_k_minus_1[i];
    		mag_filtered_k_minus_1[i] = mag_filtered[i];
    	}
    #endif

	#ifdef CALIBRATE
    	// Compensate accelerometer error
		#ifdef ACC_FILTER_2OD_5HZ
    		accRaw[0] = (acc_filtered[0] - ACCEL_X_OFFSET) * ACCEL_X_SCALE;
    		accRaw[1] = (acc_filtered[1] - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE;
    		accRaw[2] = (acc_filtered[2] - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE;
		#else
    		accRaw[0] = (accRaw[0] - ACCEL_X_OFFSET) * ACCEL_X_SCALE;
    		accRaw[1] = (accRaw[1] - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE;
    		accRaw[2] = (accRaw[2] - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE;
		#endif

    	// Compensate magnetometer error
		#ifdef MAG_FILTER_2OD_5HZ
    		for (int i=0; i<3; i++)
    			magRaw[i] = mag_filtered[i] - mag_bias[i];

    		float result[3] = {0, 0, 0};
    		for (int i=0; i<3; i++)
    			for (int j=0; j<3; ++j)
    				result[i] += mag_calibration_matrix[i][j] * magRaw[j];

    		//calibrated values
    		for (int i=0; i<3; i++) magRaw[i] = result[i];

		#else
    		for (int i=0; i<3; ++i)
    			magRaw[i] = magRaw[i] - mag_bias[i];

    		float result[3] = {0, 0, 0};
    		for (int i=0; i<3; ++i)
    			for (int j=0; j<3; ++j)
    				result[i] += mag_calibration_matrix[i][j] * magRaw[j];

    		//calibrated values
    		for (int i=0; i<3; ++i) magRaw[i] = result[i];

		#endif

    	// Compensate gyroscope error
    	gyrRaw[0] -= OFFSET_GYRO_X;
    	gyrRaw[1] -= OFFSET_GYRO_Y;
    	gyrRaw[2] -= OFFSET_GYRO_Z;
	#endif
    
#endif

#ifdef C_IO_IMU_USE_MPU6050_HMC5883

    uint8_t  buffer[14];
    c_common_i2c_readBytes(I2Cx_imu, MPU6050_I2C_ADDRESS, MPU6050_ACCEL_XOUT_H, 14, buffer);

    /** A sensitividade do acelerômetro da MPU6050 é dada pela tabela (extraída do datasheet):
    AFS_SEL | Full Scale Range | LSB Sensitivity
    --------|------------------|----------------
    0       | ±2g              | 16384 LSB/g
    1       | ±4g              | 8192 LSB/g
    2       | ±8g              | 4096 LSB/g
    3       | ±16g             | 2048 LSB/g
    ***********************************************/
    float accScale = 16384.0f;

    accRaw[0] = -1.0f*(float)((((signed char)buffer[0]) << 8) | ((uint8_t)buffer[1] & 0xFF))/accScale;
    accRaw[1] = -1.0f*(float)((((signed char)buffer[2]) << 8) | ((uint8_t)buffer[3] & 0xFF))/accScale;
    accRaw[2] =       (float)((((signed char)buffer[4]) << 8) | ((uint8_t)buffer[5] & 0xFF))/accScale;

    /** A sensitividade do giroscópio da MPU6050 é dada pela tabela (extraída do datasheet):
    FS_SEL | Full Scale Range | LSB Sensitivity
    -------|------------------|----------------
    0      | ± 250 °/s        | 131 LSB/°/s
    1      | ± 500 °/s        | 65.5 LSB/°/s
    2      | ± 1000 °/s       | 32.8 LSB/°/s
    3      | ± 2000 °/s       | 16.4 LSB/°/s
    ***********************************************/
    float gyrScale = 131.0f;

    gyrRaw[0] = (float)((((signed char)buffer[8])  << 8) | ((uint8_t)buffer[9]  & 0xFF))/gyrScale;
    gyrRaw[1] = (float)((((signed char)buffer[10]) << 8) | ((uint8_t)buffer[11] & 0xFF))/gyrScale;
    gyrRaw[2] = (float)((((signed char)buffer[12]) << 8) | ((uint8_t)buffer[13] & 0xFF))/gyrScale;
#endif
#ifdef C_IO_IMU_USE_GY_87
    // Read x, y, z acceleration, pack the data.
       	int16_t  buffer[6]={};
       	float accScale =0.00012207f; //= 1/8192
       	/*g-ranges, full resolution - 1/sensitivity
       	 * ±2g  16384 LSB/g
         * ±4g  8192 LSB/g   <- setado no c_io_imu_MPU6050_int
         * ±8g  4096 LSB/g
         * ±16g 2048 LSB/g
       	 */

       	c_io_imu_MPU6050_getMotion6(&buffer[0],&buffer[1],&buffer[2],&buffer[3],&buffer[4],&buffer[5]);
       	//c_io_imu_MPU6050_getAcceleration(&buffer[0],&buffer[1],&buffer[2]);

       	// Para transformar em valores no SI -> acc/8192 *G m/s^2
       	accRaw[0] = buffer[0]*accScale;
       	accRaw[1] = buffer[1]*accScale;
       	accRaw[2] = buffer[2]*accScale;

       	//2nd order filter with fc=5Hz
       	#ifdef ACC_FILTER_2OD_5HZ
       		float k1_2o_5Hz=-1.778631777824585, k2_2o_5Hz=0.800802646665708, k3_2o_5Hz=0.005542717210281, k4_2o_5Hz=0.011085434420561, k5_2o_5Hz=0.005542717210281;

       		for (int i=0; i<3; i++){
    			acc_filtered[i] = -k1_2o_5Hz*acc_filtered_k_minus_1[i] - k2_2o_5Hz*acc_filtered_k_minus_2[i] + k3_2o_5Hz*accRaw[i] + k4_2o_5Hz*acc_raw_k_minus_1[i] + k5_2o_5Hz*acc_raw_k_minus_2[i];
    			// Filter memory
    			acc_raw_k_minus_2[i] = acc_raw_k_minus_1[i];
    			acc_raw_k_minus_1[i] = accRaw[i];
    			acc_filtered_k_minus_2[i] = acc_filtered_k_minus_1[i];
    			acc_filtered_k_minus_1[i] = acc_filtered[i];
    		}
    	#endif

    	// Read x, y, z from gyro, pack the data
        /** A sensitividade do giroscopio é dada pela tabela (extraída do datasheet):
         FS_SEL | Full Scale Range | LSB Sensitivity
        --------|------------------|----------------
            0   |    ± 250 °/s     |  131 LSB/°/s
            1   |    ± 500 °/s     |  65.5 LSB/°/s
            2   |    ± 1000 °/s    |  32.8 LSB/°/s
            3   |    ± 2000 °/s    |  16.4 LSB/°/s <- setado no c_io_imu_MPU6050_int
        ***********************************************/

        float gyrScale =16.4f;
        gyrScale = 0.0174532925f/gyrScale;//0.0174532925 = PI/180

        //c_io_imu_MPU6050_getRotation(&buffer[0],&buffer[1],&buffer[2]);
        //c_common_i2c_readBytes(I2Cx_imu, GYRO_ADDR, GYRO_X_ADDR, 6, imuBuffer);
        sample_time_gyro_us[0] = c_io_imu_sample_time_us();

        gyrRaw[0] =  buffer[3]*gyrScale;
        gyrRaw[1] =  buffer[4]*gyrScale;
        gyrRaw[2] =  buffer[5]*gyrScale;

        // Read x, y, z from magnetometer;

        c_common_i2c_readBytes(I2Cx_imu, HMC5883L_ADDRESS, HMC5883L_REG_OUT_X_M , 6, imuBuffer);

        magRaw[0] =  (int16_t)((imuBuffer[1] | (imuBuffer[0] << 8)));// X
        magRaw[1] =  (int16_t)((imuBuffer[5] | (imuBuffer[4] << 8)));// Y
        magRaw[2] =  (int16_t)((imuBuffer[3] | (imuBuffer[2] << 8)));// Z

        //2nd order filter with fc=5Hz
        #ifdef MAG_FILTER_2OD_5HZ
        //float k1_2o_5Hz=-1.778631777824585, k2_2o_5Hz=0.800802646665708, k3_2o_5Hz=0.005542717210281, k4_2o_5Hz=0.011085434420561, k5_2o_5Hz=0.005542717210281;
        	for (int i=0; i<3; i++){
        		mag_filtered[i] = -k1_2o_5Hz*mag_filtered_k_minus_1[i] - k2_2o_5Hz*mag_filtered_k_minus_2[i] + k3_2o_5Hz*magRaw[i] + k4_2o_5Hz*mag_raw_k_minus_1[i] + k5_2o_5Hz*mag_raw_k_minus_2[i];
        		// Filter memory
        		mag_raw_k_minus_2[i] = mag_raw_k_minus_1[i];
        		mag_raw_k_minus_1[i] = magRaw[i];
        		mag_filtered_k_minus_2[i] = mag_filtered_k_minus_1[i];
        		mag_filtered_k_minus_1[i] = mag_filtered[i];
        	}
        #endif

    	#ifdef CALIBRATE
        	// Compensate accelerometer error
    		#ifdef ACC_FILTER_2OD_5HZ
        		accRaw[0] = (acc_filtered[0] - ACCEL_X_OFFSET) * ACCEL_X_SCALE;
        		accRaw[1] = (acc_filtered[1] - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE;
        		accRaw[2] = (acc_filtered[2] - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE;
    		#else
        		accRaw[0] = (accRaw[0] - ACCEL_X_OFFSET) * ACCEL_X_SCALE;
        		accRaw[1] = (accRaw[1] - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE;
        		accRaw[2] = (accRaw[2] - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE;
    		#endif

        	// Compensate magnetometer error
    		#ifdef MAG_FILTER_2OD_5HZ
        		for (int i=0; i<3; i++)
        			magRaw[i] = mag_filtered[i] - mag_bias[i];

        		float result[3] = {0, 0, 0};
        		for (int i=0; i<3; i++)
        			for (int j=0; j<3; ++j)
        				result[i] += mag_calibration_matrix[i][j] * magRaw[j];

        		//calibrated values
        		for (int i=0; i<3; i++) magRaw[i] = result[i];

    		#else
        		for (int i=0; i<3; ++i)
        			magRaw[i] = magRaw[i] - mag_bias[i];

        		float result[3] = {0, 0, 0};
        		for (int i=0; i<3; ++i)
        			for (int j=0; j<3; ++j)
        				result[i] += mag_calibration_matrix[i][j] * magRaw[j];

        		//calibrated values
        		for (int i=0; i<3; ++i) magRaw[i] = result[i];

    		#endif

        	// Compensate gyroscope error
        	gyrRaw[0] -= OFFSET_GYRO_X;
        	gyrRaw[1] -= OFFSET_GYRO_Y;
        	gyrRaw[2] -= OFFSET_GYRO_Z;
    	#endif
#endif

}

void c_io_imu_getBarometerRaw(long *pressure,float *temperature){
#ifdef C_IO_IMU_USE_GY_87
	/*Barometer*/
	*temperature=c_io_imu_bmp180_getTemperature();
	/*Read pressure*/
	*pressure= c_io_imu_bmp180_getPressure();
#endif
}
float c_io_imu_getTemperature(){
#ifdef C_IO_IMU_USE_GY_87
	/*Read temperature*/
	float temperature=c_io_imu_bmp180_getTemperature();
	return temperature;
#endif
}
long c_io_imu_getPressure(){
#ifdef C_IO_IMU_USE_GY_87
	/*Read pressure*/
	long pressure=c_io_imu_bmp180_getPressure();
	return pressure;
#endif
}

float c_io_imu_getAltitude(){
#ifdef C_IO_IMU_USE_GY_87
	/*Read pressure*/
	float pressure=c_io_imu_bmp180_getPressure();
	/*Calculate altitude*/
	float altitude=c_io_imu_bmp180_Altitude(pressure);
	return altitude;
#endif
}
float abs2(float num){
  if (num < 0)
    return -num;
  else
    return num;
}

/** \brief Calibra a IMU considerando o veículo em repouso.
 *
 *
 */
void c_io_imu_calibrate() {

}

/** \brief Calcula a taxa de variacao dos angulos de Euler a partir da velocidade angular representada no frame do corpo
 *
 */
void c_io_imu_EulerMatrix(float * rpy, float * velAngular){

// Calculando a taxa de variacao dos angulos de Euler a partir da medicao dos gyros
  rpy[PV_IMU_DROLL ] = velAngular[PV_IMU_ROLL] +
		  	  	  	  	  	  	  velAngular[PV_IMU_PITCH ]*sin(rpy[PV_IMU_ROLL ])*tan(rpy[PV_IMU_PITCH ]) +
		  	  	  	  	  	  	  velAngular[PV_IMU_YAW ]*cos(rpy[PV_IMU_ROLL ])*tan(rpy[PV_IMU_PITCH ]);

  rpy[PV_IMU_DPITCH ] = velAngular[PV_IMU_PITCH ]*cos(rpy[PV_IMU_ROLL ]) - velAngular[PV_IMU_YAW ]*sin(rpy[PV_IMU_ROLL ]);

  rpy[PV_IMU_DYAW ] = velAngular[PV_IMU_PITCH ]*sin(rpy[PV_IMU_ROLL ])/cos(rpy[PV_IMU_PITCH ]) +
		  	  	  	  	  	  	  velAngular[PV_IMU_YAW ]*cos(rpy[PV_IMU_ROLL ])/cos(rpy[PV_IMU_PITCH ]);
}

/**\brief Quaternion para Euler
 * Solucao adaptada de http://www.sedris.org/wg8home/Documents/WG80485.pdf, "Technical Concepts Orientation,
 * Rotation, Velocity and Acceleration and the SRM" escrito por Paul Berner, página 39
 *
 * @param quaternion
 * @param angulo euler convencao ZYX
 */
void c_io_imu_Quaternion2Euler(float * q, float * rpy){

	float sqx = q[1]*q[1];
    float sqy = q[2]*q[2];
    float sqz = q[3]*q[3];
	float singularity_check = (q[1]*q[3] - q[0]*q[2]);

	if (singularity_check > 0.49) { // singularity at north pole
		rpy[0] = 0;
		rpy[1] = - PI/2;
		rpy[2] = 2 * atan2(q[1],q[0]);
		return;
	}
	if (singularity_check < -0.49) { // singularity at south pole
		rpy[0] = 0;
		rpy[1] =  PI/2;
		rpy[2] = -2 * atan2(q[1],q[0]);
		return;
	}

	rpy[0] = atan2(q[2]*q[3]+q[0]*q[1] , 0.5 - sqx - sqy);
	rpy[1] = asin(-2*singularity_check);
	rpy[2] = atan2(q[1]*q[2]+q[0]*q[3] , 0.5  - sqy - sqz);
}

//void c_io_imu_Euler2Quaternion(float * rpy, float * q){
//	q[0] = cos(rpy[0]/2)*cos(rpy[1]/2)*cos(rpy[2]/2) + sin(rpy[0]/2)*sin(rpy[1]/2)*sin(rpy[2]/2);
//	q[1] = sin(rpy[0]/2)*cos(rpy[1]/2)*cos(rpy[2]/2) - cos(rpy[0]/2)*sin(rpy[1]/2)*sin(rpy[2]/2);
//	q[2] = cos(rpy[0]/2)*sin(rpy[1]/2)*cos(rpy[2]/2) + sin(rpy[0]/2)*cos(rpy[1]/2)*sin(rpy[2]/2);
//	q[3] = cos(rpy[0]/2)*cos(rpy[1]/2)*sin(rpy[2]/2) - sin(rpy[0]/2)*sin(rpy[1]/2)*cos(rpy[2]/2);
//}


/**\brief Quaternion para Euler
 * Solucao adaptada de http://www.x-io.co.uk/res/doc/madgwick_internal_report.pdf, "An efficient orientation filter for inertial and
inertial/magnetic sensor arrays" escrito por Sebastian O.H. Madgwick, página 6, equacoes (7)(8)(9).
 *
 * @param quaternion
 * @param angulo euler
 */
void c_io_imu_Quaternion2EulerMadgwick(float * q, float * rpy){

	float singularity_check = 2*(q[1]*q[3] + q[0]*q[2]);

	if (singularity_check > 0.499) { // singularity at north pole
		rpy[0] = 0;
		rpy[1] = - PI/2;
		rpy[2] = 2 * atan2(q[1],q[0]);
		return;
	}
	if (singularity_check < -0.499) { // singularity at south pole
		rpy[0] = 0;
		rpy[1] =  PI/2;
		rpy[2] = -2 * atan2(q[1],q[0]);
		return;
	}

	rpy[0]=atan2(2*(q[2]*q[3] - q[0]*q[1]) , 2*(q[0]*q[0] + q[3]*q[3] -0.5));
	rpy[1]=-asin(singularity_check);
	rpy[2]=atan2(2*(q[1]*q[2] -q[0]*q[3]) , 2*(q[0]*q[0] +q[1]*q[1] -0.5));
}
/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */

