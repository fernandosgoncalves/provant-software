/**
  ******************************************************************************
  * @file    app/remote-controlled-flight/main.c
  * @author  Martin Vincent Bloedorn & Patrick José Pereira
  * @version V1.0.0
  * @date    27-August-2014
  * @brief   Startup do projeto.
  * @warning Modificar os arquivos 
    provant-software/io-board/stm32f4/common/modules/common/stm32f4xx_conf.h
    provant-software/io-board/stm32f4/lib/cmsis/inc/stm32f4xx.h
    provant-software/io-board/stm32f4/lib/cmsis/inc/stm32f4xx_conf.h
    dependendo da placa que esta trabalhando.
    Recompilando com:
    make allclean
    make
  *	TODO
  *
  *****************************************************************************/
/* Includes ------------------------------------------------------------------*/

/* Std includes */
#include <stdio.h>
#include <math.h>
#include "inttypes.h"

/* Hardware includes. */
#include "stm32f4xx_conf.h"
#include "system_stm32f4xx.h"
#define ARM_MATH_CM4
#include "arm_math.h"

/* FreeRTOS kernel includes */
#include "FreeRTOS.h"
#include "queue.h"
#include "timers.h"
#include "task.h"

/* FreeRTOS+Trace Includes */
#include "trcUser.h"

/* ProVANT Modules */
#include "pv_module_co.h"
#include "pv_module_in.h"
#include "pv_module_do.h"
#include "pv_module_gps.h"
#include "pv_module_sm.h"

/* Common Components, FOR TESTING */
#include "c_common_gpio.h"
#include "c_io_blctrl.h"
#include "c_io_sonar.h"
#include "c_io_herkulex.h"

/** @addtogroup ProVANT_Modules
  * \brief Ponto de entrada do software geral do VANT.
  *
  * @{
  */

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void vApplicationTickHook() {};
void vApplicationIdleHook() {};
void vApplicationStackOverflowHook() {};
void vApplicationMallocFailedHook() {};


void FPU_init(){
	/* Enable FPU.*/
	__asm("  LDR.W R0, =0xE000ED88\n"
		"  LDR R1, [R0]\n"
		"  ORR R1, R1, #(0xF << 20)\n"
		"  STR R1, [R0]");

	//#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
		SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
	//#endif
}

/* Tasks ----------------------------------------------------------------------*/

// `I'm alive` kind of task
void blink_led_task(void *pvParameters)
{
  GPIOPin LED_builtin;
  GPIOPin LED_builtin2;
  #ifdef STM32F4_H407
    LED_builtin = c_common_gpio_init(GPIOC, GPIO_Pin_13, GPIO_Mode_OUT);
  #else
    LED_builtin = c_common_gpio_init(GPIOA, GPIO_Pin_7, GPIO_Mode_OUT);
    LED_builtin2 = c_common_gpio_init(GPIOB, GPIO_Pin_1, GPIO_Mode_OUT);
    c_common_gpio_toggle(LED_builtin2);
  #endif
    while(1)
    {
      c_common_gpio_toggle(LED_builtin);
      #ifdef STM32F4_DISCOVERY
        c_common_gpio_toggle(LED_builtin2);
      #endif
      vTaskDelay(100/portTICK_RATE_MS);
    }
}

// Task control output 
void module_co_task(void *pvParameters)
{
	module_co_run();
}

// Task input 
void module_in_task(void *pvParameters)
{
	module_in_run();
}

// Task data out 
void module_do_task(void *pvParameters)
{
  module_do_run();
}

// Task gps
void module_gps_task(void *pvParameters)
{
  module_gps_run();
}

// Task state machine
void module_sm_task(void *pvParameters)
{
  module_sm_run();
}

/* Main ----------------------------------------------------------------------*/
int main(void)
{
	/* Init system and trace */
	SystemInit();
	FPU_init();

	vTraceInitTraceData();
	vTraceConsoleMessage("Starting application...");
	if (! uiTraceStart() )
		vTraceConsoleMessage("Could not start recorder!");

	/* Init modules */
	module_in_init();
	module_co_init();
    module_do_init();

    /* Connect modules: interface1.o* = interface2.i* */
    pv_interface_do.iInputData  = pv_interface_in.oInputData;
    pv_interface_co.iInputData  = pv_interface_in.oInputData;
    pv_interface_do.iControlOutputData  = pv_interface_co.oControlOutputData;

	/* create tasks
	 * Prioridades - quanto maior o valor, maior a prioridade
	 * */
    xTaskCreate(blink_led_task, (signed char *)"Blink led", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY+1, NULL);
    xTaskCreate(module_do_task, (signed char *)"Data out", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY+1, NULL);
    xTaskCreate(module_in_task, (signed char *)"Data input", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY+3, NULL);
    xTaskCreate(module_co_task, (signed char *)"Control + output", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY+3, NULL);
    //xTaskCreate(module_gps_task, (signed char *)"Gps", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY+2, NULL);
    //xTaskCreate(module_sm_task, (signed char *)"State machine", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY+1, NULL);

	//xTaskCreate(sonar_task, (signed char *)"Sonar task", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY+1, NULL);

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* should never reach here! */
	for(;;);
}

/**
  * @}
  */
