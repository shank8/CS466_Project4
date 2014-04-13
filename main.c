/*******************************************************************************
 * Programmer:                                                                 *
 * Class: CptS 466                                                             *
 * Lab Project:                                                                *
 * Date:                                                                       *
 *                                                                             *
 * Description:  FreeRTOS Project Template                                     *
 *                                                                             *
 ******************************************************************************/

// #include all necessary standard and user-defined libraries
#include <plib.h> // Includes all major functions and macros required to develop
                  // programs for the PIC32MX4
#include <p32xxxx.h> // Need specific PIC32 names for memory regions

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"
#include "ConfigPerformance.h"

#include "system_config.h"
#include "motor_control.h"
#include <peripheral/spi.h>
#include "timers.h"
#include "timer_util.h"
#include "pmodACL.h"
#include "semphr.h"

typedef enum {
      EXCEP_IRQ = 0,            // interrupt
      EXCEP_AdEL = 4,            // address error exception (load or ifetch)
      EXCEP_AdES,                // address error exception (store)
      EXCEP_IBE,                // bus error (ifetch)
      EXCEP_DBE,                // bus error (load/store)
      EXCEP_Sys,                // syscall
      EXCEP_Bp,                // breakpoint
      EXCEP_RI,                // reserved instruction
      EXCEP_CpU,                // coprocessor unusable
      EXCEP_Overflow,            // arithmetic overflow
      EXCEP_Trap,                // trap (possible divide by zero)
      EXCEP_IS1 = 16,            // implementation specfic 1
      EXCEP_CEU,                // CorExtend Unuseable
      EXCEP_C2E                // coprocessor 2
  } _excep_code;


unsigned long long timer2Counter = 0;
unsigned long long totalMotorRunCounter = 0;
// Place your #pragma statements here, or in another .h file;
// #pragma statements are used to set your operating clock frequency

/* SYSCLK = 80 MHz (8 MHz Crystal/ FPLLIDIV * FPLLMUL / FPLLODIV)
// PBCLK = 40 MHz
// Primary Osc w/PLL (XT+,HS+,EC+PLL)
// WDT OFF
// Other options are don't care */

/* Oscillator Settings
*/
#pragma config FNOSC = PRIPLL // Oscillator selection
#pragma config POSCMOD = EC // Primary oscillator mode
#pragma config FPLLIDIV = DIV_2 // PLL input divider
#pragma config FPLLMUL = MUL_20 // PLL multiplier
#pragma config FPLLODIV = DIV_1 // PLL output divider
#pragma config FPBDIV = DIV_2 // Peripheral bus clock divider
#pragma config FSOSCEN = OFF // Secondary oscillator enable
//
#define MOTOR_ID 0
#define MOTOR_DELAY_ID 1
#define PWM_COUNTER_ID 2
#define DISPLAY_STATS_ID 3
// Place your #define constants and macros here, or in another .h file

static void prvSetupHardware( void );
void aclControl(void * pvParameters);
void displayStats(void * pvParameters);
void motorTimerCallback( TimerHandle_t timer);
void motorDelayTimerCallback( TimerHandle_t timer);
void pwmTimerCallback( TimerHandle_t timer);
void displayStatsTimerCallback(TimerHandle_t timer);

 TimerHandle_t motorTimerHandle = NULL;
 TimerHandle_t motorAfterDelayTimerHandle = NULL;
 TimerHandle_t timerCounterHandle = NULL;
 TimerHandle_t displayStatsHandle = NULL;

 static xSemaphoreHandle sema = NULL;
 
 int aclDataReady = 0;
 float xAxis = 0, yAxis = 0, zAxis = 0;
// Tasks

int main (void)
{
    
	// Variable declarations
        
    
	// Setup/initialize ports
	// Setup/initialize devices
        prvSetupHardware ();

        vSemaphoreCreateBinary(sema);
        // 1) Motor Control
        // 2) I2C Control
        // 3) SPI Control
        xTaskCreate(aclControl, "ACL Control", 200, NULL, tskIDLE_PRIORITY + 1, NULL);
        //xTaskCreate(displayStats, "Display Stats", 200, NULL, tskIDLE_PRIORITY + 2, NULL);
    
        motorTimerHandle = xTimerCreate("motorTimer", 500 / portTICK_PERIOD_MS, pdTRUE, (void *) MOTOR_ID, motorTimerCallback);
        motorAfterDelayTimerHandle = xTimerCreate("motorDelayTimer", 500 / portTICK_PERIOD_MS, pdTRUE, (void *)MOTOR_DELAY_ID, motorDelayTimerCallback);
        timerCounterHandle = xTimerCreate("timerCounter", PWM_PERIOD, pdTRUE, (void *)PWM_COUNTER_ID, pwmTimerCallback);
        displayStatsHandle = xTimerCreate("displayStatsTimer", 1000 / portTICK_PERIOD_MS, pdTRUE, NULL, displayStatsTimerCallback);

        if(displayStatsHandle != NULL){
            xTimerStart(displayStatsHandle, 0);
        }
        vTaskStartScheduler ();

        // Should not reach this point!
	while (1) // Embedded programs run forever
	{
		// Event loop
	}

	return 0;
}

void aclControl(void * pvParameters){
    PMODACL_AXIS pmodACLAxis;
    
    
    while(1){
        
        //xSemaphoreTake(sema, portMAX_DELAY);
        
        if(aclDataReady){
            aclDataReady = 0;
            PmodACLGetAxisData(SPI_CHANNEL2,&pmodACLAxis);
            xAxis = pmodACLAxis.xAxis;
            yAxis = pmodACLAxis.yAxis;
            zAxis = pmodACLAxis.zAxis;
        }
    }
}

void motorTimerCallback( TimerHandle_t timer){
   // configASSERT(timer);

    // This is called every 500ms and should increment motor control
    // variables that are used to measure speed and distance
    totalMotorRunCounter++;
    control_motor();
}
void motorDelayTimerCallback( TimerHandle_t timer){
   // configASSERT(timer);
    control_motor_after_delay();
}
void pwmTimerCallback( TimerHandle_t timer){
    //Increment time counter
    timer2Counter++;
}
void displayStatsTimerCallback(TimerHandle_t timer){
    // Display statistics every 1 second
    display_statistics();
}

/*************************************************************
 * Function:                                                 *
 * Date Created:                                             *
 * Date Last Modified:                                       *
 * Description:                                              *
 * Input parameters:                                         *
 * Returns:                                                  *
 * Usages:                                                   *
 * Preconditions:                                            *
 * Postconditions:                                           *
 *************************************************************/

// Put other function definitions below main (), or they
// may go in another .c source file; Functions most likely
// will include port and device setups/initalizations;
// Be sure to comment all functions with the above block
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
            /* Set up SPI for display of speed, distance, temperature and total time*/
        setup_UART1();
        initialize_CLS();
      
        setup_SPI2();

        /* Set up switches for input triggers */
        setup_switches();
        /* Set up hbridge for the motor running */
        set_hbridge();

        /* Set up the input capture module to detect SA pulses of the motors */
        setup_IC2();
        setup_IC3();

        /* Set I2C to read temperature from Pmod Tmp2 */
        setup_I2C();

        /* Configure corresponding interrupts */
        configure_interrupts();

	/* Configure the hardware for maximum performance. */
	vHardwareConfigurePerformance();

	/* Setup to use the external interrupt controller. */
	vHardwareUseMultiVectoredInterrupts();

	portDISABLE_INTERRUPTS();

}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If the application makes use of the
	vTaskDelete() API function (as this demo application does) then it is also
	important that vApplicationIdleHook() is permitted to return to its calling
	function, because it is the responsibility of the idle task to clean up
	memory allocated by the kernel to any task that has since been deleted. */
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time task stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook	function is
	called if a task stack overflow is detected.  Note the system/interrupt
	stack is not checked. */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
	/* This function will be called by each tick interrupt if
	configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
	added here, but the tick hook is called from an interrupt context, so
	code must not attempt to block, and only the interrupt safe FreeRTOS API
	functions can be used (those that end in FromISR()). */
}
/*-----------------------------------------------------------*/

void _general_exception_handler( unsigned long ulCause, unsigned long ulStatus )
{
	/* This overrides the definition provided by the kernel.  Other exceptions
	should be handled here. */
        _excep_code error = ulStatus;

	for( ;; );
}
/*-----------------------------------------------------------*/

void vAssertCalled( const char * pcFile, unsigned long ulLine )
{
volatile unsigned long ul = 0;

	( void ) pcFile;
	( void ) ulLine;

	__asm volatile( "di" );
	{
		/* Set ul to a non-zero value using the debugger to step out of this
		function. */
		while( ul == 0 )
		{
			portNOP();
		}
	}
	__asm volatile( "ei" );
}


void __ISR(_EXTERNAL_1_VECTOR, ipl7auto) Ext3Handler_PmodACLInt1(void)
{
 //we have data, signal main loop to collect
    aclDataReady = 1;
    portBASE_TYPE taskWoken;
   // OC1CONbits.ON = 1;
    //xSemaphoreGiveFromISR (sema, &taskWoken);
    INTClearFlag(INT_INT1);
}

