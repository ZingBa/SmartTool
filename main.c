/* Includes ------------------------------------------------------------------*/
#include "stm32f4_discovery.h"
/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

//private includes
#include "alphaNum.h"
#include "functions.h"

//freertos functions
void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName );
void vApplicationMallocFailedHook(void);
void vApplicationIdleHook(void);

//VARIABLES
//to handle var
int count = 0;
int u = 3;

//private GLOBAL variables
xQueueHandle xAxisQueue;
xQueueHandle yAxisQueue;
xQueueHandle distanceQueue;
uint8_t xAxis, yAxis;
int distance;
xQueueHandle unitQueue;
int unit;

//read variables
int xV, yV, dV, u;

//remote task handling
xTaskHandle read;
xTaskHandle dist;
xTaskHandle blnc;

//read all input vales
void vGateKeeperFunction (void *pvParameters){
   
   while (1) 
   {  
     //read data
       //x axis
       xAxis = read_accel_x();
       //y axis
       yAxis = read_accel_y();
       //distance sensor
       distance = preciseMeasurement();
       //detect key pressed
       unit = buttonPressed();
       if(unit == -1) unit = u;
      
    //send data to queues
      //send X
      xQueueSendToBack(        xAxisQueue,        &xAxis,       portMAX_DELAY);
      //send Y
      xQueueSendToBack(        yAxisQueue,        &yAxis,       portMAX_DELAY);
      //send distance
      xQueueSendToBack(        distanceQueue,     &distance,    portMAX_DELAY);
      //send button
      xQueueSendToBack(        unitQueue,         &unit,        portMAX_DELAY);
      
      vTaskDelay(500 / portTICK_RATE_MS);        
   }
}
//update Screen distance
void vTask1Function (void *pvParameters){
   while (1) 
   {  
       //receive distance
       xQueueReceive(       distanceQueue,      &dV,            portMAX_DELAY );
       xQueueReceive(       unitQueue,          &u,             portMAX_DELAY );
       
       //show on screen
       updateScreen(u, dV);
       
       vTaskDelay(500 / portTICK_RATE_MS);     
   }
}
//update Screen balance
void vTask2Function (void *pvParameters){
     while (1) 
     {  
       //receive X
       xQueueReceive(        xAxisQueue,        &xV,        portMAX_DELAY );
       //receive Y
       xQueueReceive(        yAxisQueue,        &yV,        portMAX_DELAY );
       
       //update accelerator part of the screen
       update_accel(xV, yV);
       
       vTaskDelay(500 / portTICK_RATE_MS);          
     }
}


int main(void)
{
  //initialize all hardware required
  init();
  startUp();
  Delay_s(1);
  workScreen();
  Delay_s(1);
  
  //create data queues
  xAxisQueue =          xQueueCreate(    3,    sizeof(uint8_t)  );
  yAxisQueue =          xQueueCreate(    3,    sizeof(uint8_t)  );
  distanceQueue =       xQueueCreate(    3,    sizeof(int)  );
  unitQueue =           xQueueCreate(    3,    sizeof(int)  );
  
  //create tasks
  xTaskCreate(  vGateKeeperFunction,    ( const signed char * ) "GateKeeper",
    configMINIMAL_STACK_SIZE,    ( void* ) NULL,    tskIDLE_PRIORITY + 3UL,   
    &read  );
  
  xTaskCreate(  vTask1Function,         ( const signed char * ) "Distance",
    configMINIMAL_STACK_SIZE,    ( void* ) NULL,    tskIDLE_PRIORITY + 2UL,   
    &dist  );
  
  xTaskCreate(  vTask2Function,         ( const signed char * ) "Balance",
    configMINIMAL_STACK_SIZE,    ( void* ) NULL,    tskIDLE_PRIORITY + 1UL,   
    &blnc  );
      
  /*    RUN TASK SCHEDULER      */
  vTaskStartScheduler();
  
  /*    AFTER THIS POINT ERROR HANDLERS COME IN */
}

void vApplicationIdleHook(void){
  count++;
}


/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
  ( void ) pcTaskName;
  ( void ) pxTask;

  /* Run time stack overflow checking is performed if
  configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
  function is called if a stack overflow is detected. */
  taskDISABLE_INTERRUPTS();
  for( ;; );
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



#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif


