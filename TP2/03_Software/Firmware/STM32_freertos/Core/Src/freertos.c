/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "comunicacion.h"	// Recepción y validación de datos
#include "queue.h"
#include "usart.h"

#include "adc.h"
#include "spi.h"
#include "bmp280_spi.h"		// Sensor BMP280

#include <stdbool.h>
#include <string.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

//uint16_t adc_valores[3];

extern TIM_HandleTypeDef htim3;

uint8_t mensaje_de_salida[11] = {
	      0x02,        // Byte 0 - Inicio
	      0x01, 0xA2,  // Byte 1-2
	      0x01, 0xAA,  // Byte 3-4
	      0x01, 0xA2,  // Byte 5-6
	      0x07,        // Byte 7
	      0x0F,        // Byte 8
	      0x01,		   // Byte 9
		  0x00   	   // CRC (CRC calculado de bytes 0-10)
	  };


osMutexId transmisionMutex;

osMutexId MensajeDeSalidaMutexHandle;	// Mutex para acceder a la variable global trama desde varias tareas de forma segura
osMutexDef(MensajeDeSalida);

osMutexId  busDatosSalidasMutex;  		// Mutex para arbitraje entre uart TTL y RS485

osMessageQId UART_TTL_RX_Queue;			// Cola para puerto UART TTL
osMessageQId UART_RS485_RX_Queue;		// Cola para puerto UART TTL


/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId EnvioUartHandle;
osThreadId ReadInputsTaskHandle;
osThreadId LeerADCHandle;
osThreadId LeerBMP280Handle;
osThreadId LeerUARTHandle;
osTimerId TimerUARTHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */



/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Start_EnvioUart_Task(void const * argument);
void StartReadInputsTask(void const * argument);
void StartLeerADC(void const * argument);
void StartLeerBMP280(void const * argument);
void Start_LeerUart(void const * argument);
void CallbackTimerUART(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */

	osMutexDef(transmisionMutex);
	transmisionMutex = osMutexCreate(osMutex(transmisionMutex));			// Mutex para UART

	MensajeDeSalidaMutexHandle = osMutexCreate(osMutex(MensajeDeSalida)); 	// Mutex para mensaje_de_salida[]

	osMutexDef(busDatosSalidasMutex);
	busDatosSalidasMutex  = osMutexCreate(osMutex(busDatosSalidasMutex));	// Mutex para UART

	osMessageQDef(UART_TTL_RX_Queue, 4, uint8_t*);							// Cola para arbitraje de puertos UART
	UART_TTL_RX_Queue = osMessageCreate(osMessageQ(UART_TTL_RX_Queue), NULL);

	osMessageQDef(UART_RS485_RX_Queue, 4, uint8_t*);						// Cola para arbitraje de puertos UART
	UART_RS485_RX_Queue = osMessageCreate(osMessageQ(UART_RS485_RX_Queue), NULL);

  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of TimerUART */
  osTimerDef(TimerUART, CallbackTimerUART);
  TimerUARTHandle = osTimerCreate(osTimer(TimerUART), osTimerOnce, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  master_port = MASTER_TTL;
  osTimerStart(TimerUARTHandle, CONTROL_TIMEOUT_MS);
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of EnvioUart */
  osThreadDef(EnvioUart, Start_EnvioUart_Task, osPriorityAboveNormal, 0, 128);
  EnvioUartHandle = osThreadCreate(osThread(EnvioUart), NULL);

  /* definition and creation of ReadInputsTask */
  osThreadDef(ReadInputsTask, StartReadInputsTask, osPriorityBelowNormal, 0, 128);
  ReadInputsTaskHandle = osThreadCreate(osThread(ReadInputsTask), NULL);

  /* definition and creation of LeerADC */
  osThreadDef(LeerADC, StartLeerADC, osPriorityNormal, 0, 256);
  LeerADCHandle = osThreadCreate(osThread(LeerADC), NULL);

  /* definition and creation of LeerBMP280 */
  osThreadDef(LeerBMP280, StartLeerBMP280, osPriorityLow, 0, 256);
  LeerBMP280Handle = osThreadCreate(osThread(LeerBMP280), NULL);

  /* definition and creation of LeerUART */
  osThreadDef(LeerUART, Start_LeerUart, osPriorityHigh, 0, 512);
  LeerUARTHandle = osThreadCreate(osThread(LeerUART), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
	  osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Start_EnvioUart_Task */
/**
* @brief Function implementing the EnvioUart thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_EnvioUart_Task */
void Start_EnvioUart_Task(void const * argument)
{
  /* USER CODE BEGIN Start_EnvioUart_Task */
  /* Infinite loop */


	HAL_StatusTypeDef status_uart_ttl = HAL_ERROR, status_uart_rs485 = HAL_ERROR;

	uint8_t trama[12] = {0};
	uint8_t i, ReintentarEnvio = 0;


  for(;;)
  {
	  osMutexWait(MensajeDeSalidaMutexHandle, osWaitForever);	// Permitir lectura de la variable mensaje_de_salida

		for(i=0; i<11; i++)
			trama[i] = mensaje_de_salida[i];

		osMutexRelease(MensajeDeSalidaMutexHandle);

		trama[11] = calcular_crc(trama, 11);


		// Envío de los datos. Número de reintentos automáticos = 10

		osMutexWait(transmisionMutex, osWaitForever);


		// UART TTL

		if(master_port == MASTER_TTL)
		{
			for (ReintentarEnvio = 0; ReintentarEnvio < 10; ReintentarEnvio++) {
				status_uart_ttl = HAL_UART_Transmit(&huart1, trama, sizeof(trama), 500);
				if (status_uart_ttl == HAL_OK) break;
			}
		}

		else if (master_port == MASTER_RS485)
		{
			// UART RS485
			HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_SET);

			for (ReintentarEnvio = 0; ReintentarEnvio < 10; ReintentarEnvio++) {
				status_uart_rs485 = HAL_UART_Transmit(&huart3, trama, sizeof(trama), 500);
				if (status_uart_rs485 == HAL_OK) break;
			}

			HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET);
		}
/*
		// LED ERROR
		if (status_uart_ttl!= HAL_OK || status_uart_rs485 != HAL_OK) {
			for(i=0; i<5; i++){
				HAL_GPIO_TogglePin(led_GPIO_Port, led_Pin);
				osDelay(500);
			}

			HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_SET);
		}*/

		status_uart_ttl = status_uart_rs485 = HAL_ERROR;

		osMutexRelease(transmisionMutex);


    osDelay(20);
  }
  /* USER CODE END Start_EnvioUart_Task */
}

/* USER CODE BEGIN Header_StartReadInputsTask */
/**
* @brief Function implementing the ReadInputsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReadInputsTask */
void StartReadInputsTask(void const * argument)
{
  /* USER CODE BEGIN StartReadInputsTask */
  /* Infinite loop */
  for(;;)
  {
	  osMutexWait(MensajeDeSalidaMutexHandle, osWaitForever); 	// Permitir escritura de la variable mensaje_de_salida

	  mensaje_de_salida [7] = 0;
	  mensaje_de_salida [7] |= !HAL_GPIO_ReadPin(DIN_01_GPIO_Port, DIN_01_Pin);
	  mensaje_de_salida [7] |= !HAL_GPIO_ReadPin(DIN_02_GPIO_Port, DIN_02_Pin) << 1;
	  mensaje_de_salida [7] |= !HAL_GPIO_ReadPin(DIN_03_GPIO_Port, DIN_03_Pin) << 2;

	  osMutexRelease(MensajeDeSalidaMutexHandle);
    osDelay(250);
  }
  /* USER CODE END StartReadInputsTask */
}

/* USER CODE BEGIN Header_StartLeerADC */
/**
* @brief Function implementing the LeerADC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLeerADC */
void StartLeerADC(void const * argument)
{
  /* USER CODE BEGIN StartLeerADC */
  /* Infinite loop */

	for (;;)
	{
		osMutexWait(MensajeDeSalidaMutexHandle, osWaitForever);			// Protección de variable compartida

		for (int i = 0; i < 3; i++) {									// Carga de valores
			mensaje_de_salida[2*i + 1] = adc_valores[i] & 0xFF;
		    mensaje_de_salida[2*i + 2] = (adc_valores[i] >> 8) & 0xFF;
		}

		osMutexRelease(MensajeDeSalidaMutexHandle);

		osDelay(50);
	}
  /* USER CODE END StartLeerADC */
}

/* USER CODE BEGIN Header_StartLeerBMP280 */
/**
* @brief Function implementing the LeerBMP280 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLeerBMP280 */
void StartLeerBMP280(void const * argument)
{
  /* USER CODE BEGIN StartLeerBMP280 */
  /* Infinite loop */
	float temperatura;
	float presion;
	uint32_t presion_entero;
	uint8_t temp_u8 = 0;

	BMP280_SPI_Init(&hspi2, SPI_NSS_GPIO_Port, SPI_NSS_Pin);

	for(;;)
	{
		temperatura = BMP280_ReadTemperature();
		presion = BMP280_ReadPressure();
		presion_entero = (uint32_t)presion;

		osMutexWait(MensajeDeSalidaMutexHandle, osWaitForever);			// Protección de variable compartida

		temp_u8 = (uint8_t)temperatura;
		mensaje_de_salida[8]  = temp_u8;
		mensaje_de_salida[9] = (presion_entero >> 8) & 0xFF;
		mensaje_de_salida[10] = presion_entero & 0xFF;

		osMutexRelease(MensajeDeSalidaMutexHandle);

		osDelay(1000);
	}
  /* USER CODE END StartLeerBMP280 */
}

/* USER CODE BEGIN Header_Start_LeerUart */
/**
* @brief Function implementing the LeerUART thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_LeerUart */
void Start_LeerUart(void const * argument)
{
  /* USER CODE BEGIN Start_LeerUart */

	osEvent   evt;
	uint8_t  *frame, *local;
	osMessageQId procQueue, flushQueue;


	    for (;;) {

	    	// Determinación del puerto es maestro (se vacía el otro)

	    	if (master_port == MASTER_TTL)
	    	{
	    		procQueue  = UART_TTL_RX_Queue;
	    	    flushQueue = UART_RS485_RX_Queue;
	    	}
	    	else if (master_port == MASTER_RS485)
	    	{
	    		procQueue  = UART_RS485_RX_Queue;
	    	    flushQueue = UART_TTL_RX_Queue;
	    	}
	    	else
	    	{
	    		// Arranque inicial: por defecto TTL maestro
	    	    procQueue  = UART_TTL_RX_Queue;
	    	    flushQueue = UART_RS485_RX_Queue;
	    	    master_port = MASTER_TTL;
	    	}

	    	// Vaciado de la cola del puerto no-maestro
	    	while ((evt = osMessageGet(flushQueue, 0)).status == osEventMessage)
	    	{
	    	// descartar los mensajes
	    	}



	    	evt = osMessageGet(procQueue, CONTROL_TIMEOUT_MS);
	        if (evt.status == osEventMessage)
	        {
	            frame = (uint8_t*)evt.value.v;
	            /* Validar encabezado 0x02 y CRC8 */
	            if (frame[0] == HEADER_TRAMA &&
	                calcular_crc(frame, LONGITUD_CADENA_CONTROL - 1) == frame[LONGITUD_CADENA_CONTROL - 1])
	            {
	            	// Copiar Trama valida
	                memcpy((void*)active_frame, frame, LONGITUD_CADENA_CONTROL);

	                // Asignar maestro según cola
	                master_port = (procQueue == UART_TTL_RX_Queue)? MASTER_TTL : MASTER_RS485;
	                // Reiniciar watchdog UART
	                osTimerStop(TimerUARTHandle);
	                osTimerStart(TimerUARTHandle, CONTROL_TIMEOUT_MS);
	            }
	        }

	    	// Actualizar salidas según active_frame

	        if (master_port != MASTER_NONE) {

	        	local = (uint8_t *)active_frame;

	        	// Salidas digitales

	            HAL_GPIO_WritePin(DOUT_01_GPIO_Port, DOUT_01_Pin, (local[1] & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	            HAL_GPIO_WritePin(DOUT_02_GPIO_Port, DOUT_02_Pin, (local[1] & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	            HAL_GPIO_WritePin(DOUT_03_GPIO_Port, DOUT_03_Pin, (local[1] & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);


	            // Salidas PWM

	            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, local[2] | (local[3] << 8));
	            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, local[4] | (local[5] << 8));
	        }

	        osDelay(5);
	    }

  /* USER CODE END Start_LeerUart */
}

/* CallbackTimerUART function */
__weak void CallbackTimerUART(void const * argument)
{
  /* USER CODE BEGIN CallbackTimerUART */

	if(master_port == MASTER_TTL)
	{
		master_port = MASTER_RS485;
		HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);
	}

	else
	{
		master_port = MASTER_TTL;
		HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_SET);
	}

	osTimerStart(TimerUARTHandle, CONTROL_TIMEOUT_MS);

  /* USER CODE END CallbackTimerUART */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

