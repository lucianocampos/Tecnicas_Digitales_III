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

#include <stdbool.h>
#include <string.h>

#include "usart.h"

#include "comunicacion.h"	// Recepción y validación de datos

#include "adc.h"

#include "spi.h"
#include "bmp280_spi.h"		// Sensor BMP280

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

extern volatile bool procesar_trama_recibida;
extern TIM_HandleTypeDef htim3;

uint8_t mensaje_de_salida[11] = {0};


osMutexId transmisionMutex;

osMutexId MensajeDeSalidaMutexHandle;	// Mutex para acceder a la variable global trama desde varias tareas de forma segura
osMutexDef(MensajeDeSalida);


/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId Uart_TTL_TaskHandle;
osThreadId ReadInputsTaskHandle;
osThreadId LeerADCHandle;
osThreadId LeerBMP280Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */



/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartUart_TTL_Task(void const * argument);
void StartReadInputsTask(void const * argument);
void StartLeerADC(void const * argument);
void StartLeerBMP280(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

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
	transmisionMutex = osMutexCreate(osMutex(transmisionMutex));	// Mutex para UART

	MensajeDeSalidaMutexHandle = osMutexCreate(osMutex(MensajeDeSalida)); // Mutex para mensaje_de_salida[]


  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Uart_TTL_Task */
  osThreadDef(Uart_TTL_Task, StartUart_TTL_Task, osPriorityAboveNormal, 0, 256);
  Uart_TTL_TaskHandle = osThreadCreate(osThread(Uart_TTL_Task), NULL);

  /* definition and creation of ReadInputsTask */
  osThreadDef(ReadInputsTask, StartReadInputsTask, osPriorityBelowNormal, 0, 128);
  ReadInputsTaskHandle = osThreadCreate(osThread(ReadInputsTask), NULL);

  /* definition and creation of LeerADC */
  osThreadDef(LeerADC, StartLeerADC, osPriorityNormal, 0, 256);
  LeerADCHandle = osThreadCreate(osThread(LeerADC), NULL);

  /* definition and creation of LeerBMP280 */
  osThreadDef(LeerBMP280, StartLeerBMP280, osPriorityLow, 0, 256);
  LeerBMP280Handle = osThreadCreate(osThread(LeerBMP280), NULL);

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

/* USER CODE BEGIN Header_StartUart_TTL_Task */
/**
* @brief Function implementing the Uart_TTL_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUart_TTL_Task */
void StartUart_TTL_Task(void const * argument)
{
  /* USER CODE BEGIN StartUart_TTL_Task */
  /* Infinite loop */
	  uint8_t trama_recibida_copia_local[LONGITUD_CADENA_CONTROL];
	  const TickType_t timeout = pdMS_TO_TICKS(10000); // Espera máxima para recibir respuesta
	  TickType_t espera_inicio;

	  uint8_t trama[12] = {
	      0x02,        // Byte 0 - Inicio
	      0x01, 0xA2,  // Byte 1-2
	      0x01, 0xAA,  // Byte 3-4
	      0x01, 0xA2,  // Byte 5-6
	      0x07,        // Byte 7
	      0x0F,        // Byte 8
	      0x01,		   // Byte 9
		  0x00   	   // CRC (CRC calculado de bytes 0-10)
	  };


	  for(;;)
	  {
		  osMutexWait(MensajeDeSalidaMutexHandle, osWaitForever);	// Permitir lectura de la variable mensaje_de_salida

		  for(int i=1; i<8; i++)
			  trama[i] = mensaje_de_salida[i];

		  osMutexRelease(MensajeDeSalidaMutexHandle);

		  trama[11] = calcular_crc(trama, 11);


		  // Envío de los datos
		  osMutexWait(transmisionMutex, osWaitForever);
		  //HAL_UART_Transmit(&huart1, (uint8_t*)"Hola Mundo\r\n", 12, HAL_MAX_DELAY);
		  HAL_UART_Transmit(&huart1, (const uint8_t*)trama, sizeof(trama), HAL_MAX_DELAY);
		  osMutexRelease(transmisionMutex);
		  //HAL_GPIO_TogglePin(led_GPIO_Port, led_Pin);


		  // Recepción de datos con timeout
		  espera_inicio = xTaskGetTickCount();
		  while ((procesar_trama_recibida == false) && (xTaskGetTickCount() - espera_inicio < timeout)) {
			  osDelay(10); // espera activa no agresiva
		  }

		  // Se validan los datos recibidos con el CRC (comunicacion.c) y se procesa la entrada
		  if (procesar_trama_recibida == true) {
			  taskENTER_CRITICAL();
		      memcpy(trama_recibida_copia_local, (const uint8_t *)trama_recibida, sizeof(trama_recibida));
		      procesar_trama_recibida = false;
		      taskEXIT_CRITICAL();

		      // Actualización de salidas digitales
		      HAL_GPIO_WritePin(DOUT_01_GPIO_Port, DOUT_01_Pin, (trama_recibida_copia_local[1] & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		      HAL_GPIO_WritePin(DOUT_02_GPIO_Port, DOUT_02_Pin, (trama_recibida_copia_local[1] & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		      HAL_GPIO_WritePin(DOUT_03_GPIO_Port, DOUT_03_Pin, (trama_recibida_copia_local[1] & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);

		      // Actualización de salidas PWM
		      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, trama_recibida_copia_local[2] | (trama_recibida_copia_local[3] << 8));
		      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, trama_recibida_copia_local[4] | (trama_recibida_copia_local[5] << 8));
		  }


		  osDelay(50);  // Intervalo entre ciclos (puede ser menor o mayor)
	  }
  /* USER CODE END StartUart_TTL_Task */
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
    osDelay(1);
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
		    //HAL_GPIO_TogglePin(led_GPIO_Port, led_Pin);
		}

		osMutexRelease(MensajeDeSalidaMutexHandle);

		osDelay(100);
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

  BMP280_SPI_Init(&hspi2, SPI_NSS_GPIO_Port, SPI_NSS_Pin);

  for(;;)
  {
	  temperatura = BMP280_ReadTemperature();
	  presion = BMP280_ReadPressure();
	  presion_entero = (uint32_t)presion;

	  osMutexWait(MensajeDeSalidaMutexHandle, osWaitForever);			// Protección de variable compartida

	  mensaje_de_salida[8] = (uint8_t)temperatura;
	  mensaje_de_salida[9] = (presion_entero >> 8) & 0xFF;
	  mensaje_de_salida[10] = presion_entero & 0xFF;

	  osMutexRelease(MensajeDeSalidaMutexHandle);
	  HAL_GPIO_TogglePin(led_GPIO_Port, led_Pin);

	  osDelay(50);

  }
  /* USER CODE END StartLeerBMP280 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

