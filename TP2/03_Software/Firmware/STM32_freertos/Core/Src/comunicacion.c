/* funciones_comunes.c - corregido con pool de buffers RX para RS485 y rearme válido en errores */

// INCLUDE --------------------------------------------------------------------------
#include "comunicacion.h"
#include "usart.h"
#include "cmsis_os.h"
#include "freertos.h"
#include "queue.h"


// VARIABLES -------------------------------------------------------------------------
volatile uint8_t trama_recibida_uart_ttl[LONGITUD_TRAMA_RX];

volatile uint8_t active_frame[LONGITUD_TRAMA_RX ];
volatile bool busIdle = true;

volatile uint8_t trama_recibida_uart_rs485[LONGITUD_TRAMA_RX];
volatile uint8_t trama_rs485_pool[RX_BUFFER_POOL_SIZE][LONGITUD_TRAMA_RX ];
uint8_t pool_index = 0;

Modo_Protocolo_t modo_protocolo;	// Variable para selección de puerto UART: MODBUS ó TTL

// FUNCIONES -------------------------------------------------------------------------

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	volatile uint8_t *pTrama;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	uint32_t ptr;

	if (huart->Instance == USART1 && modo_protocolo == MODO_TTL)  // TTL
	{
		pTrama = trama_recibida_uart_ttl;
		ptr = (uint32_t)pTrama;
		HAL_UART_Receive_IT(&huart1, (uint8_t*)pTrama, LONGITUD_TRAMA_RX);

		if (pTrama[0] == HEADER_TRAMA)
		{
			xQueueSendFromISR(UART_RX_Queue, &ptr, &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
		return;
	}
	else if (huart->Instance == USART3 && modo_protocolo == MODO_MODBUS)  // RS485
	{
		pTrama = trama_rs485_pool[pool_index];
		pool_index = (pool_index + 1) % RX_BUFFER_POOL_SIZE;
		ptr = (uint32_t)pTrama;

		HAL_UART_Receive_IT(&huart3, (uint8_t*)pTrama, LONGITUD_TRAMA_RX );

		if (pTrama[0] == HEADER_TRAMA)
		{
			BaseType_t result = xQueueSendFromISR(UART_RX_Queue, &ptr, &xHigherPriorityTaskWoken);
			if (result == errQUEUE_FULL)
			{
				uint32_t dummy;
				xQueueReceiveFromISR(UART_RX_Queue, &dummy, NULL);
				xQueueSendFromISR(UART_RX_Queue, &ptr, &xHigherPriorityTaskWoken);
			}
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
		return;
	}
	else return;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART3)
	{
		if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE))
			__HAL_UART_CLEAR_OREFLAG(huart);
		if (__HAL_UART_GET_FLAG(huart, UART_FLAG_FE))
			__HAL_UART_CLEAR_FEFLAG(huart);
		if (__HAL_UART_GET_FLAG(huart, UART_FLAG_PE))
			__HAL_UART_CLEAR_PEFLAG(huart);

		// Rearme coherente con el pool
		HAL_UART_Receive_IT(&huart3, (uint8_t *)trama_rs485_pool[pool_index], LONGITUD_TRAMA_RX);
		pool_index = (pool_index + 1) % RX_BUFFER_POOL_SIZE;
	}
	else if (huart->Instance == USART1)
	{
		if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE))
			__HAL_UART_CLEAR_OREFLAG(huart);
		HAL_UART_Receive_IT(&huart1, (uint8_t*)trama_recibida_uart_ttl, LONGITUD_TRAMA_RX);
	}
}

/** -----------------------------------------------------------------------------------------------------
* @brief 			: Funcion para calculo del CRC-16
* @param argument	: Trama recibida por uart, longitud de trama
* @retval 			: Bytes CRC
*/

uint16_t calcular_crc16(const uint8_t *datos, uint8_t longitud) {

	uint16_t crc = CRC16_CLAVE ;
	for (uint8_t i = 0; i < longitud; i++)
	{
		crc ^= datos[i];
	    for (uint8_t j = 0; j < 8; j++)
	    {
	    	if (crc & 0x0001)
	    		crc = (crc >> 1) ^ 0xA001;
	        else
	            crc >>= 1;
	    }
	}
	return crc;
}
