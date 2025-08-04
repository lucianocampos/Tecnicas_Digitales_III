/*
 * funciones_comunes.h
 *
 *  Created on: Jul 13, 2025
 *      Author: L
 */

#ifndef SRC_COMUNICACION_H_
#define SRC_COMUNICACION_H_


// Librerias ---------------------------------------------------------------

#include "cmsis_os.h"
#include "stm32f1xx_hal.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include <string.h>

// Defines -----------------------------------------------------------------

#define HEADER_TRAMA 0X02
#define CRC16_CLAVE 0xFFFF			// Palabra clave del CRC-16

// RX STM32 - RECEPCION DE DATOS

#define LONGITUD_TRAMA_RX  8                      	// 1 header + 5 datos + 2 CRC16
#define LONGITUD_DATOS_RX  (LONGITUD_TRAMA_RX-2)  	// bytes previos al CRC
#define POS_CRC_LSB_RX     (LONGITUD_DATOS_RX)    	// índice LSB de CRC
#define POS_CRC_MSB_RX     (LONGITUD_DATOS_RX+1)  	// índice MSB de CRC

// TX STM32 - ENVIO DE DATOS
#define LONGITUD_TRAMA_TX  13						// Longitud en bytes del mensaje de salida
#define LONGITUD_DATOS_TX  (LONGITUD_TRAMA_TX-2)  	// bytes previos al CRC
#define POS_CRC_LSB_TX     (LONGITUD_DATOS_TX)    	// índice LSB de CRC
#define POS_CRC_MSB_TX     (LONGITUD_DATOS_TX+1)  	// índice MSB de CRC

#define CONTROL_TIMEOUT_MS 100
#define RX_BUFFER_POOL_SIZE 4



// Variables ---------------------------------------------------------------

extern volatile uint8_t active_frame[LONGITUD_TRAMA_RX];

//extern osMessageQId UART_TTL_RX_Queue;
//extern osMessageQId UART_RS485_RX_Queue;
extern QueueHandle_t UART_RX_Queue;

extern volatile uint8_t trama_recibida_uart_ttl[LONGITUD_TRAMA_RX];
extern volatile uint8_t trama_recibida_uart_rs485[LONGITUD_TRAMA_RX];

extern osTimerId TimerUARTHandle;

extern volatile bool busIdle;

extern volatile uint8_t trama_rs485_pool[RX_BUFFER_POOL_SIZE][LONGITUD_TRAMA_RX ];
extern uint8_t pool_index;

typedef enum {
    MODO_TTL = 0,
    MODO_MODBUS = 1
} Modo_Protocolo_t;

extern Modo_Protocolo_t modo_protocolo;

// Funciones ---------------------------------------------------------------

uint16_t calcular_crc16(const uint8_t *datos, uint8_t longitud);	//	CALCULO DEL CRC-16
void HAL_UART_RxCpltCallback(UART_HandleTypeDef*);

#endif /* SRC_COMUNICACION_H_ */
