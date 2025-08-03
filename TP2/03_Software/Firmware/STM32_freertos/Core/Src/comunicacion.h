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

#define CRC_CLAVE 0x00							// Palabra clave del CRC-8
#define LONGITUD_CADENA_CONTROL 7				// Longitud en bytes de la cadena de control recibida por la bluepill

#define CONTROL_TIMEOUT_MS 100
#define HEADER_TRAMA 0X02


// Variables ---------------------------------------------------------------

extern volatile uint8_t active_frame[LONGITUD_CADENA_CONTROL];

//extern osMessageQId UART_TTL_RX_Queue;
//extern osMessageQId UART_RS485_RX_Queue;
extern QueueHandle_t UART_RX_Queue;

extern volatile uint8_t trama_recibida_uart_ttl[LONGITUD_CADENA_CONTROL];
extern volatile uint8_t trama_recibida_uart_rs485[LONGITUD_CADENA_CONTROL];

extern osTimerId TimerUARTHandle;

extern volatile bool busIdle;

extern volatile uint8_t trama_rs485_pool[][LONGITUD_CADENA_CONTROL];
extern uint8_t pool_index;


// Funciones ---------------------------------------------------------------

uint8_t calcular_crc(const volatile uint8_t *, uint8_t);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef*);

#endif /* SRC_COMUNICACION_H_ */
