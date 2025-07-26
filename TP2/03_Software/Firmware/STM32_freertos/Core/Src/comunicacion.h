/*
 * funciones_comunes.h
 *
 *  Created on: Jul 13, 2025
 *      Author: L
 */

#ifndef SRC_COMUNICACION_H_
#define SRC_COMUNICACION_H_

#define CRC_CLAVE 0x00							// Palabra clave del CRC-8
#define LONGITUD_CADENA_CONTROL 7				// Longitud en bytes de la cadena de control recibida por la bluepill

#define CONTROL_TIMEOUT_MS 5000


// Librerias ---------------------------------------------------------------

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "cmsis_os.h"
#include "freertos.h"

// Variables ---------------------------------------------------------------

extern volatile uint8_t trama_recibida_uart_ttl[LONGITUD_CADENA_CONTROL];
extern volatile uint8_t trama_recibida_uart_rs485[LONGITUD_CADENA_CONTROL];

extern osMessageQId UART_TTL_RX_Queue;
extern osMessageQId UART_RS485_RX_Queue;

typedef enum {									// Arbitraje de bus maestro
    MASTER_NONE   = 0,
    MASTER_TTL    = 1,
    MASTER_RS485  = 2
} MasterPort_t;

extern volatile MasterPort_t master_port;
extern volatile TickType_t last_cmd_tick;
extern volatile uint8_t active_frame[LONGITUD_CADENA_CONTROL];


// Funciones ---------------------------------------------------------------

uint8_t calcular_crc(const volatile uint8_t *, uint8_t);

#endif /* SRC_COMUNICACION_H_ */
