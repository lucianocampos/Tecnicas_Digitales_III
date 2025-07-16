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


// Librerias ---------------------------------------------------------------

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>

// Variables ---------------------------------------------------------------

extern volatile uint8_t trama_recibida[LONGITUD_CADENA_CONTROL];


// Funciones ---------------------------------------------------------------

uint8_t calcular_crc(const uint8_t *, uint8_t);

#endif /* SRC_COMUNICACION_H_ */
