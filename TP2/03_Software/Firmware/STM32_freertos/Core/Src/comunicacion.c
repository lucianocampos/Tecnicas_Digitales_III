/*
 * funciones_comunes.c
 *
 *  Created on: Jul 13, 2025
 *      Author: L
 */

#include "comunicacion.h"
#include "usart.h"			// Para manejo del puerto UART
#include <stdbool.h>


//  VARIABLES ------------------------------------------------------------------------

volatile bool procesar_trama_recibida = false;	// Bandera para procesar datos recibidos una vez validados los datos con el CRC-8
volatile uint8_t trama_recibida[LONGITUD_CADENA_CONTROL];			// trama recibida validada con CRC para manejo de salidas.


/** RECEPCIÓN Y VALIDACIÓN DE DATOS --------------------------------------------------
 * @brief Callback que se ejecuta automáticamente cuando se completa
 *        la recepción UART en modo interrupción.
 *
 *        Verifica el CRC del mensaje recibido y, si es válido,
 *        activa una bandera para procesar los datos.
 *        Luego reinicia la recepción UART para seguir escuchando.
 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uint8_t crc_calculado = calcular_crc((const uint8_t *)trama_recibida, LONGITUD_CADENA_CONTROL-1);
        if (crc_calculado == trama_recibida[LONGITUD_CADENA_CONTROL-1]) {
            procesar_trama_recibida = true;
        }
        else{
        	procesar_trama_recibida = false;
        }
        HAL_UART_Receive_IT(&huart1, (uint8_t *)trama_recibida, sizeof(trama_recibida));
    }
}



/* VALIDACIÓN DE DATOS : CALCULO DE CRC ------------------------------------------------
 *@brief Calcula el byte de CRC con la palabra clave definida en comunicacion.h para
 *		 validar tramas recibidas
 */

uint8_t calcular_crc(const uint8_t *trama_recibida, uint8_t length) {
    uint8_t crc = CRC_CLAVE;  // Palabra clave

    for (uint8_t i = 0; i < length; i++) {
        crc ^= trama_recibida[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
        }
    }
    return crc;
}
