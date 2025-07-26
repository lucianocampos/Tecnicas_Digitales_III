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

volatile bool procesar_trama_recibida_uart_ttl = false;		// Bandera para procesar datos recibidos una vez validados los datos con el CRC-8
volatile bool procesar_trama_recibida_uart_rs485 = false;	// Bandera para procesar datos recibidos una vez validados los datos con el CRC-8

volatile uint8_t trama_recibida_uart_ttl[LONGITUD_CADENA_CONTROL];	// trama recibida validada con CRC para manejo de salidas.
volatile uint8_t trama_recibida_uart_rs485[LONGITUD_CADENA_CONTROL];	// trama recibida validada con CRC para manejo de salidas.

volatile MasterPort_t master_port = MASTER_NONE;
volatile TickType_t   last_cmd_tick = 0;

volatile uint8_t active_frame[LONGITUD_CADENA_CONTROL];


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
    BaseType_t  xHigherPriorityTaskWoken = pdFALSE;
    TickType_t  now = xTaskGetTickCountFromISR();
    uint8_t     crc;

    if (huart->Instance == USART1) {  								// Puerto TTL prioritario
        crc = calcular_crc((const uint8_t *)trama_recibida_uart_ttl,
                           (uint8_t)(LONGITUD_CADENA_CONTROL - 1));
        if (crc == trama_recibida_uart_ttl[LONGITUD_CADENA_CONTROL - 1]) {

            memcpy((void *)active_frame, (const void *)trama_recibida_uart_ttl, LONGITUD_CADENA_CONTROL);

            master_port   = MASTER_TTL;
            last_cmd_tick = now;
        }

        HAL_UART_Receive_IT(&huart1, (uint8_t *)trama_recibida_uart_ttl, (uint16_t)LONGITUD_CADENA_CONTROL);
    }

    else if (huart->Instance == USART3) {  // RS485 solo si TTL caducó
        crc = calcular_crc((const uint8_t *)trama_recibida_uart_rs485,
                           (uint8_t)(LONGITUD_CADENA_CONTROL - 1));
        if (	crc == trama_recibida_uart_rs485[LONGITUD_CADENA_CONTROL - 1] &&
        		(master_port != MASTER_TTL ||
        		(now - last_cmd_tick) > pdMS_TO_TICKS(CONTROL_TIMEOUT_MS))){

        	memcpy((void *)active_frame, (const void *)trama_recibida_uart_rs485, (size_t)LONGITUD_CADENA_CONTROL);

            master_port   = MASTER_RS485;
            last_cmd_tick = now;
        }

        HAL_UART_Receive_IT(&huart3, (uint8_t *)trama_recibida_uart_rs485, (uint16_t)LONGITUD_CADENA_CONTROL);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*{
	uint8_t crc_calculado;

    if ( huart->Instance == USART1 )
    {
        crc_calculado = calcular_crc((const uint8_t *)trama_recibida_uart_ttl, LONGITUD_CADENA_CONTROL-1);
        procesar_trama_recibida_uart_ttl = (crc_calculado == trama_recibida_uart_ttl[LONGITUD_CADENA_CONTROL - 1]);
        osMessagePut(UART_TTL_RX_Queue, (uint32_t)trama_recibida_uart_ttl, 0);
        HAL_UART_Receive_IT(&huart1, (uint8_t *)trama_recibida_uart_ttl, LONGITUD_CADENA_CONTROL);
    }

    else if (huart->Instance == USART3)
    {
    	crc_calculado = calcular_crc((const uint8_t *)trama_recibida_uart_rs485, LONGITUD_CADENA_CONTROL - 1);
    	procesar_trama_recibida_uart_rs485 = (crc_calculado == trama_recibida_uart_rs485[LONGITUD_CADENA_CONTROL - 1]);
        osMessagePut(UART_RS485_RX_Queue, (uint32_t)trama_recibida_uart_rs485, 0);
    	HAL_UART_Receive_IT(&huart3, (uint8_t *)trama_recibida_uart_rs485, LONGITUD_CADENA_CONTROL);
    }
}*/


/* VALIDACIÓN DE DATOS : CALCULO DE CRC ------------------------------------------------
 *@brief Calcula el byte de CRC con la palabra clave definida en comunicacion.h para
 *		 validar tramas recibidas
 */

uint8_t calcular_crc(const volatile uint8_t *trama_recibida, uint8_t length) {
    uint8_t crc = CRC_CLAVE;  // Palabra clave

    for (uint8_t i = 0; i < length; i++) {
        crc ^= trama_recibida[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
        }
    }
    return crc;
}
