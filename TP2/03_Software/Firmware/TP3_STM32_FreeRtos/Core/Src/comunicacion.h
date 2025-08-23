/*
 * funciones_comunes.h
 *
 *  Created on: Jul 13, 2025
 *      Author: L
 */

/**
 * @file funciones_comunes.h
 * @brief Declaraciones comunes para comunicación UART TTL y RS-485/Modbus en STM32 (HAL + FreeRTOS).
 * @details
 * Contiene macros de framing (encabezado y CRC-16), buffers y estados de recepción,
 * selección de protocolo en tiempo de ejecución, y prototipos de funciones públicas:
 * - Cálculo de CRC-16 (LSB-first, polinomio 0xA001, semilla @ref CRC16_CLAVE).
 * - Ventana de silencio t3.5 para Modbus (reinicio/expiración).
 * - Envío de trama por RS-485 con control DE/RE y respeto de t3.5.
 *
 * Supuestos:
 * - `LONGITUD_TRAMA_RX` coincide con el largo real de trama de entrada (8 bytes: 1 header + 5 datos + 2 CRC).
 * - `UART_RX_Queue` está creado y es consumido por tareas de usuario.
 * - `modo_protocolo` selecciona TTL (USART1) o Modbus (USART3).
 */

#ifndef SRC_COMUNICACION_H_
#define SRC_COMUNICACION_H_


/** \name Librerías
 *  \brief Dependencias del HAL y FreeRTOS.
 *  @{ */
// Librerias ---------------------------------------------------------------

#include "cmsis_os.h"
#include "stm32f1xx_hal.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include <string.h>
/** @} */

/** \name Defines de framing y CRC
 *  @{ */
// Defines -----------------------------------------------------------------

/** @brief Encabezado esperado en las tramas de entrada. */
#define HEADER_TRAMA 0X02

/** @brief Semilla del algoritmo CRC-16 (Modbus, LSB-first, polinomio 0xA001). */
#define CRC16_CLAVE 0xFFFF			// Palabra clave del CRC-16

/** @} */

/** \name RX STM32 - Recepción de datos
 *  \brief Layout de la trama de entrada y posiciones del CRC.
 *  @{ */

// RX STM32 - RECEPCION DE DATOS

/** @brief Longitud total de trama de RX [bytes]. */
#define LONGITUD_TRAMA_RX  8                      	// 1 header + 5 datos + 2 CRC16
/** @brief Número de bytes de datos previos al CRC. */
#define LONGITUD_DATOS_RX  (LONGITUD_TRAMA_RX - 2)  // bytes previos al CRC
/** @brief Índice del byte LSB del CRC en la trama RX. */
#define POS_CRC_LSB_RX     (LONGITUD_TRAMA_RX - 2)  // índice LSB de CRC
/** @brief Índice del byte MSB del CRC en la trama RX. */
#define POS_CRC_MSB_RX     (LONGITUD_TRAMA_RX - 1)  // índice MSB de CRC

/** @} */

/** \name TX STM32 - Envío de datos
 *  \brief Layout de la trama de salida y posiciones del CRC.
 *  @{ */

// TX STM32 - ENVIO DE DATOS
/** @brief Longitud total de trama de TX [bytes]. */
#define LONGITUD_TRAMA_TX  13						// Longitud en bytes del mensaje de salida
/** @brief Número de bytes de datos previos al CRC en TX. */
#define LONGITUD_DATOS_TX  (LONGITUD_TRAMA_TX-2)  	// bytes previos al CRC
/** @brief Índice del byte LSB del CRC en la trama TX. */
#define POS_CRC_LSB_TX     (LONGITUD_DATOS_TX)    	// índice LSB de CRC
/** @brief Índice del byte MSB del CRC en la trama TX. */
#define POS_CRC_MSB_TX     (LONGITUD_DATOS_TX+1)  	// índice MSB de CRC

/** @brief Timeout genérico de control (ms) para operaciones de comunicación. */
#define CONTROL_TIMEOUT_MS 100
/** @} */



/** \name Variables externas de comunicación
 *  \brief Buffers, colas y estados compartidos entre módulos.
 *  @{ */
// Variables ---------------------------------------------------------------

/** @brief Buffer activo genérico para manipulación de tramas entrantes. */
extern volatile uint8_t active_frame[LONGITUD_TRAMA_RX];
/** @brief Cola de recepción de UART; las tramas válidas se envían por valor. */
extern QueueHandle_t UART_RX_Queue;

/** @brief Buffer de RX por UART1 (TTL). */
extern volatile uint8_t trama_recibida_uart_ttl[LONGITUD_TRAMA_RX];
/** @brief Buffer de RX por UART3 (RS-485/Modbus). */
extern volatile uint8_t trama_recibida_uart_rs485[LONGITUD_TRAMA_RX];

/** @brief Timer de usuario asociado a UART (si se usa). */
extern osTimerId TimerUARTHandle;

/** @brief Estado del bus RS-485: true si silencio ≥ t3.5 (bus libre), false en actividad. */
extern volatile bool busIdle;

/**
 * @brief Selección de protocolo de comunicación.
 * @details
 * - @ref MODO_TTL : UART1 TTL
 * - @ref MODO_MODBUS : UART3 RS-485 (Modbus RTU)
 */
typedef enum {
    MODO_TTL = 0,
    MODO_MODBUS = 1
} Modo_Protocolo_t;

/** @brief Modo de protocolo vigente. */
extern Modo_Protocolo_t modo_protocolo;

/** @brief Buffer acumulador para RX RS-485 (byte-a-byte). */
extern volatile uint8_t  rs485_buf[LONGITUD_TRAMA_RX];  // Buffer para RS485
/** @brief Índice del próximo byte a escribir en @ref rs485_buf. */
extern volatile uint8_t  rs485_idx;		                // Indice de byte
/** @brief Último byte recibido (USART3) en modo byte-a-byte. */
extern volatile uint8_t  rs485_byte;                    // Recepción byte a byte
/** @} */


/** \name API pública
 *  \brief Prototipos de funciones exportadas.
 *  @{ */
// Funciones ---------------------------------------------------------------

/**
 * @brief Calcula el CRC-16 (Modbus) de un bloque de datos.
 * @param datos     Puntero a los bytes de entrada.
 * @param longitud  Cantidad de bytes a procesar (sin incluir el propio CRC) [bytes].
 * @return Valor de 16 bits (LSB primero).
 * @note Semilla @ref CRC16_CLAVE, polinomio 0xA001, LSB-first.
 */
uint16_t calcular_crc16(const uint8_t *datos, uint8_t longitud);	//	CALCULO DEL CRC-16

/**
 * @brief Calcula el CRC-16 (ruta de envío); misma convención que @ref calcular_crc16.
 * @param datos     Puntero a los bytes de entrada.
 * @param longitud  Cantidad de bytes a procesar [bytes].
 * @return Valor de 16 bits (LSB primero).
 */
uint16_t calcular_crc16_send(const uint8_t *datos, uint8_t longitud);	//	CALCULO DEL CRC-16

/**
 * @brief Callback de recepción completa de HAL UART (contexto ISR).
 * @param huart Puntero al handler de la UART.
 * @note Debe usarse con primitivas *FromISR* al interactuar con FreeRTOS.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef*);

/**
 * @brief Reinicia la ventana de silencio Modbus t3.5 (usa TIM2 en one-shot).
 * @note Tras reiniciar, el bus permanecerá ocupado hasta que expire t3.5.
 */
void mb_t35_restart(void);

/**
 * @brief Marca la expiración de t3.5; limpia parcial de RX y declara bus libre.
 * @note Invocada típicamente desde la ISR del timer configurado.
 */
void mb_t35_on_expire(void);

/**
 * @brief Envía una trama por RS-485 (USART3) respetando t3.5 y conmutando DE/RE.
 * @param pData   Puntero a los datos a transmitir.
 * @param Size    Longitud en bytes.
 * @param Timeout Timeout total para liberar bus y transmitir [ms].
 * @return HAL_OK en éxito; HAL_TIMEOUT si no se libera el bus; otros códigos HAL en error.
 * @warning Requiere `huart3` inicializado y pines DE/RE correctos.
 */
HAL_StatusTypeDef RS485_SendFrame(uint8_t *pData, uint16_t Size, uint32_t Timeout);

/** @} */

#endif /* SRC_COMUNICACION_H_ */
