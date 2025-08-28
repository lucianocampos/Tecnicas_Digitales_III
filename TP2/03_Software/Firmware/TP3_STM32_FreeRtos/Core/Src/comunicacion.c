/**
 * @file funciones_comunes.c
 * @brief Callbacks UART (TTL/RS-485), ventana t3.5 para Modbus y utilidades comunes.
 * @details
 * Definiciones previas:
 * - UART1: TTL con trama fija de LONGITUD_TRAMA_RX (8 bytes típicos).
 * - UART3: RS-485/Modbus, recepción byte-a-byte y ventana de silencio ≥ t3.5.
 *
 * Supuestos de funcionamiento:
 * - Las colas de RX (`UART_RX_Queue`) están creadas antes de iniciar la recepción.
 * - `modo_protocolo` determina en tiempo de ejecución si se procesa TTL o Modbus.
 * - `LONGITUD_TRAMA_RX` coincide con la longitud de trama esperada (8).
 *
 * NOTA: Solo se añadieron comentarios. No se modificó lógica, nombres, tamaños ni llamadas HAL/FreeRTOS.
 */

/* funciones_comunes.c - corregido con pool de buffers RX para RS485 y rearme válido en errores */

// INCLUDE --------------------------------------------------------------------------
#include "comunicacion.h"
#include "usart.h"
#include "tim.h"
#include "cmsis_os.h"
#include "freertos.h"
#include "queue.h"


// VARIABLES -------------------------------------------------------------------------
/**
 * @brief Buffer de recepción por UART1 (TTL), trama fija de LONGITUD_TRAMA_RX bytes.
 * @note Acceso en ISR, declarado volatile.
 */
volatile uint8_t trama_recibida_uart_ttl[LONGITUD_TRAMA_RX];

/**
 * @brief Buffer de recepción por UART3 (RS-485), trama fija de LONGITUD_TRAMA_RX bytes.
 * @note Acceso en ISR, declarado volatile.
 */
volatile uint8_t trama_recibida_uart_rs485[LONGITUD_TRAMA_RX];

/**
 * @brief Buffer activo genérico para operaciones temporales de trama (no usado directamente en callbacks).
 */
volatile uint8_t active_frame[LONGITUD_TRAMA_RX ];

/**
 * @brief Indica si el bus RS-485 permanece en silencio ≥ t3.5 (true = libre).
 * @details Se pone a false mientras hay actividad; se vuelve true al expirar t3.5.
 */
volatile bool busIdle = true;

/**
 * @brief Acumulador de bytes para RX Modbus (pool/ventana) y su índice/último byte recibido.
 */
volatile uint8_t  rs485_buf[LONGITUD_TRAMA_RX];
volatile uint8_t  rs485_idx  = 0;
volatile uint8_t  rs485_byte = 0;

/**
 * @brief Modo de protocolo seleccionado por GPIO/jumper al inicio (TTL o Modbus).
 */
Modo_Protocolo_t modo_protocolo;	// Variable para selección de puerto UART: MODBUS ó TTL

/**
 * @brief Reinicia la ventana de silencio Modbus (t3.5) usando TIM2 en one-shot.
 * @details Deja `busIdle=false` hasta que expire (en `mb_t35_on_expire()` se vuelve true).
 */
void mb_t35_restart(void);   // reinicia la ventana de silencio ≥ t3.5

// FUNCIONES -------------------------------------------------------------------------

/**
 * @brief Callback de recepción completa de HAL UART (contexto ISR).
 * @param huart Puntero al manejador HAL de la UART.
 * @details
 * Comportamiento:
 * - **USART1 + MODO_TTL**: recibe trama fija de LONGITUD_TRAMA_RX. Si `HEADER_TRAMA` es válido, encola POR VALOR.
 *   Rearma `HAL_UART_Receive_IT()` para la siguiente trama completa.
 * - **USART3 + MODO_MODBUS**: recepción byte-a-byte en `rs485_byte`, se acumula en `rs485_buf` hasta completar
 *   LONGITUD_TRAMA_RX; en ese punto encola POR VALOR y reinicia índice. Se rearma recepción de 1 byte.
 *
 * Condiciones de validez:
 * - `UART_RX_Queue` válido y creado.
 * - Interacciones con FreeRTOS mediante primitivas *FromISR* y `portYIELD_FROM_ISR`.
 *
 * @warning No ejecutar llamadas bloqueantes dentro del ISR. Mantener el código crítico al mínimo.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // --- USART1: TTL, trama fija de 8 bytes, por valor a la cola ---
    if (huart->Instance == USART1 && modo_protocolo == MODO_TTL)
    {
        uint8_t *p = (uint8_t*)trama_recibida_uart_ttl;

        // Encolar POR VALOR (8 bytes) solo si el encabezado es válido
        if (p[0] == HEADER_TRAMA) {
            xQueueSendFromISR(UART_RX_Queue, p, &xHigherPriorityTaskWoken);
        }

        // Rearmar recepción de 8 bytes
        HAL_UART_Receive_IT(&huart1, (uint8_t*)trama_recibida_uart_ttl, LONGITUD_TRAMA_RX);

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        return;
    }

    // --- USART3: RS485/Modbus, trama fija de 8 bytes, por valor a la cola ---
    if (huart->Instance == USART3 && modo_protocolo == MODO_MODBUS)
    {
    	//HAL_GPIO_TogglePin(DOUT_01_GPIO_Port, DOUT_01_Pin);

        // Ventana de recepción de 3.5 caracteres
        busIdle = false;
        mb_t35_restart();

        if (rs485_idx < LONGITUD_TRAMA_RX) {	// Recepción byte a byte
            rs485_buf[rs485_idx++] = rs485_byte;
            if (rs485_idx == LONGITUD_TRAMA_RX) {
            	xQueueSendFromISR(UART_RX_Queue, (const void*)rs485_buf, &xHigherPriorityTaskWoken);
                rs485_idx = 0;
            }
        }

        // Rearme de recepción byte a byte
        HAL_UART_Receive_IT(&huart3, (uint8_t*)&rs485_byte, 1);

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        return;
    }

}

/**
 * @brief Callback de error UART de HAL (contexto ISR).
 * @param huart Puntero al manejador de la UART que presentó el error.
 * @details
 * - **USART3**: lee SR/DR para limpiar condiciones pendientes; limpia ORE/FE/PE si aplican; reinicia el índice y rearma RX de 1 byte.
 * - **USART1**: limpia ORE si aplica y rearma recepción de LONGITUD_TRAMA_RX bytes.
 *
 * @note La lectura de SR y DR (descartada) sigue el flujo recomendado por la referencia para limpiar flags.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART3)
	{

		volatile uint32_t sr = huart->Instance->SR;
		volatile uint32_t dr = huart->Instance->DR;
		(void)sr; (void)dr;

		if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE))
			__HAL_UART_CLEAR_OREFLAG(huart);

		if (__HAL_UART_GET_FLAG(huart, UART_FLAG_FE))
			__HAL_UART_CLEAR_FEFLAG(huart);

		if (__HAL_UART_GET_FLAG(huart, UART_FLAG_PE))
			__HAL_UART_CLEAR_PEFLAG(huart);

		rs485_idx = 0;
		HAL_UART_Receive_IT(&huart3, (uint8_t*)&rs485_byte, 1);
	}
	else if (huart->Instance == USART1)
	{
		if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE))
			__HAL_UART_CLEAR_OREFLAG(huart);
		HAL_UART_Receive_IT(&huart1, (uint8_t*)trama_recibida_uart_ttl, LONGITUD_TRAMA_RX);
	}
}

/** -----------------------------------------------------------------------------------------------------
* @brief 			: Funcion para calculo del CRC-16 (estilo Modbus, LSB-first, polinomio 0xA001).
* @param datos		: Puntero a los bytes de datos de entrada.
* @param longitud	: Cantidad de bytes a procesar (sin incluir el propio CRC) [bytes].
* @retval 			: Valor CRC de 16 bits (LSB primero).
* @details
* Semilla: `CRC16_CLAVE` (definida externamente).
* Algoritmo: XOR inicial por byte; por cada bit, si LSB=1, (crc>>1)^0xA001; en caso contrario, (crc>>1).
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


/**
 * @brief Flag interno que indica expiración de t3.5 (TIM2) para RS-485/Modbus.
 * @note Escrito en `mb_t35_on_expire()`, leído por la lógica de RX.
 */
static volatile uint8_t g_t35_expired = 0;

/**
 * @brief Configura y arranca TIM2 en modo one-shot para un tiempo en microsegundos.
 * @param usec Duración deseada del disparo único [µs].
 * @details
 * - Recalcula prescaler a ~1 MHz según PCLK1 y factor de duplicación en F1.
 * - Ajusta ARR = usec-1 (con manejo de caso usec=0).
 * - Habilita interrupción de update y arranca el contador.
 *
 * @note El .ioc puede dejar PSC estimado; aquí se recalcula por robustez ante cambios de clock.
 */
static void tim2_start_oneshot_us(uint32_t usec)
{
    __HAL_TIM_DISABLE(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    // Dejá PSC del .ioc apuntando ~1 MHz. Igual lo recalculo por si cambia el clock:
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    RCC_ClkInitTypeDef clk; uint32_t lat;
    HAL_RCC_GetClockConfig(&clk, &lat);
    if (clk.APB1CLKDivider != RCC_HCLK_DIV1) pclk1 *= 2;  // regla F1

    uint32_t psc = (pclk1 + 1000000UL - 1) / 1000000UL;   // ~1 MHz
    if (psc) psc -= 1;
    __HAL_TIM_SET_PRESCALER(&htim2, psc);

    __HAL_TIM_SET_AUTORELOAD(&htim2, (usec == 0) ? 0 : (usec - 1));
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(&htim2);
}

/**
 * @brief Tiempo de carácter (tchar) para 115200 8N1 en microsegundos.
 * @return 87 [µs].
 * @details 10 bits/char / 115200 ≈ 86,8 µs → 87 µs.
 */
static inline uint32_t mb_char_time_us_115200_8N1(void)
{
    return 87; // tchar
}

/**
 * @brief Reinicia la ventana de silencio Modbus (t3.5) a partir de tchar calculado.
 * @details t3.5 = ceil(3.5 * tchar). Se usa TIM2 en one-shot; al expirar se marcará `busIdle=true`.
 */
void mb_t35_restart(void)
{
    g_t35_expired = 0;
    const uint32_t tchar = mb_char_time_us_115200_8N1();
    const uint32_t t35   = (tchar * 7) / 2 + 1;     // ceil(3.5 * tchar)
    tim2_start_oneshot_us(t35);
}

/**
 * @brief Handler de expiración de t3.5 (invocado desde la ISR del TIM correspondiente).
 * @details
 * - Si había bytes parciales (`0 < rs485_idx < LONGITUD_TRAMA_RX`), descarta parcial y reinicia índice.
 * - Marca `busIdle=true` (bus libre por silencio ≥ t3.5).
 */
void mb_t35_on_expire(void)
{
    g_t35_expired = 1;
    if (rs485_idx != 0 && rs485_idx < LONGITUD_TRAMA_RX) {
        rs485_idx = 0;                 // reset “empezar de cero”
    }
    busIdle = true;   // silencio ≥ t3.5 -> bus libre
}

/**
 * @brief Envía una trama por RS-485 (USART3) respetando ventana de silencio t3.5.
 * @param pData   Puntero a la trama a transmitir.
 * @param Size    Longitud en bytes.
 * @param Timeout Tiempo máximo de espera [ms] tanto para liberar bus como para HAL_UART_Transmit.
 * @retval HAL_StatusTypeDef HAL_OK en éxito; HAL_TIMEOUT si el bus no se libera en el plazo; otros códigos HAL en error.
 * @details
 * Secuencia:
 * 1) Espera `busIdle==true` (silencio ≥ t3.5) con timeout cooperativo (`taskYIELD()`).
 * 2) Sube DE/RE (modo TX), transmite y espera `UART_FLAG_TC`.
 * 3) Baja DE/RE (modo RX) y reinicia ventana t3.5 para evitar colisiones.
 *
 * @warning Asegurar que `huart3` esté inicializado y que los pines DE/RE correspondan al transceiver.
 */
HAL_StatusTypeDef RS485_SendFrame(uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    // Esperar a que el bus esté libre (t3.5 vencido)
    uint32_t tickstart = HAL_GetTick();
    while (!busIdle)
    {
        if ((HAL_GetTick() - tickstart) > Timeout)
            return HAL_TIMEOUT;   // no se liberó el bus en el tiempo dado
        taskYIELD();              // ceder CPU si usás FreeRTOS
    }

    // Subir DE (pasar a transmisión)
    busIdle = false;
    HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_SET);

    // Enviar trama
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart3, pData, Size, Timeout);

    // Esperar a que el UART termine de transmitir físicamente
    if (status == HAL_OK)
    {
        while (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_TC) == RESET) { }
    }

    // Bajar DE (volver a recepción)
    HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET);

    mb_t35_restart();	// ventana de silencio 3.5 caracteres (ModBus)

    return status;
}

/**
 * @brief Callback débil para evento IDLE de UART (línea en reposo).
 * @param huart Puntero al manejador de la UART correspondiente.
 * @details En MODO_MODBUS sobre USART3, reinicia t3.5 cuando se detecta IDLE.
 * @note Implementación `__weak`: puede sobreescribirse en otro módulo.
 */
__weak void HAL_UART_IDLE_Callback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3 && modo_protocolo == MODO_MODBUS) {
    	mb_t35_restart();	// por defecto idle se marca tiene cada 1 char pero en ModBus se necesita 3.5
    }
}
