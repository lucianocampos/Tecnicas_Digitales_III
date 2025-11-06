#include <stdio.h>
#include "driver/adc.h"
#include "driver/dac.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "portmacro.h"
#include "mpu6050.h"
#include "roll_pitch.h"

#include "esp_log.h"
#include <stdlib.h>
#include <string.h>

// Modo DEBUG (ESP_LOGI en UART 0)
#define DEBUG_MODE 0

// Handle para notificaciones de tarea
TaskHandle_t xHandleComUART = NULL;

// Declaración de gpios
#define Pulsador_ISR   26       // GPIO del pulsador

// Declaración de parámetros I2C para el MPU6050
#define I2C_MASTER_SCL_IO    22 // SCL pin
#define I2C_MASTER_SDA_IO    21 // SDA pin
#define I2C_MASTER_FREQ_HZ   400000
#define I2C_MASTER_NUM       I2C_NUM_0
#define ESP_INTR_FLAG_DEFAULT 0

// Declaración de parámetros UART
#define UART_PORT_NUM      UART_NUM_2
#define UART_BAUD_RATE     115200
#define UART_TX_PIN        17
#define UART_RX_PIN        16
#define UART_RX_BUF_SIZE   256

// Declaración de modos de transmisión UART
#define MODO_ADC_DAC        0
#define MODO_MPU            1
#define MODO_MPU_ADC_DAC    2
#define MODO_MAX            2
#define HEADER_UART_MODO_ADC_DAC     0x02
#define HEADER_UART_MODO_MPU         0x04
#define HEADER_UART_MODO_MPU_ADC_DAC 0x07
#define ACK_UART                     0x07

// Antirebote del pulsador
#define DEBOUNCE_MS 500

// Declaración de parámetros de freertos
#define NUCLEO_COMM_MPU   0   // Core 0: Comunicación y lectura i2C
#define NUCLEO_ANALOGICO  1   // Core 1: Lectura ADC y DAC


// Declaración de funciones de inicialización
esp_err_t eInicializarPerifericos (void);
esp_err_t eInicializarMPU6050(void);
esp_err_t eCrearTareasFreeRTOS (void);
esp_err_t eInicializarISR_Pulsador(void);

// Declaración de la ISR del pulsador
void IRAM_ATTR vISR_Handler_Pulsador(void* arg);

// Declaración de funciones de tareas
static uint16_t crc_uart(const uint8_t *data, size_t len);
static inline uint8_t lo8(uint16_t x) { return (uint8_t)(x & 0xFF); }
static inline uint8_t hi8(uint16_t x) { return (uint8_t)((x >> 8) & 0xFF); }

// Declaración de tareas FreeRTOS
void vTaskManejoCanalesAnalogicos(void *pvParameters);
void vTaskLecturaMPU6050(void *pvParameters);
void vTaskComunicacionUart(void *pvParameters);

// Declaración de cola para comunicación UART
QueueHandle_t xQueueUART = 0;

// Estructura para los datos ADC (los mantiene sincronizados)
typedef struct{
    uint8_t valor_DAC;              // Modo de transmisión por UART
    uint16_t Valor_ADC[4];          // Array para los 4 canales ADC
    int16_t iAceleracion_raw[3];    // Aceleración raw del MPU6050
    int16_t iGiro_raw[3];           // Giro raw del MPU6050
} structColaFreeRTOS;

// Semaforo mutex para escritura de la cola
SemaphoreHandle_t xMutexCola = NULL;

static const char *TAG = "main";

void app_main(void)
{
    esp_err_t retorno;

    if(DEBUG_MODE)
    {
        ESP_LOGI(TAG, "[START]\tapp_main start");
        // Chequeo de funcionamiento luego del reset
        printf("Arranque puerto serie\n");
    }

    fflush(stdout);

    // Delay para apertura del puerto serie del monitor serie
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_log_level_set(TAG, ESP_LOG_INFO);

    retorno = eInicializarPerifericos();            // PERIFÉRICOS
    if (retorno != ESP_OK && DEBUG_MODE) {
        printf("[FALLA]\tError al inicializar los periféricos: %d\n", retorno);
        return;
    }

    retorno = eInicializarISR_Pulsador();          // ISR PULSADOR
    if (retorno != ESP_OK && DEBUG_MODE) {
        printf("[FALLA]\tError al inicializar la ISR del pulsador: %d\n", retorno);
        return;
    }

    retorno = eInicializarMPU6050();               // MPU6050
    if (retorno != ESP_OK && DEBUG_MODE) {
        printf("[FALLA]\tError al inicializar el MPU6050: %d\n", retorno);
        return;
    }
    // Mutex para acceso a la cola
    xMutexCola = xSemaphoreCreateMutex();
    if (xMutexCola == NULL && DEBUG_MODE) {
        printf("[FALLA]\tError al crear el mutex\n");
        return;
    }

    // Cola para comuniación entre tareas de adquisición y UART
    xQueueUART = xQueueCreate(1, sizeof(structColaFreeRTOS));
    if (xQueueUART == NULL && DEBUG_MODE) {
        printf("[FALLA]\tError al crear la cola UART\n");
        return;
    }

    // Creación de tareas FreeRTOS
    retorno = eCrearTareasFreeRTOS();
    if (retorno != ESP_OK && DEBUG_MODE) {
        printf("[FALLA]\tError al crear las tareas FreeRTOS: %d\n", retorno);
        return;
    }

    if(DEBUG_MODE)
        ESP_LOGI(TAG, "[OK]\tPerifericos iniciados, tareas creadas");

    return;
}

esp_err_t eInicializarPerifericos (void)
{
    if (DEBUG_MODE)
    {
        ESP_LOGI(TAG, "[START]\teInicializarPerifericos start");
    }

    esp_err_t resultado;

    // Configuración del ADC: 12 bits, canales 0, 3, 6 y 7 con atenuación de 11 dB
    resultado = adc1_config_width(ADC_WIDTH_BIT_12);
    if(resultado != ESP_OK && DEBUG_MODE){
        ESP_LOGI(TAG, "[FALLA]\tError en adc1_config_width");
        return resultado;
    }

    resultado = adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
    if(resultado != ESP_OK && DEBUG_MODE){
        ESP_LOGI(TAG, "[FALLA]\tError en adc1_config_channel_atten canal 0");
        return resultado;
    }

    resultado = adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11);
    if(resultado != ESP_OK && DEBUG_MODE){
        ESP_LOGI(TAG, "[FALLA]\tError en adc1_config_channel_atten canal 3");
        return resultado;
    }

    resultado = adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
    if(resultado != ESP_OK && DEBUG_MODE){
        ESP_LOGI(TAG, "[FALLA]\tError en adc1_config_channel_atten canal 6");
        return resultado;
    }

    resultado = adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11);
    if(resultado != ESP_OK && DEBUG_MODE){
        ESP_LOGI(TAG, "[FALLA]\tError en adc1_config_channel_atten canal 7");
        return resultado;
    }

    // Configuración del DAC: GPIO25

    resultado = dac_output_enable(DAC_CHANNEL_1);
    if(resultado != ESP_OK && DEBUG_MODE){
        ESP_LOGI(TAG, "[FALLA]\tError en dac_output_enable");
        return resultado;
    }

    // Configuración de la UART. 8N1, sin control de flujo

    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    resultado = uart_param_config(UART_PORT_NUM, &uart_config);
    if(resultado != ESP_OK && DEBUG_MODE){
        ESP_LOGI(TAG, "[FALLA]\tError en uart_param_config");
        return resultado;
    }

    resultado = uart_set_pin(UART_NUM_2, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if(resultado != ESP_OK && DEBUG_MODE){
        ESP_LOGI(TAG, "[FALLA]\tError en uart_set_pin");
        return resultado;
    }

    resultado = uart_driver_install(UART_PORT_NUM, UART_RX_BUF_SIZE, UART_RX_BUF_SIZE, 0, NULL, 0);
    if(resultado != ESP_OK && DEBUG_MODE){
        ESP_LOGI(TAG, "[FALLA]\tError en uart_driver_install");
        return resultado;
    }

    if (DEBUG_MODE)
        ESP_LOGI(TAG, "[OK]\teInicializarPerifericos OK");
    
    return ESP_OK;
}

esp_err_t eInicializarMPU6050(void)
{
    if(DEBUG_MODE)
        ESP_LOGI(TAG, "[START]\teInicializarMPU6050 start");
    
    esp_err_t resultado;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    resultado = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (resultado != ESP_OK && DEBUG_MODE) {
        printf("[FALLA]\tI2C error de configuración.");
        return resultado;
    }
    resultado = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, ESP_INTR_FLAG_DEFAULT);
    if (resultado != ESP_OK && DEBUG_MODE) {
        printf("[FALLA]\tI2C error de configuración.");
        return resultado;
    }

    resultado= mpu6050_init(I2C_MASTER_NUM);
    if(resultado != ESP_OK && DEBUG_MODE){
        printf(TAG, "[FALLA]\tError al inicializar la comunicación I2C con el MPU6050");
        return resultado;
    }   

    if(DEBUG_MODE)
        ESP_LOGI(TAG, "[OK]\teInicializarMPU6050 OK");

    return resultado;
}   

esp_err_t eInicializarISR_Pulsador(void)
{
    if(DEBUG_MODE)
        ESP_LOGI(TAG, "[START]\teInicializarISR_Pulsador");

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE, // Interrupción en flanco de bajada
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << Pulsador_ISR),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0); // Sin cola de eventos para ISR
    gpio_isr_handler_add(Pulsador_ISR, vISR_Handler_Pulsador, NULL); // ISR vacía como placeholder

    if (DEBUG_MODE)
        ESP_LOGI(TAG, "[OK]\teInicializarISR_Pulsador OK");

    return ESP_OK;
}

void IRAM_ATTR vISR_Handler_Pulsador(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    static TickType_t xTickPrevio = 0;                      // tick FreeRTOS del último pulso. Antirebote
    const TickType_t xTickActual = xTaskGetTickCount();    // tick FreeRTOS actual. Antirebote

    // Antirebote simple
    if ((xTickActual - xTickPrevio) >= pdMS_TO_TICKS(DEBOUNCE_MS)) {
        xTickPrevio = xTickActual;
        xTaskNotifyFromISR(xHandleComUART, 0, eNoAction, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    }

}

esp_err_t eCrearTareasFreeRTOS (void)
{
    if(DEBUG_MODE)
        ESP_LOGI(TAG, "[START]\teCrearTareasFreeRTOS");

    static uint8_t uCParameterToPass;
    TaskHandle_t xHandleADC = NULL;
    TaskHandle_t xHandleMPU6050 = NULL;
    TaskHandle_t xHandleUART = NULL;
    BaseType_t xRes;

    xRes = xTaskCreatePinnedToCore(vTaskManejoCanalesAnalogicos,
                "ManejoCanalesAnalogicos",
                2048,
                &uCParameterToPass,
                15,
                &xHandleADC,
                NUCLEO_ANALOGICO);
    if (xRes != pdPASS && DEBUG_MODE) {
        ESP_LOGE(TAG, "[FALLA]\tCreación tarea ManejoCanalesAnalogicos (res=%d)", (int)xRes);
        return ESP_FAIL;
    }

    xRes = xTaskCreatePinnedToCore(vTaskLecturaMPU6050,
                 "LecturaMPU6050",
                2048,
                &uCParameterToPass,
                14,
                &xHandleMPU6050,
                NUCLEO_COMM_MPU);
    if (xRes != pdPASS && DEBUG_MODE) {
        ESP_LOGE(TAG, "[FALLA]\tCreación tarea EscrituraDAC (res=%d)", (int)xRes);
        return ESP_FAIL;
    }

    xRes = xTaskCreatePinnedToCore(vTaskComunicacionUart,
                 "ComunicacionUART",
                3072,
                &uCParameterToPass,
                12,
                &xHandleComUART,
                NUCLEO_COMM_MPU);
    if (xRes != pdPASS && DEBUG_MODE) {
        ESP_LOGE(TAG, "[FALLA]\tCreación tarea ComunicacionUART (res=%d)", (int)xRes);
        return ESP_FAIL;
    }

    if(DEBUG_MODE)
        ESP_LOGI(TAG, "[OK]\tCreación Tareas FreeRTOS");

    return ESP_OK;
}

static uint16_t crc_uart(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xA001;
            else         crc = (crc >> 1);
        }
    }
    return crc;
}

void vTaskManejoCanalesAnalogicos(void *pvParameters)
{
    structColaFreeRTOS  sDatosADC = {0};
    uint8_t dac_valor = 0;
    int16_t dac_amplitud = 0, dac_max = 0, dac_min = 0, dac_offset = 128, dac_step_up=1, dac_step_down=1, dac_step = 1;    

    if(DEBUG_MODE)
        ESP_LOGI(TAG, "[START]\tFreeRTOS: vTaskManejoCanalesAnalogicos");

    for(;;)
    {
        // Sobrescribir datos en la cola UART
        if(xSemaphoreTake(xMutexCola, pdMS_TO_TICKS(10)) == pdTRUE) {

            xQueuePeek(xQueueUART, &sDatosADC, 0);

            // Lectura de las entradas analógicas
            sDatosADC.Valor_ADC[0]  = adc1_get_raw(ADC1_CHANNEL_0);   // GPIO36
            sDatosADC.Valor_ADC[1] = adc1_get_raw(ADC1_CHANNEL_3);    // GPIO39
            sDatosADC.Valor_ADC[2] = adc1_get_raw(ADC1_CHANNEL_6);    // GPIO34
            sDatosADC.Valor_ADC[3] = adc1_get_raw(ADC1_CHANNEL_7);    // GPIO35

            // Incremento o decremento del valor del DAC

            dac_amplitud    = (128*sDatosADC.Valor_ADC[0]/4096);
            dac_offset      = (255*sDatosADC.Valor_ADC[1]/4096)+10;
            dac_step_up     = (9*sDatosADC.Valor_ADC[2]/4096) + 1;
            dac_step_down   = (9*sDatosADC.Valor_ADC[3]/4096) + 1;
            
            dac_max = dac_offset + dac_amplitud;
            dac_min = dac_offset - dac_amplitud;

            dac_valor += dac_step;

            if (dac_valor >= dac_max)
            {
                dac_valor = dac_max;
                dac_step = -dac_step_down;
            }
            
            else if (dac_valor <= dac_min)
            {
                dac_valor = dac_min;
                dac_step =  dac_step_up;
            }
            
            // Escribir DAC y publicar en la cola
            dac_output_voltage(DAC_CHANNEL_1, dac_valor);
            sDatosADC.valor_DAC = dac_valor;
            
            xQueueOverwrite(xQueueUART, &sDatosADC);    //Sobrescritura de la cola
            xSemaphoreGive(xMutexCola);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void vTaskLecturaMPU6050(void *pvParameters)
{
    if(DEBUG_MODE)
        ESP_LOGI(TAG, "[START]\tFreeRTOS: vTaskLecturaMPU6050 started");
    
    structColaFreeRTOS  sDatosRawMPU6050 = {0}, sDatosPreviosCola = {0};

    //int16_t iAceleracion_x_raw, iAceleracion_y_raw, iAceleracion_z_raw;
    //int16_t iGiro_x_raw, iGiro_y_raw, iGiro_z_raw;

    //float fAceleracion_x, fAceleracion_y, fAceleracion_z;
    //float fGiro_x, fGiro_y, fGiro_z;

    float accel_bias[3] = {0.00f, 0.00f, 0.00f};
    float gyro_bias[3]  = {0.00f, 0.00f, 0.00f};

    mpu6050_calibrate(I2C_MASTER_NUM, accel_bias, gyro_bias);
    roll_pitch_init();

    while(1)
    {
        if(xSemaphoreTake(xMutexCola, pdMS_TO_TICKS(10)) == pdTRUE) {

            if (mpu6050_read_raw_data(  I2C_MASTER_NUM, 
                                        &sDatosRawMPU6050.iAceleracion_raw[0], 
                                        &sDatosRawMPU6050.iAceleracion_raw[1], 
                                        &sDatosRawMPU6050.iAceleracion_raw[2],
                                        &sDatosRawMPU6050.iGiro_raw[0], 
                                        &sDatosRawMPU6050.iGiro_raw[1], 
                                        &sDatosRawMPU6050.iGiro_raw[2]) == ESP_OK)
            {
            if(xQueuePeek(xQueueUART, &sDatosPreviosCola, 0))
            {
                sDatosRawMPU6050.valor_DAC = sDatosPreviosCola.valor_DAC;
                for(int i=0; i<4; i++)
                {
                    sDatosRawMPU6050.Valor_ADC[i] = sDatosPreviosCola.Valor_ADC[i];
                }
            }
            xQueueOverwrite(xQueueUART, &sDatosRawMPU6050);    //Sobrescritura de la cola
            }
            
            xSemaphoreGive(xMutexCola);
        }
        
        else if (DEBUG_MODE) {
            ESP_LOGE(TAG, "[FALLA]\tError al leer datos del MPU6050");
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void vTaskComunicacionUart(void *pvParameters)
{
    structColaFreeRTOS sDatosSalidaUart;
    uint8_t modo_transmision = 0;
    uint32_t ulNotifiedValue;

    float fAceleracion_x, fAceleracion_y, fAceleracion_z;
    float fGiro_x, fGiro_y, fGiro_z;

    uint8_t mensaje_uart[32];
    uint16_t crc;
    size_t n=0;

    if(DEBUG_MODE)
        ESP_LOGI(TAG, "[START]\tFreeRTOS: vTaskComunicacionUart started");
    
    while(1)
    {

        if(xTaskNotifyWait(0, 0, &ulNotifiedValue, 0) == pdTRUE)
        {
            modo_transmision = (modo_transmision + 1) % (MODO_MAX + 1);
            if(DEBUG_MODE)
                ESP_LOGI(TAG, "[OK]\tModo de transmisión cambiado");
        }

        /* Recepción de datos desde la cola UART: esperar hasta 200 ms por datos
           (evita comprobar sin bloqueo y perder la oportunidad de imprimir) */
        if (xQueueReceive(xQueueUART, &sDatosSalidaUart, pdMS_TO_TICKS(200)) == pdTRUE)
        {
            switch (modo_transmision) {
                case MODO_ADC_DAC:
                    if(DEBUG_MODE){    
                        printf("ADC 0: %4d\tADC 1: %4d\tADC 2:%4d\tADC 3:%4d\tDAC: %3d\n",
                                sDatosSalidaUart.Valor_ADC[0], 
                                sDatosSalidaUart.Valor_ADC[1], 
                                sDatosSalidaUart.Valor_ADC[2],
                                sDatosSalidaUart.Valor_ADC[3],
                                sDatosSalidaUart.valor_DAC);
                    }
                    // Preparación del mensaje UART
                    n = 0;
                    mensaje_uart[n++] = HEADER_UART_MODO_ADC_DAC;
                    // ADC y DAC
                    mensaje_uart[n++] = lo8(sDatosSalidaUart.Valor_ADC[0]);
                    mensaje_uart[n++] = hi8(sDatosSalidaUart.Valor_ADC[0]);
                    mensaje_uart[n++] = lo8(sDatosSalidaUart.Valor_ADC[1]);
                    mensaje_uart[n++] = hi8(sDatosSalidaUart.Valor_ADC[1]);
                    mensaje_uart[n++] = lo8(sDatosSalidaUart.Valor_ADC[2]);
                    mensaje_uart[n++] = hi8(sDatosSalidaUart.Valor_ADC[2]);
                    mensaje_uart[n++] = lo8(sDatosSalidaUart.Valor_ADC[3]);
                    mensaje_uart[n++] = hi8(sDatosSalidaUart.Valor_ADC[3]);
                    mensaje_uart[n++] = sDatosSalidaUart.valor_DAC; // 1 byte

                    break;

                case MODO_MPU:
                    if(DEBUG_MODE){
                    mpu6050_convert_accel(  sDatosSalidaUart.iAceleracion_raw[0], 
                                            sDatosSalidaUart.iAceleracion_raw[1],
                                            sDatosSalidaUart.iAceleracion_raw[2],
                                            &fAceleracion_x, &fAceleracion_y, &fAceleracion_z);

                    mpu6050_convert_gyro(   sDatosSalidaUart.iGiro_raw[0], 
                                            sDatosSalidaUart.iGiro_raw[1], 
                                            sDatosSalidaUart.iGiro_raw[2],
                                            &fGiro_x, &fGiro_y, &fGiro_z);

                    roll_pitch_update(fAceleracion_x, fAceleracion_y, fAceleracion_z, fGiro_x, fGiro_y, fGiro_z);

                    
                    printf( "A[m/s^2]: %.2f %.2f %.2f | G[°/s]: %.2f %.2f %.2f | Roll: %.2f Pitch: %.2f\n",
                            fAceleracion_x, fAceleracion_y, fAceleracion_z, 
                            fGiro_x, fGiro_y, fGiro_z, 
                            roll_get(), pitch_get());
                    }

                    n = 0;
                    mensaje_uart[n++] = HEADER_UART_MODO_MPU;
                    // Aceleración cruda
                    mensaje_uart[n++] = lo8((uint16_t)sDatosSalidaUart.iAceleracion_raw[0]);
                    mensaje_uart[n++] = hi8((uint16_t)sDatosSalidaUart.iAceleracion_raw[0]);
                    mensaje_uart[n++] = lo8((uint16_t)sDatosSalidaUart.iAceleracion_raw[1]);
                    mensaje_uart[n++] = hi8((uint16_t)sDatosSalidaUart.iAceleracion_raw[1]);
                    mensaje_uart[n++] = lo8((uint16_t)sDatosSalidaUart.iAceleracion_raw[2]);
                    mensaje_uart[n++] = hi8((uint16_t)sDatosSalidaUart.iAceleracion_raw[2]);

                    // Giro crudo
                    mensaje_uart[n++] = lo8((uint16_t)sDatosSalidaUart.iGiro_raw[0]);
                    mensaje_uart[n++] = hi8((uint16_t)sDatosSalidaUart.iGiro_raw[0]);
                    mensaje_uart[n++] = lo8((uint16_t)sDatosSalidaUart.iGiro_raw[1]);
                    mensaje_uart[n++] = hi8((uint16_t)sDatosSalidaUart.iGiro_raw[1]);
                    mensaje_uart[n++] = lo8((uint16_t)sDatosSalidaUart.iGiro_raw[2]);
                    mensaje_uart[n++] = hi8((uint16_t)sDatosSalidaUart.iGiro_raw[2]);

                    break;
                case MODO_MPU_ADC_DAC:
                    // Conversión de datos raw a unidades físicas

                    if(DEBUG_MODE){
                        printf("ADC 0: %4d\tADC 1: %4d\tADC 2:%4d\tDAC: %3d\n",
                                sDatosSalidaUart.Valor_ADC[0], 
                                sDatosSalidaUart.Valor_ADC[1], 
                                sDatosSalidaUart.Valor_ADC[2],
                                sDatosSalidaUart.valor_DAC);

                    mpu6050_convert_accel(  sDatosSalidaUart.iAceleracion_raw[0], 
                                            sDatosSalidaUart.iAceleracion_raw[1],
                                            sDatosSalidaUart.iAceleracion_raw[2],
                                            &fAceleracion_x, &fAceleracion_y, &fAceleracion_z);

                    mpu6050_convert_gyro(   sDatosSalidaUart.iGiro_raw[0], 
                                            sDatosSalidaUart.iGiro_raw[1], 
                                            sDatosSalidaUart.iGiro_raw[2],
                                            &fGiro_x, &fGiro_y, &fGiro_z);

                    roll_pitch_update(fAceleracion_x, fAceleracion_y, fAceleracion_z, fGiro_x, fGiro_y, fGiro_z);

                    
                    printf( "A[m/s^2]: %.2f %.2f %.2f | G[°/s]: %.2f %.2f %.2f | Roll: %.2f Pitch: %.2f\n",
                            fAceleracion_x, fAceleracion_y, fAceleracion_z, 
                            fGiro_x, fGiro_y, fGiro_z, 
                            roll_get(), pitch_get());
                    }

                    // Preparación del mensaje UART
                    n = 0;
                    mensaje_uart[n++] = HEADER_UART_MODO_MPU_ADC_DAC;
                    // ADC y DAC
                    mensaje_uart[n++] = lo8(sDatosSalidaUart.Valor_ADC[0]);
                    mensaje_uart[n++] = hi8(sDatosSalidaUart.Valor_ADC[0]);
                    mensaje_uart[n++] = lo8(sDatosSalidaUart.Valor_ADC[1]);
                    mensaje_uart[n++] = hi8(sDatosSalidaUart.Valor_ADC[1]);
                    mensaje_uart[n++] = lo8(sDatosSalidaUart.Valor_ADC[2]);
                    mensaje_uart[n++] = hi8(sDatosSalidaUart.Valor_ADC[2]);
                    mensaje_uart[n++] = lo8(sDatosSalidaUart.Valor_ADC[3]);
                    mensaje_uart[n++] = hi8(sDatosSalidaUart.Valor_ADC[3]);
                    mensaje_uart[n++] = sDatosSalidaUart.valor_DAC; // 1 byte

                    // Aceleración cruda
                    mensaje_uart[n++] = lo8((uint16_t)sDatosSalidaUart.iAceleracion_raw[0]);
                    mensaje_uart[n++] = hi8((uint16_t)sDatosSalidaUart.iAceleracion_raw[0]);
                    mensaje_uart[n++] = lo8((uint16_t)sDatosSalidaUart.iAceleracion_raw[1]);
                    mensaje_uart[n++] = hi8((uint16_t)sDatosSalidaUart.iAceleracion_raw[1]);
                    mensaje_uart[n++] = lo8((uint16_t)sDatosSalidaUart.iAceleracion_raw[2]);
                    mensaje_uart[n++] = hi8((uint16_t)sDatosSalidaUart.iAceleracion_raw[2]);

                    // Giro crudo
                    mensaje_uart[n++] = lo8((uint16_t)sDatosSalidaUart.iGiro_raw[0]);
                    mensaje_uart[n++] = hi8((uint16_t)sDatosSalidaUart.iGiro_raw[0]);
                    mensaje_uart[n++] = lo8((uint16_t)sDatosSalidaUart.iGiro_raw[1]);
                    mensaje_uart[n++] = hi8((uint16_t)sDatosSalidaUart.iGiro_raw[1]);
                    mensaje_uart[n++] = lo8((uint16_t)sDatosSalidaUart.iGiro_raw[2]);
                    mensaje_uart[n++] = hi8((uint16_t)sDatosSalidaUart.iGiro_raw[2]);

                    break;
                default:
                    if(DEBUG_MODE){
                        printf("[FALLA] Modo de transmisión desconocido\n");
                        ESP_LOGI(TAG, "[FALLA]\tModo de transmisión desconocido");
                    }
                    n=0;
                    break;
            }

            // Envío del mensaje UART con CRC16
            if (n > 0) {
                crc = crc_uart(mensaje_uart, n);
                mensaje_uart[n++] = lo8(crc);
                mensaje_uart[n++] = hi8(crc);

                uart_write_bytes(UART_PORT_NUM, (const char*)mensaje_uart, n);
                uart_wait_tx_done(UART_PORT_NUM, pdMS_TO_TICKS(50));
            }
            
            if(DEBUG_MODE)
                printf("\n");
            /* Forzar envío inmediato a la UART */
            fflush(stdout);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}