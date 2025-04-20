/* File: Config.h  
 * Comments: TD3-TP1. Archivo de configuración
 * Revision history: A
 */

// VARIABLES GLOBALES PIC ------------------------------------------------------
#define _XTAL_FREQ 4000000              // Oscilador interno 12F629, 4MHz
#define PWM_OUT GPIObits.GP0            // Declaración de pines
#define PULSADOR GPIObits.GP1           // Declaración de pines
#define UART_RX GPIObits.GP2            // Declaración de pines     
#define UART_TX GPIObits.GP4            // Declaración de pines


// VARIABLES GLOBALES PROGRAMA -------------------------------------------------


// PWM -------------------------------------------------------------------------

const unsigned char PRECARGA_TMR0_PWM = 185;    // Valor de precarga = 255 - Periodo del PWM en segundos

volatile unsigned int FLAG_PWM_ENABLE = 0;      // ON/OFF de PWM

volatile unsigned int  Contador_PWM = 0;        //  Variable de conteo para pwm
volatile unsigned int  PWM_DUTY_CYCLE = 0;      //  Variable de manejo pwm
const int NIVELES_PWM = 50;                     //  Cantidad de niveles del PWM. Resolución 2.5%
volatile unsigned int FLAG_REFRESCAR_PWM;       // Refresco del PWM cada interrupción por timer 0


// UART ------------------------------------------------------------------------
volatile char DATO_RECIBIDO_UART = 0x00;        // Respuesta UART
const char DATO_ENVIADO_UART = 'a';             // Respuesta UART

const int UART_PERIOD_HALF = 52;                // Valor de precarga de timer 0 para delay de T/2 en comunicación serie 9600bps, 8N1 
const int UART_PERIOD_FULL = 165;               // Valor de precarga de timer 0 para delay de T/2 en comunicación serie 9600bps, 8N1 (ideal 150; se compensa el tiempo de instrucción)

volatile unsigned int FLAG_UART = 0;            // Detección de bit de start
volatile unsigned int FLAG_RECEPCION = 0;       // Validación de recepción
volatile unsigned int DESBORDE_TIMER0 = 0;      // Validación de recepción


// INTERCEPCIÓN DEL WDT --------------------------------------------------------
const long PRECARGA_TMR1_WDT = 3040;            // Precarga TMR1, 16 bits 
const int MAX_REINICIOS_WDT = 236;              // Variable para temporización de reinicio
volatile int CONTADOR_REINICIOS_WDT = 0;        // Variable para temporización de reinicio
volatile unsigned int WDT_RESET = 0;            // Reseteo por desborde de WDT