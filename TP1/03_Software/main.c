/* File:   main.c
 * Proyecto: TP1, TD3, UTN FRC
 * Created on 24 de marzo de 2025, 15:52
 */

#include <xc.h>
#include <pic12f629.h>
#include "pic_config.h"
#include "funciones.h"
#include "variables_definiciones.h"

//  INTERRUPCIONES -------------------------------------------------------------
void __interrupt() isr(void) {
     
    // INTERRUPCI�N POR TMR0 ***************************************************
    if (INTCONbits.T0IF) {
        TMR0 = 0;
        INTCONbits.T0IF = 0;            // Bandera de TMR0
    }


    // INTERRUPCI�N BIT START UART *********************************************
    if (INTCONbits.INTF){
        
        // se reinicia el WDT para evitar reinicios no esperados
        // Se deshabilitan interrupciones no deseadas y el PWM en caso de
        // que el �ltimo haya estado funcionando
        
        CLRWDT();
        OPTION_REGbits.PSA = 1;         // Asigna el prescaler al WDT
        OPTION_REGbits.PS = 0b111;      // Prescaler a 1:32 (T=2048ms)
        
        TMR0 = 0;
        INTCONbits.T0IF = 0;
        INTCONbits.T0IE = 1;   
        
        INTCONbits.GPIF = 0; 
        INTCONbits.GPIE = 0;
        
        FLAG_PWM_ENABLE = 0;
        PIR1bits.TMR1IF = 0;
        INTCONbits.PEIE = 0;        // Interrupciones perif�ricas
        
        // Si el dato le�do es v�lido seg�n RECEPCION_UART se responde a PC
        if (RECEPCION_UART()){
            RESPONDER_UART(0x1B);        // 0b00011011
            FLAG_PWM_ENABLE = 1;
            PWM_DUTY_CYCLE = DATO_RECIBIDO_UART;    // 50 NIVELES DE PWM, RESOLUCI�N PWM ~2.5%
        }
        
        TMR0 = 0;
        INTCONbits.T0IF = 0;
        INTCONbits.T0IE = 0;
        
        CONTADOR_REINICIOS_WDT = 0;
        T1CONbits.TMR1CS = 0;       // Selecciona el reloj interno (Fosc/4)
        T1CONbits.T1CKPS = 0b11;    // Configura el prescaler a 1:8
        TMR1 = PRECARGA_TMR1_WDT;   // Precarga TMR1
        PIR1bits.TMR1IF = 0;
        T1CONbits.TMR1ON = 1;       // Enciende Timer1
        PIE1bits.T1IE = 1;
        
        INTCONbits.GPIF = 0; 
        INTCONbits.INTF = 0;            // Limpieza de bandera
        
        INTCONbits.PEIE = 1;        // Interrupciones perif�ricas
        INTCONbits.GPIE = 1;                    // Habilito interrupcion por cambio en GPIO (pulsador).
    }
    
    // INTERRUPCI�N TMR1 (WDT RESET) *******************************************
    if (PIR1bits.TMR1IF){
        
        //  Cuando se establece la comunicaci�n serie y se recibe un dato
        //  se resetea CONTADOR_REINICIOS_WDT y este bloque temporiza 100 seg
        //  para enviar el mensaje a la PC. Luego temporiza otros 120 seg y
        //  bloquea el reseteo del watchdog pueda volver en el main
        
        TMR1 = PRECARGA_TMR1_WDT;
        PIR1bits.TMR1IF = 0;
                    
        if(FLAG_PWM_ENABLE == 1){
            CONTADOR_REINICIOS_WDT++;
            
            if (CONTADOR_REINICIOS_WDT == 200){  // 100 segs desde ultima COMM
                RESPONDER_UART(0xee);
            }
            
            if (CONTADOR_REINICIOS_WDT == 236){  // 100 segs desde ultima COMM
                RESPONDER_UART(0xee);
            }
            
            if (CONTADOR_REINICIOS_WDT == 240){  // 100 segs desde ultima COMM
                RESPONDER_UART(0xee);
                FLAG_PWM_ENABLE = 0;
            }
        }

        else    // Redundancia en desactivaci�n de la interrupci�n
            PIE1bits.TMR1IE = 0;
    }
    
    //  INTERRUPCI�N POR HW EN GP4 (PULSADOR) **********************************
    
    if (INTCONbits.GPIF) {    // Interrupci�n es cambio de estado en GPIO
        
        // Se revalida la interrupci�n verificando el estado del pulsador
        // y se ajusta el PWM en consecuencia.
        
        volatile unsigned char dummy_reg = GPIO;    // 
        INTCONbits.GPIF = 0;                        // Limpiar la bandera de interrupci�n
        
        if (PULSADOR == 1 && FLAG_PWM_ENABLE == 1)
            Ajustar_PWM();            
    }
}

// RECEPCI�N POR UART ----------------------------------------------------------
//      Esta funci�n guarda en la variable global DATO_RECIBIDO_UART el dato
//      recibido por comunicaci�n serie en el pin UART_RX

int RECEPCION_UART (void){

    int desplazamiento_bit = 0;
    int RECEPCION_EN_PROCESO = 1;
    int RESULTADO_COMM = 0;
    unsigned char CARACTER_RECIBIDO_UART = 0x00;
    
    // Se programa el timer 0 para hacer la lectura a mitad del tiempo de bit, y
    //  verificar que start_bit == 0 para validar que no se dispar� la 
    // interrupci�n por ruido el�ctrico
                                             
    
    if(UART_RX == 0){                                                           
        
        // Si se valida el start_bit se comienza la recepci�n. Al finalizar la
        // lectura de los bits de datos el dato se valida mediante el stop_bit
        TMR0 = UART_PERIOD_FULL;
        while(RECEPCION_EN_PROCESO == 1)
        {
            //CLRWDT();
            if(INTCONbits.T0IF){
                TMR0 = UART_PERIOD_FULL;
                INTCONbits.T0IF = 0;                                            
                if(desplazamiento_bit <8)
                    CARACTER_RECIBIDO_UART |= (UART_RX << desplazamiento_bit);
                
                else{
                    if (UART_RX == 1)                                           // Validaci�n del caracter recibido
                        RESULTADO_COMM = 1;
                    
                    RECEPCION_EN_PROCESO = 0;                                   
                }
                desplazamiento_bit++;
            }
        }
        
        // Se verifica que solo se env�o un caracter desde PC y se valida el dato recibido
        __delay_us(UART_PERIOD_FULL);                
        if((RESULTADO_COMM == 1) & (UART_RX == 1))                                                        
            DATO_RECIBIDO_UART= CARACTER_RECIBIDO_UART;
        else
            RESULTADO_COMM = 0;
    }
   
    // Se deshabilita uso de TMR0 para interrupci�n
    
    return (RESULTADO_COMM);
}


// RESPUESTA POR UART ----------------------------------------------------------
//      Esta funci�n env�a por pin UART_TX el caract�r RESPUESTA seg�n una
//      comunicaci�n serie 9600 bps, 8N1

void RESPONDER_UART(unsigned char RESPUESTA)
{
    int bit = 0;
    int ENVIO_EN_PROCESO = 1;       // FLAG PARA CONTROL DE ENV�O

    TMR0 = UART_PERIOD_FULL;

    while (ENVIO_EN_PROCESO && bit <= 10) {
        while (!INTCONbits.T0IF);   // Bucle de espera bloqueante
        
        TMR0 = UART_PERIOD_FULL;
        INTCONbits.T0IF = 0;
        
        switch (bit) {
            case 0: // Start bit
                UART_TX = 0;
                break;
                
            case 1:
            case 2:
            case 3:
            case 4:
            case 5:
            case 6:
            case 7:
            case 8: 
                // En cada iteraci�n se carga el bit 0 de la variable RESPUESTA
                // y se actualiza el valor de esa variable desplazando un bit
                // a la derecha, de esta manera todas las operaciones tienen
                // el mismo delay y el timing es m�s preciso
                UART_TX = RESPUESTA & 1;
                RESPUESTA >>=1;
                break;
                
            case 9:     // Stop bit
                UART_TX = 1;
                break;
                
            case 10:    // Termina el env�o
                UART_TX = 1;
                ENVIO_EN_PROCESO = 0; 
                break;
                
            default:
                break;
        }
        bit++;
    }
    
    return;
}


// AJUSTAR PWM POR PULSADOR ----------------------------------------------------

void Ajustar_PWM (void){
    
    if (PWM_DUTY_CYCLE < 25)        //  10%
        PWM_DUTY_CYCLE = 25;

    else if (PWM_DUTY_CYCLE < 64)   //  25%
        PWM_DUTY_CYCLE = 64;

    else if (PWM_DUTY_CYCLE < 122)  // 50%
        PWM_DUTY_CYCLE = 122;

    else if (PWM_DUTY_CYCLE < 192)  // 75%
        PWM_DUTY_CYCLE = 192;

    else if (PWM_DUTY_CYCLE < 255)  // 100%
        PWM_DUTY_CYCLE = 255;

    else if (PWM_DUTY_CYCLE == 255)
        PWM_DUTY_CYCLE = 0;
    
    return;
}


// CONFIGURACI�N DE INTERRUPCIONES POR TMR0 ------------------------------------
void set_timer_interrupt (unsigned char VALOR_PRECARGA_TIMER){
    
    TMR0 = VALOR_PRECARGA_TIMER;    // Precarga del registro
    INTCONbits.T0IF = 0;            // Reset bandera
    INTCONbits.T0IE = 1;            // Habilitar interrupci�n por TMR0
    return;
}


// PARPADEO DEL LED PARA INDICACIONES VISUALES ---------------------------------
 void LED_BLINK(int N, int periodo){
     
    for (int i=0; i<N; i++){
        PWM_OUT = ~PWM_OUT ;
        for(int j=0; j<periodo; j++){
            __delay_ms(1);
        }
    }
 }


// ******** MAIN ***************************************************************

void main(void) {
    //  CONFIGURACI�N GPIO -----------------------------------------------------
    TRISIObits.TRISIO0=0;       // GP0 = Salida PWM
    TRISIObits.TRISIO1=1;       // GP1 = Entrada Pulsador
    TRISIObits.TRISIO2=1;       // GP2 = UART RX
    TRISIObits.TRISIO4=0;       // GP4 = UART TX
    
    CMCON = 0x07;               // Desactivar comparadores
    PWM_OUT = 0;                // Inicializaci�n de GPIO PWM
    UART_TX = 1;                // Inicializaci�n de GPIO UART
    
    
    //  CONFIGURACI�N TIMER 0 --------------------------------------------------
    //  TMR0 controla el PWM
    OPTION_REGbits.T0CS = 0;    // CLK interno para TMR0. T=4/fosc
    TMR0 = 0;                   // Inicializo timer
        
    // CONFIGURACI�N WDT Y PRESCALER
    OPTION_REGbits.PSA = 1;     // Asigna el prescaler al WDT
    OPTION_REGbits.PS = 0b111;  // Prescaler a 1:32 (T=576ms)
         
    
    //  INTERRUPCIONES ---------------------------------------------------------
    INTCONbits.INTE = 1;        // Interrupci�n externa por GP2 (UART RX)
    OPTION_REGbits.INTEDG = 0;  // Interrupci�n en flanco de bajada en GP2 (UART RX)    
    INTCONbits.INTF = 0;        // Inicializaci�n de bandera
    
    IOCbits.IOC1 = 1;           // Interrupci�n por cambio en GP1 (PULSADOR)
    
    INTCONbits.T0IE = 0;        // Inicialmente deshabilitado (se habilita en la funci�n)
        
    INTCONbits.GIE = 1;         // Global Interrupt Enable. Permite atender interrupciones
    INTCONbits.GPIE = 0;        // interrupci�n por cambio en GPIOs deshabilitada
    
    //  VARIABLES --------------------------------------------------------------
    int Contador_PWM = 0;
        
    CLRWDT();
    
    //  Main -------------------------------------------------------------------
    while(1){
        
        if(FLAG_PWM_ENABLE == 1){            
            
            if(PWM_DUTY_CYCLE == 255)
                PWM_OUT = 1;
            
            else if(Contador_PWM < PWM_DUTY_CYCLE)
                PWM_OUT = 1;
            else
                PWM_OUT = 0;
            
            if(Contador_PWM < 255)
                Contador_PWM+=5;    // Resoluci�n PWM ~ 2.34%
            else
                Contador_PWM = 0;
        }
        
        if (CONTADOR_REINICIOS_WDT < MAX_REINICIOS_WDT)
            CLRWDT();
        
        if(STATUSbits.nTO == 0){
            FLAG_PWM_ENABLE = 0;
        }
    }
    return;
}

