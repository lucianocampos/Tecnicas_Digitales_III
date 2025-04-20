// FUNCIONES -------------------------------------------------------------------

int RECEPCION_UART (void);                      // COMM. SERIE, RECEPCIÓN, 9600bps, 8N1
void RESPONDER_UART (unsigned char RESPUESTA);  // COMM. SERIE, ENVÍO, 9600bps, 8N1
void set_timer_interrupt (unsigned char );      // HABILITA INTERRUPCIÓN POR TMR0 Y PRECARGA EL REGISTRO CORRESPONDIENTE
void Ajustar_PWM (void);                        // CAMBIO DE CICLO DE TRABAJO MEDIANTE INTERRUPCIÓN POR PULSADOR
void LED_BLINK (int, int);                      // PARPADEO DE LED PARA INDICACIONES VISUALES