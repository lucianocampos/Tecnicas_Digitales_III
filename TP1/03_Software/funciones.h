// FUNCIONES -------------------------------------------------------------------

int RECEPCION_UART (void);                      // COMM. SERIE, RECEPCI�N, 9600bps, 8N1
void RESPONDER_UART (unsigned char RESPUESTA);  // COMM. SERIE, ENV�O, 9600bps, 8N1
void set_timer_interrupt (unsigned char );      // HABILITA INTERRUPCI�N POR TMR0 Y PRECARGA EL REGISTRO CORRESPONDIENTE
void Ajustar_PWM (void);                        // CAMBIO DE CICLO DE TRABAJO MEDIANTE INTERRUPCI�N POR PULSADOR
void LED_BLINK (int, int);                      // PARPADEO DE LED PARA INDICACIONES VISUALES