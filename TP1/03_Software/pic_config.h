// PIC12F629 Configuration Bit Settings

#pragma config FOSC = INTRCIO   // Selección oscilador interno
#pragma config WDTE = ON        // Watchdog deshabilitado
#pragma config PWRTE = ON       // Power-Up Timer habilitado
#pragma config MCLRE = OFF      // MCLR pin deshabilitado
#pragma config BOREN = ON       // Brown-out Detect habilitado (UVLO)
#pragma config CP = OFF         // Code Protection deshabilitado
#pragma config CPD = OFF        // Data Code Protection deshabilitado (protección de la memoria EEPROM para lectura con HW programador)

