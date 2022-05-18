/*
 * File:   main.c
 * Author: josej
 *
 * Created on May 11, 2022, 3:24 PM
 */

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

#include <xc.h>
#include <stdint.h>

/*------------------------------------------------------------------------------
 * CONSTANTES
 ------------------------------------------------------------------------------*/
#define _XTAL_FREQ 1000000
#define FLAG_SPI 0xFF
#define IN_MIN 0
#define IN_MAX 255
#define OUT_MIN 25
#define OUT_MAX 151
#define address_servo1_p1 1
#define address_servo2_p1 2
#define address_servo3_p1 3
#define address_servo4_p1 4
#define address_servo1_p2 5
#define address_servo2_p2 6
#define address_servo3_p2 7
#define address_servo4_p2 8

/*------------------------------------------------------------------------------
 * VARIABLES
 ------------------------------------------------------------------------------*/
uint8_t servo1 = 0;
uint8_t servo2 = 0;
uint8_t servo3 = 0;
uint8_t servo4 = 0;
unsigned short CCPR_servo1 = 0;
unsigned short CCPR_servo2 = 0;
unsigned short CCPR_servo3 = 0;
unsigned short CCPR_servo4 = 0;
uint8_t bandera_CCPR = 0;
uint8_t val_temp = 0;
uint8_t old_CCPR_servo1 = 0;
uint8_t old_CCPR_servo2 = 0;
uint8_t old_CCPR_servo3 = 0;
uint8_t old_CCPR_servo4 = 0;
uint8_t modo = 3;
uint8_t bandera_write_p1 = 0;
uint8_t bandera_write_p2 = 0;
uint8_t bandera_read_p1 = 0;
uint8_t bandera_read_p2 = 0;
uint8_t CCPR_temp = 0;
uint8_t serial_temp = 0;
uint8_t serial_temp2 = 0;
uint8_t serial_temp3 = 0;
uint8_t bandera_serial = 0;


/*------------------------------------------------------------------------------
 * PROTOTIPO DE FUNCIONES
 ------------------------------------------------------------------------------*/
void setup(void);
unsigned short map(uint8_t val, uint8_t in_min, uint8_t in_max,
            unsigned short out_min, unsigned short out_max);
uint8_t read_EEPROM(uint8_t address);
void write_EEPROM(uint8_t address, uint8_t data);


/*------------------------------------------------------------------------------
 * INTERRUPCIONES
 ------------------------------------------------------------------------------*/
void __interrupt() isr (void){
    if(PORTBbits.RB7){                  // ¿Es maestro?
        if(PIR1bits.ADIF){                      // Fue interrupción del ADC?
            if(ADCON0bits.CHS == 0){            // Verificamos sea AN0 el canal seleccionado
                CCPR_servo1 = map(ADRESH, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de ancho de pulso
            }
            else if(ADCON0bits.CHS == 1){            // Verificamos sea AN1 el canal seleccionado
                CCPR_servo2 = map(ADRESH, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de ancho de pulso

            }
            else if(ADCON0bits.CHS == 2){            // Verificamos sea AN1 el canal seleccionado
                CCPR_servo3 = map(ADRESH, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de ancho de pulso
                CCPR_servo3 = (CCPR_servo3 & 0b11111100) + 0b10;
            }
            else if(ADCON0bits.CHS == 3){            // Verificamos sea AN1 el canal seleccionado
                CCPR_servo4 = map(ADRESH, IN_MIN, IN_MAX, OUT_MIN, 0b1011000); // Valor de ancho de pulso
                CCPR_servo4 = (CCPR_servo4 & 0b11111100) + 0b11;
            }
            PIR1bits.ADIF = 0;                  // Limpiamos bandera de interrupción
        }

        else if(INTCONbits.RBIF){       // Fue interrupción del PORTB
            if(!PORTBbits.RB0){         // Verificamos si fue RB0 quien generó la interrupción

                if (modo == 1){
                    modo = 2;
                }
                else if (modo == 2){
                    modo = 3;
                }
                else if (modo == 3){
                    modo = 1;
                }
            }
            else if (!PORTBbits.RB1){    // Si es RB1

                if (modo == 3){
                    bandera_read_p1 = 1;
                }
                else{
                    bandera_write_p1 = 1;
                }
            }
            else if (!PORTBbits.RB2){   // Si es RB2
                if (modo == 3){
                    bandera_read_p2 = 1;
                }
                else{
                    bandera_write_p2 = 1;
                }
            }
            INTCONbits.RBIF = 0;        // Limpiamos bandera de interrupción
        }

        else if(PIR1bits.RCIF){              // Se recibió un dato?
            if (modo == 2){
                serial_temp = RCREG;                   // Valor que se recibió
                serial_temp3 = serial_temp;
            }
            else {
                serial_temp2 = RCREG;
            }
        }
    }

    else {                                          // ¿Es esclavo?
        if(PIR1bits.SSPIF){                         // ¿Recibió datos el esclavo?
            val_temp = SSPBUF;                      // Valor recibido se almacena en variable temporal
            bandera_CCPR = val_temp & 0b00000011;   // Verificamos qué servo mandó el valor
            val_temp = val_temp&0b11111100;         // Obtenemos los 6 bits más significativos

            if (bandera_CCPR == 0b00000010){        // Fue el servo 3
                CCPR1L = (uint8_t)(val_temp>>2);    // Guardamos los 6 bits mas significativos en CPR1L
                CCP1CONbits.DC1B = val_temp & 0b11; // Guardamos los 2 bits menos significativos en DC1B
            }

            else if(bandera_CCPR == 0b00000011){    // Fue el servo 4
                CCPR2L = (uint8_t)(val_temp>>2);    // Guardamos los 6 bits mas significativos en CPR1L
                CCP2CONbits.DC2B0 = val_temp & 0b1; // Guardamos los 2 bits menos significativos en DC1B
                CCP2CONbits.DC2B1 = val_temp>>1 & 0b1;
            }
        PIR1bits.SSPIF = 0;                         // Limpiamos bandera de interrupción
        }


    }
    return;
}

/*------------------------------------------------------------------------------
 * CICLO PRINCIPAL
 ------------------------------------------------------------------------------*/
void main(void) {
    setup();
    while(1){
        if (PORTBbits.RB7){                         // ¿Es maestro?
            if (modo == 1){                         // Modo de control con los potenciometros

                PORTE = 0b001;

                if(ADCON0bits.GO == 0){             // No hay proceso de conversion
                if(ADCON0bits.CHS == 0b0000)
                    ADCON0bits.CHS = 0b0001;        // Cambio de canal 0-1
                else if(ADCON0bits.CHS == 0b0001)
                    ADCON0bits.CHS = 0b0010;        // Cambio de canal 1-2
                else if(ADCON0bits.CHS == 0b0010)
                    ADCON0bits.CHS = 0b0011;        // Cambio de canal 2-3
                else if(ADCON0bits.CHS == 0b0011)
                    ADCON0bits.CHS = 0b0000;        // Cambio de canal 3-0
                __delay_us(40);                     // Tiempo de adquisicion

                ADCON0bits.GO = 1;                  // Iniciamos proceso de conversion
                }

                // Actualizar salida CCP solo si el potenciometro cambió
                if(CCPR_servo1 != old_CCPR_servo1){
                    CCPR1L = (uint8_t)(CCPR_servo1>>2);    // Guardamos los 8 bits mas significativos en CPR1L
                    CCP1CONbits.DC1B = CCPR_servo1 & 0b11; // Guardamos los 2 bits menos significativos en DC1B
                    old_CCPR_servo1 = CCPR_servo1;      // Actualizamos el valor del potenciometro
                }

                if(CCPR_servo2 != old_CCPR_servo2){
                    CCPR2L = (uint8_t)(CCPR_servo2>>2);    // Guardamos los 8 bits mas significativos en CPR2L
                    CCP2CONbits.DC2B0 = CCPR_servo2 & 0b1; // Guardamos los 2 bits menos significativos en DC2B
                    CCP2CONbits.DC2B1 = CCPR_servo2>>1 & 0b1;
                    old_CCPR_servo2 = CCPR_servo2;      // Actualizamos el valor del potenciometro
                }

                // Enviar valor de potenciometro solo si cambió
                if(CCPR_servo3 != old_CCPR_servo3){
                    SSPBUF = CCPR_servo3;               // Cargamos el valor del potenciometro al buffer
                    while(!SSPSTATbits.BF){}            // Esperamos a que termine el envío
                    old_CCPR_servo3 = CCPR_servo3;      // Actualizamos el valor del potenciometro
                }
                if(CCPR_servo4 != old_CCPR_servo4){
                    SSPBUF = CCPR_servo4;               // Cargamos el valor del potenciometro al buffer
                    while(!SSPSTATbits.BF){}            // Esperamos a que termine el envío
                    old_CCPR_servo4 = CCPR_servo4;      // Actualizamos el valor del potenciometro
                }
            }
            else if (modo == 2){                            // Modo de control con computadora

                PORTE = 0b010;

                bandera_serial = (serial_temp & 0x03);      // Verificamos qué servo mandó el valor
                
                if (bandera_serial == 0){                  // Fue el servo 1
                    serial_temp3 = serial_temp3&0b11111100;       // Obtenemos los 6 bits más significativos
                    CCPR_servo1 = map(serial_temp3, 0, 0b1100000, OUT_MIN, OUT_MAX);
                    CCPR1L = (uint8_t)(CCPR_servo1>>2);        // Guardamos los 6 bits mas significativos en CPR1L
                    CCP1CONbits.DC1B = CCPR_servo1 & 0b11;     // Guardamos los 2 bits menos significativos en DC1B
                }
                
                else if(bandera_serial == 1){                    // Fue el servo 2
                    serial_temp3 = serial_temp3&0b11111100;       // Obtenemos los 6 bits más significativos
                    CCPR_servo2 = map(serial_temp3, 0, 0b1100000, OUT_MIN, OUT_MAX);
                    CCPR2L = (uint8_t)(CCPR_servo2>>2);         // Guardamos los 6 bits mas significativos en CPR2L
                    CCP2CONbits.DC2B0 = CCPR_servo2 & 0b1;      // Guardamos los 2 bits menos significativos en DC2B
                    CCP2CONbits.DC2B1 = CCPR_servo2>>1 & 0b1;
                }
                
                else if (bandera_serial == 2){
                    CCPR_servo3 = (map(serial_temp3, 0, 0b1100011, OUT_MIN, OUT_MAX) & 0xFC) + 0b10;
                    __delay_ms(1);
                    SSPBUF = CCPR_servo3;                   // Cargamos el valor del potenciometro al buffer
                    while(!SSPSTATbits.BF){}                // Esperamos a que termine el envío
                }

                else if (bandera_serial == 3){
                    CCPR_servo4 = (map(serial_temp3, 0, 0b1100011, OUT_MIN, 0b1011000) & 0xFC) + 0b11;
                    SSPBUF = CCPR_servo4;                   // Cargamos el valor del potenciometro al buffer
                    while(!SSPSTATbits.BF){}                // Esperamos a que termine el envío
                }
  
            }

            else if (modo == 3){                            // Modo de control con EEPROM

                PORTE = 0b100;

                // Leer valores de la EEPROM
                if (bandera_read_p1 == 1){
                    CCPR_temp = read_EEPROM(address_servo1_p1);
                    CCPR1L = (uint8_t)(CCPR_temp>>2);    // Guardamos los 8 bits mas significativos en CPR1L
                    CCP1CONbits.DC1B = CCPR_temp & 0b11; // Guardamos los 2 bits menos significativos en DC1B
                    __delay_ms(1);
                    CCPR_temp = read_EEPROM(address_servo2_p1);
                    CCPR2L = (uint8_t)(CCPR_temp>>2);    // Guardamos los 8 bits mas significativos en CPR1L
                    CCP2CONbits.DC2B0 = CCPR_temp & 0b1; // Guardamos los 2 bits menos significativos en DC1B
                    CCP2CONbits.DC2B1 = CCPR_temp>>1 & 0b1;
                    __delay_ms(1);
                    CCPR_temp = read_EEPROM(address_servo3_p1);
                    SSPBUF = (CCPR_temp & 0b11111100) + 0b10;   // Cargamos el valor del potenciometro al buffer
                    while(!SSPSTATbits.BF){}                    // Esperamos a que termine el envío
                    __delay_ms(1);
                    CCPR_temp = read_EEPROM(address_servo4_p1);
                    SSPBUF = (CCPR_temp & 0b11111100) + 0b11;   // Cargamos el valor del potenciometro al buffer
                    while(!SSPSTATbits.BF){}                    // Esperamos a que termine el envío
                    __delay_ms(1);
                    bandera_read_p1 = 0;
                }

                if (bandera_read_p2 == 1){
                    CCPR_temp = read_EEPROM(address_servo1_p2);
                    CCPR1L = (uint8_t)(CCPR_temp>>2);    // Guardamos los 8 bits mas significativos en CPR1L
                    CCP1CONbits.DC1B = CCPR_temp & 0b11; // Guardamos los 2 bits menos significativos en DC1B
                    __delay_ms(1);
                    CCPR_temp = read_EEPROM(address_servo2_p2);
                    CCPR2L = (uint8_t)(CCPR_temp>>2);    // Guardamos los 8 bits mas significativos en CPR1L
                    CCP2CONbits.DC2B0 = CCPR_temp & 0b1; // Guardamos los 2 bits menos significativos en DC1B
                    CCP2CONbits.DC2B1 = CCPR_temp>>1 & 0b1;
                    __delay_ms(1);
                    CCPR_temp = read_EEPROM(address_servo3_p2);
                    SSPBUF = (CCPR_temp & 0b11111100) + 0b10;   // Cargamos el valor del potenciometro al buffer
                    while(!SSPSTATbits.BF){}                    // Esperamos a que termine el envío
                    __delay_ms(1);
                    CCPR_temp = read_EEPROM(address_servo4_p2);
                    SSPBUF = (CCPR_temp & 0b11111100) + 0b11;   // Cargamos el valor del potenciometro al buffer
                    while(!SSPSTATbits.BF){}                    // Esperamos a que termine el envío
                    __delay_ms(1);
                    bandera_read_p2 = 0;
                }
            }
            // Guardar posiciones de servos
            // Posicion 1
            if (bandera_write_p1 == 1){
                write_EEPROM(address_servo1_p1,CCPR_servo1);
                __delay_ms(10);
                write_EEPROM(address_servo2_p1,CCPR_servo2);
                __delay_ms(10);
                write_EEPROM(address_servo3_p1,CCPR_servo3);
                __delay_ms(10);
                write_EEPROM(address_servo4_p1,CCPR_servo4);
                __delay_ms(10);
                bandera_write_p1 = 0;
            }

            // Posicion 2
            if (bandera_write_p2 == 1){
                write_EEPROM(address_servo1_p2,CCPR_servo1);
                __delay_ms(10);
                write_EEPROM(address_servo2_p2,CCPR_servo2);
                __delay_ms(10);
                write_EEPROM(address_servo3_p2,CCPR_servo3);
                __delay_ms(10);
                write_EEPROM(address_servo4_p2,CCPR_servo4);
                __delay_ms(10);
                bandera_write_p2 = 0;
            }
        }
    }
    return;
}

/*------------------------------------------------------------------------------
 * CONFIGURACION
 ------------------------------------------------------------------------------*/
void setup(void){

    OSCCONbits.IRCF = 0b100;    // 1MHz
    OSCCONbits.SCS = 1;         // Reloj interno

    TRISD = 0x00;
    PORTD = 0;

    // Configuración de SPI
    // Configs de Maestro
    if(PORTBbits.RB7){
        ANSELH = 0;
        ANSEL = 0b00001111;         // AN0, AN1, AN2 y AN3 como entradas analogicas

        //TRISC = 0b00010000;         // -> SDI entrada, SCK y SD0 como salida
        //PORTC = 0;

        TRISCbits.TRISC3 = 0;
        TRISCbits.TRISC4 = 1;
        TRISCbits.TRISC5 = 0;

        TRISA = 0b00001111;         // RA0, RA1, RA2 y RA3 como entradas
        PORTA = 0;

        TRISB = 0xFF;               // Puerto B como entrada
        OPTION_REGbits.nRBPU = 0;   // Habilitamos resistencias pull-up Puerto B
        WPUB = 0x07;                // Pull-up en RB0, RB1 y RB2

        TRISE = 0x00;               // Puerto E como salida
        PORTE = 0b100;

        // SSPCON <5:0>
        SSPCONbits.SSPM = 0b0000;   // -> SPI Maestro, Reloj -> Fosc/4 (250kbits/s)
        SSPCONbits.CKP = 0;         // -> Reloj inactivo en 0
        SSPCONbits.SSPEN = 1;       // -> Habilitamos pines de SPI
        // SSPSTAT<7:6>
        SSPSTATbits.CKE = 1;        // -> Dato enviado cada flanco de subida
        SSPSTATbits.SMP = 1;        // -> Dato al final del pulso de reloj
        SSPBUF = servo1;            // Enviamos un dato inicial

        // Configuraciones ADC
        ADCON0bits.ADCS = 0b00;     // FOSC/2
        ADCON1bits.VCFG0 = 0;       // VDD
        ADCON1bits.VCFG1 = 0;       // VSS
        ADCON0bits.CHS = 0b0000;    // AN0
        ADCON1bits.ADFM = 0;        // Justificado a la izquierda
        ADCON0bits.ADON = 1;        // Habilitamos modulo ADC
        __delay_us(40);             // Sample time

        // Configuración CCP
        // Configuración PWM
        PR2 = 62;                  // Periodo de 4ms

        // CCP1 para SERVO 1
        TRISCbits.TRISC2 = 1;       // Deshabilitamos salida de CCP1
        CCP1CON = 0;                // Apagamos CCP1
        CCP1CONbits.P1M = 0;        // Modo single output
        CCP1CONbits.CCP1M = 0b1100; // PWM
        CCPR1L = 25>>2;              // Valor mínimo
        CCP1CONbits.DC1B = 25 & 0b11;
        // CCP2 para SERVO 2
        TRISCbits.TRISC1 = 1;       // Deshabilitamos salida de CCP2
        CCP2CON = 0;                // Apagamos CCP2
        CCP2CONbits.CCP2M = 0b1100; // PWM
        CCPR2L = 25>>2;              // Valor mínimo
        CCP2CONbits.DC2B0 = 25 & 0b1;
        CCP2CONbits.DC2B1 = 25>>1 & 0b1;

        // Configuracion del TMR2
        PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion del TMR2
        T2CONbits.T2CKPS = 0b11;    // prescaler 1:16
        T2CONbits.TMR2ON = 1;       // Encendemos TMR2
        while(!PIR1bits.TMR2IF);    // Esperar un cliclo del TMR2
        PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion del TMR2 nuevamente

        TRISCbits.TRISC2 = 0;       // Habilitamos salida de PWM
        TRISCbits.TRISC1 = 0;       // Habilitamos salida de PWM

        // Confiruación comunicación serial
        // SYNC = 0,    BRGH = 1,   BRF16 = 1,  SPBRG = 25
        TXSTAbits.SYNC = 0;         // Asincrónica
        TXSTAbits.BRGH = 1;         // Baud rate de alta velocidad
        BAUDCTLbits.BRG16 = 1;      // 16 bits para baud rate

        SPBRG = 25;
        SPBRGH = 0;                 // Baud rate 9600

        RCSTAbits.SPEN = 1;         // Habilitamos comuniación
        TXSTAbits.TX9 = 0;          // Solo se utilizan 8 bits
        TXSTAbits.TXEN = 1;         // Habilitamos transmisor
        RCSTAbits.CREN = 1;         // Habilitamos receptor


        // Configuracion de interrupciones
        INTCONbits.PEIE = 1;
        INTCONbits.GIE = 1;
        PIE1bits.RCIE = 1;          // Habilitamos interrupciones de recepción
        INTCONbits.RBIE = 1;        // Habilitamos interrupciones del PORTB
        IOCB = 0x07;                // Habilitamos interrupción por cambio de estado para RB0, RB1 y RB2
        INTCONbits.RBIF = 0;        // Limpiamos bandera de interrupción
        PIR1bits.ADIF = 0;          // Limpiamos bandera ADC
        PIE1bits.ADIE = 1;          // Habilitamos interrupcion de ADC


    }
    // Configs del esclavo
    else{
        ANSEL = 0;
        ANSELH = 0;

        TRISC = 0b00011000;         // -> SDI y SCK entradas, SD0 como salida
        PORTC = 0;

        TRISA = 0b00100000;         // SS como entrada
        PORTA = 0;

        // SSPCON <5:0>
        SSPCONbits.SSPM = 0b0100;   // -> SPI Esclavo, SS hablitado
        SSPCONbits.CKP = 0;         // -> Reloj inactivo en 0
        SSPCONbits.SSPEN = 1;       // -> Habilitamos pines de SPI
        // SSPSTAT<7:6>
        SSPSTATbits.CKE = 1;        // -> Dato enviado cada flanco de subida
        SSPSTATbits.SMP = 0;        // -> Dato al final del pulso de reloj

        // Configuración CCP
        // Configuración PWM
        PR2 = 62;                  // Periodo de 4ms

        // CCP1 para SERVO 1
        TRISCbits.TRISC2 = 1;       // Deshabilitamos salida de CCP1
        CCP1CON = 0;                // Apagamos CCP1
        CCP1CONbits.P1M = 0;        // Modo single output
        CCP1CONbits.CCP1M = 0b1100; // PWM
        CCPR1L = 25>>2;              // Valor mínimo
        CCP1CONbits.DC1B = 25 & 0b11;
        // CCP2 para SERVO 2
        TRISCbits.TRISC1 = 1;       // Deshabilitamos salida de CCP2
        CCP2CON = 0;                // Apagamos CCP2
        CCP2CONbits.CCP2M = 0b1100; // PWM
        CCPR2L = 25>>2;              // Valor mínimo
        CCP2CONbits.DC2B0 = 25 & 0b1;
        CCP2CONbits.DC2B1 = 25>>1 & 0b1;

            // Configuracion del TMR2
        PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion del TMR2
        T2CONbits.T2CKPS = 0b11;    // prescaler 1:16
        T2CONbits.TMR2ON = 1;       // Encendemos TMR2
        while(!PIR1bits.TMR2IF);    // Esperar un cliclo del TMR2
        PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion del TMR2 nuevamente

        TRISCbits.TRISC2 = 0;       // Habilitamos salida de PWM
        TRISCbits.TRISC1 = 0;       // Habilitamos salida de PWM

        // Interrupciones
        PIR1bits.SSPIF = 0;         // Limpiamos bandera de SPI
        PIE1bits.SSPIE = 1;         // Habilitamos int. de SPI
        INTCONbits.PEIE = 1;
        INTCONbits.GIE = 1;

    }
}

/*------------------------------------------------------------------------------
 * FUNCIONES
 ------------------------------------------------------------------------------*/
unsigned short map(uint8_t x, uint8_t x0, uint8_t x1,
            unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}

uint8_t read_EEPROM(uint8_t address){
    EEADR = address;
    EECON1bits.EEPGD = 0;       // Lectura a la EEPROM
    EECON1bits.RD = 1;          // Obtenemos dato de la EEPROM
    return EEDAT;               // Regresamos dato
}

void write_EEPROM(uint8_t address, uint8_t data){
    EEADR = address;
    EEDAT = data;
    EECON1bits.EEPGD = 0;       // Escritura a la EEPROM
    EECON1bits.WREN = 1;        // Habilitamos escritura en la EEPROM

    INTCONbits.GIE = 0;         // Deshabilitamos interrupciones
    EECON2 = 0x55;
    EECON2 = 0xAA;

    EECON1bits.WR = 1;          // Iniciamos escritura

    EECON1bits.WREN = 0;        // Deshabilitamos escritura en la EEPROM
    INTCONbits.RBIF = 0;
    INTCONbits.GIE = 1;         // Habilitamos interrupciones

}