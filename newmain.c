/*
 * File:  P_maestro.c
 * Autores: Eduardo Rubin       #20291
 *          Carolina Paz        #20719
 *          Juan Emilio Reyes   #20959
 *
 * Creado el 24 de mayo de 2022, 10:20
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

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>

/*------------------------------------------------------------------------------
 * CONSTANTES
 ------------------------------------------------------------------------------*/
#define _XTAL_FREQ 1000000           // Frecuencia del oscilador
#define FLAG_SPI   0x52              // Variable para generar pulsos
#define BOTON      PORTBbits.RB0     // Asignamos un alias a RB0
#define BOTON2     PORTBbits.RB1     // Boton que va a decrementar
#define BOTON3     PORTBbits.RB2     // Asignamos un alias a RB0
#define LEN_MSG 9

/*------------------------------------------------------------------------------
 * VARIABLES
 ------------------------------------------------------------------------------*/
char val_temporal = 0;                //Valor de comparacion para recepcion de datos
char valor_ADC = 0;                   //Para guardar el valor del ADC
char mode[3] = {0x01, 0x02, 0x04};    //Para cambiar de modo
int  m = 0, POT1, POT2, POT3, POT4;
uint8_t bandera_write = 0;             // Variable para leer
uint8_t bandera_read = 0;              // Variable para escribir
uint8_t POT1_temp = 0;
int not_use = 0;

char mensaje[LEN_MSG] = {'D', 'a', 't', 'o', ':', ' ', ' ', 0x0D, 0x0A};
uint8_t indice = 0;             // Variable para saber que posición del mensaje enviar al serial
uint8_t valor_old = 0;          // Variable para guardar el valor anterior recibido
int pot_flag = 0;
char POT1_P[4] = {0x0, 0x3F, 0x1F, 0x9C};

/*------------------------------------------------------------------------------
 * PROTOTIPO DE FUNCIONES
 ------------------------------------------------------------------------------*/
void setup(void);
uint8_t read_EEPROM(uint8_t address);
void write_EEPROM(uint8_t address, uint8_t data);
void send_data(void);
unsigned short map(uint8_t val, uint8_t in_min, uint8_t in_max, unsigned short out_min, unsigned short out_max);


/*------------------------------------------------------------------------------
 * INTERRUPCIONES
 ------------------------------------------------------------------------------*/
void __interrupt() isr (void){

    if(PIR1bits.ADIF){                            // Verificar interrupcion de ADC
        if(ADCON0bits.CHS == 0){
            POT1 = ADRESH;
            CCPR1L = (ADRESH>>3) + 7 ;}           // Que tenga el mismo valor del registro

        else if (ADCON0bits.CHS == 1){
            POT2 = ADRESH;
            CCPR2L = (ADRESH>>3) + 7 ;}           // Que tenga el mismo valor del registro. El corrimiento lo divide en 2

        else if (ADCON0bits.CHS == 2){
            POT3 = ADRESH;
        }

        else if (ADCON0bits.CHS == 3){
            POT4 = ADRESH;
        }            // Que tenga el mismo valor del registro. El corrimiento lo divide en 2

        PIR1bits.ADIF = 0;          // Limpiar bandera para volver a empezar
    }

    else if(INTCONbits.RBIF){            // Fue interrupción del PORTB
        if(!BOTON){                 // Verificamos si fue RB0 quien generó la interrupción
            m++;
            if(m>2){m=0;}
            PORTE = mode[m];}       // Incremento del PORTC (INCF PORTC)

        INTCONbits.RBIF = 0;    // Limpiamos bandera de interrupción
    }
    
    else if(PIR1bits.RCIF){          // Hay datos recibidos?
        
        if (PORTE == 4){
        mensaje[6] = RCREG;     // Guardamos valor recibido en el arreglo mensaje
        pot_flag = 0xC0 & mensaje[6];
        
        PORTD = mensaje[6] & 0x3F;
        
        if(pot_flag == 0x0){
            
        POT1 = mensaje[6]&0b00111111;
        CCPR1L = map(POT1, 0, 63, 7, 39);}
            
            
        else if(pot_flag == 0x40){
            
        POT2 = mensaje[6]&0b00111111;
        CCPR2L = map(POT2, 0, 63, 7, 39);}
        
        else if(pot_flag == 0x80){
             
        POT3 = mensaje[6];
        PORTAbits.RA6 = 1;                      // Deshabilitamos el ss del esclavo
        __delay_ms(10);                         // Esperamos un tiempo para que el PIC pueda detectar el cambio en el pin
        PORTAbits.RA6 = 0;                      // habilitamos nuevamente el escalvo
        __delay_ms(10);

        //Enviar el valor del potenciometro

        SSPBUF = POT3;                          // Cargamos valor del contador al buffer
        while(!SSPSTATbits.BF){}                // Esperamos a que termine el envio
        PORTAbits.RA6 = 1;                      // Deshabilitar el ss del PIC esclavo 2
        __delay_ms(10);}
        
        else if(pot_flag == 0xC0){
        POT4 = mensaje[6];
        PORTAbits.RA6 = 1;                      // Deshabilitamos el ss del esclavo
        __delay_ms(10);                         // Esperamos un tiempo para que el PIC pueda detectar el cambio en el pin
        PORTAbits.RA6 = 0;                      // habilitamos nuevamente el escalvo
        __delay_ms(10);

        //Enviar el valor del potenciometro

        SSPBUF = POT4;                          // Cargamos valor del contador al buffer
        while(!SSPSTATbits.BF){}                // Esperamos a que termine el envio
        PORTAbits.RA6 = 1;                      // Deshabilitar el ss del PIC esclavo 2
        __delay_ms(10);}
        }
        
        else{
            not_use = RCREG;
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

    if(PORTE == 1){
    if(ADCON0bits.GO == 0){                 // Cambio de canales analogicos
           if (ADCON0bits.CHS == 0){
               ADCON0bits.CHS = 1;}         // Cambio de canal a AN5

           else if (ADCON0bits.CHS == 1){
               ADCON0bits.CHS = 2;}

           else if (ADCON0bits.CHS == 2){
               ADCON0bits.CHS = 3;}

           else{ADCON0bits.CHS = 0;}        // Cambio de canal a AN6
           __delay_us(50);

           ADCON0bits.GO = 1;}

    send_data();

    //Enviar el valor del potenciometro

    if(!BOTON2){                 // Verificamos si fue RB0 quien generó la interrupción
      write_EEPROM(0, POT1);     // ESTA Y LA DE ABAJO SI FUNCIONAN BIEN
      __delay_ms(1);
      write_EEPROM(1, POT2);
      __delay_ms(1);
      write_EEPROM(2, POT3);
      __delay_ms(1);
      write_EEPROM(3, POT4);
      __delay_ms(1);}

    if(!BOTON3){
      write_EEPROM(4, POT1);
      __delay_ms(1);
      write_EEPROM(5, POT2);
      __delay_ms(1);
      write_EEPROM(6, POT3);
      __delay_ms(1);
      write_EEPROM(7, POT4);
      __delay_ms(1);}
    }                                        // Finaliza condicion del modo
  if(PORTE == 2){
    if(!BOTON2){                 // Verificamos si fue RB0 quien generó la interrupción
      POT1 = read_EEPROM(0);
      __delay_ms(1);
      POT2 = read_EEPROM(1);
      __delay_ms(1);
      POT3 = read_EEPROM(2);
      __delay_ms(1);
      POT4 = read_EEPROM(3);
      __delay_ms(1);
      
      CCPR1L = (POT1>>3) + 7 ;           // Que tenga el mismo valor del registro
      CCPR2L = (POT2>>3) + 7 ;
      
      send_data();
    }                                           // Finaliza boton
    if(!BOTON3){                 // Verificamos si fue RB0 quien generó la interrupción
        POT1 = read_EEPROM(4);
        __delay_ms(1);
        POT2 = read_EEPROM(5);
        __delay_ms(1);
        POT3 = read_EEPROM(6);
        __delay_ms(1);
        POT4 = read_EEPROM(7);
        __delay_ms(1);

        CCPR1L = (POT1>>3) + 7 ;           // Que tenga el mismo valor del registro
        CCPR2L = (POT2>>3) + 7 ;

        send_data();
    }                                           // Finaliza boton
  }                                             // Finaliza modo
   if(PORTE == 4){ 
        
       indice = 0;                             // Reiniciamos indice para enviar todo el mensaje
       if (valor_old != mensaje[6]){           // Verificamos que el nuevo valor recibido en el serial 
                                                //   sea diferente al anterior, para imprimir solo 
            while(indice<LEN_MSG){              // Loop para imprimir el mensaje completo
                if (PIR1bits.TXIF){             // Esperamos a que esté libre el TXREG para poder enviar por el serial
                    TXREG = mensaje[indice];    // Cargamos caracter a enviar
                    indice++;                   // Incrementamos indice para enviar sigiente caracter
                }
            }
            valor_old = mensaje[6];             // Guardamos valor recibido para comparar en siguiente iteración
        
            }        
        }   
    }                                              // Finaliza ciclo del while
    return;
}                                               // Finaliza el main

/*------------------------------------------------------------------------------
 * CONFIGURACION
 ------------------------------------------------------------------------------*/
void setup(void){

    // Configuración de entradas y salidas
    ANSEL = 0b00001111;         // Entradas para los Potenciometros
    ANSELH = 0;                 // Las demas entradas AN no se utilizaran

    TRISA = 0x1F;               // Puerto A del maestro conectados a POTS
    PORTA = 0;                  // Limpiar el resto de bits

    TRISB = 0xFF;               // Puerto B como entradas para los botones

    TRISD = 0;                  // Puerto D como salidas para mostrar alguna variable
    PORTD = 0;                  // Limpiar el puerto

    TRISE = 0;                  // Puerto E como salidas para mostrar el modo
    PORTE = 0;
    PORTE = mode[0];

    //Configuracion oscilador
    OSCCONbits.IRCF = 0b100;    // 1MHz
    OSCCONbits.SCS = 1;         // Reloj interno

    //Configuracion modulo ADC
    ADCON1bits.ADFM  = 0;       // Justificado a la izquierda
    ADCON1bits.VCFG0 = 0;       // VDD tension de alimentacion
    ADCON1bits.VCFG1 = 0;       // VSS tierra

    ADCON0bits.ADCS  = 0b01;    // FOSC/8
    ADCON0bits.CHS   = 0;       // Canal inicial que se utilizaran
    __delay_us(50);
    ADCON0bits.ADON  = 1;       // Habilitar el modulo ADC
    __delay_us(50);

    PIR1bits.ADIF    = 0;       // Limpiar bandera de ADC
    PIE1bits.ADIE    = 1;       // Habilitar interrupcion

    //Configuracion comunicacion maestro SPI
    //TRISC = 0b00010000;         // -> SDI entrada, SCK y SD0 como salida
    TRISCbits.TRISC4 = 1;   
    TRISCbits.TRISC3 = 0;
    TRISCbits.TRISC5 = 0;

    PORTC = 0;

    // SSPCON <5:0>
    SSPCONbits.SSPM = 0b0000;       // SPI Maestro, Reloj -> Fosc/4 (250kbits/s)
    SSPCONbits.CKP = 0;             // Reloj inactivo en 0
    SSPCONbits.SSPEN = 1;           // Habilitamos pines de SPI

    // SSPSTAT<7:6>
    SSPSTATbits.CKE = 1;            // Dato enviado cada flanco de subida
    SSPSTATbits.SMP = 1;            // Dato al final del pulso de reloj

    //Interrupcion del puerto B
    OPTION_REGbits.nRBPU = 0;       // Habilitamos resistencias de pull-up del PORTB

    WPUBbits.WPUB0  = 1;            // Resistencia de pull-up de RB0
    WPUBbits.WPUB1  = 1;            // Resistencia en el RB1 para POT1
    WPUBbits.WPUB2  = 1;            // Resistencia en el RB2 para POT2

    INTCONbits.RBIE = 1;            // Habilitamos interrupciones del PORTB
    IOCBbits.IOCB0  = 1;            // Habilitamos interrupción por cambio de estado para RB0
    IOCBbits.IOCB1  = 1;            // Habilitar para RB1
    IOCBbits.IOCB2  = 1;            // Habilitar para RB2
    INTCONbits.RBIF = 0;            // Limpiamos bandera de interrupción

    //Configuracion del PWM
    TRISCbits.TRISC1   = 1;         // RC1 como entrada
    TRISCbits.TRISC2   = 1;         // RC2 como entrada

    PR2 = 62;                       // Configurar el periodo
    CCP1CONbits.P1M    = 0;
    CCP1CONbits.CCP1M  = 0b1100;    // Habilitar CCP1 en modo PWM
    CCP2CONbits.CCP2M  = 0b1100;    // Habilitar CCP2 en modo PWM

    CCPR1L = 0x0f;                  // ciclo de trabajo
    CCP1CONbits.DC1B   = 0;
    PIR1bits.TMR2IF    = 0;         // apagar la bandera de interrupcion
    T2CONbits.T2CKPS   = 0b11;      // prescaler de 1:16
    T2CONbits.TMR2ON   = 1;         // prescaler de 1:16

    while(PIR1bits.TMR2IF == 0);    // esperar un ciclo del TMR2
    PIR1bits.TMR2IF    = 0;

    TRISCbits.TRISC2   = 0;         // Salida del PWM2
    TRISCbits.TRISC1   = 0;         // Salida del PWM1
       
    // Configuraciones de comunicacion serial
    //SYNC = 0, BRGH = 1, BRG16 = 1, SPBRG=25 <- Valores de tabla 12-5
    TXSTAbits.SYNC = 0;         // Comunicación ascincrona (full-duplex)
    TXSTAbits.BRGH = 1;         // Baud rate de alta velocidad 
    BAUDCTLbits.BRG16 = 1;      // 16-bits para generar el baud rate
    
    SPBRG = 25;
    SPBRGH = 0;                 // Baud rate ~9600, error -> 0.16%
    
    RCSTAbits.SPEN = 1;         // Habilitamos comunicación
    TXSTAbits.TX9 = 0;          // Utilizamos solo 8 bits
    TXSTAbits.TXEN = 1;         // Habilitamos transmisor
    RCSTAbits.CREN = 1;         // Habilitamos receptor
    
    // Configuraciones de interrupciones
    INTCONbits.GIE = 1;         // Habilitamos interrupciones globales
    INTCONbits.PEIE = 1;        // Habilitamos interrupciones de perifericos
    PIE1bits.RCIE = 1;          // Habilitamos Interrupciones de recepción
    return;
}

// Leer en la EEPROM
uint8_t read_EEPROM(uint8_t address){
    EEADR = address;                // Guardamos el address en EEADR
    EECON1bits.EEPGD = 0;           // Lectura a la EEPROM
    EECON1bits.RD = 1;              // Obtenemos dato de la EEPROM
    return EEDAT;                   // Regresamos dato
}

// Escribir en la EEPROM
void write_EEPROM(uint8_t address, uint8_t data){
    EEADR = address;                // Guardamos el address en EEADR
    EEDAT = data;                   // Guardar el dato que queremos mandar en EEDAT
    EECON1bits.EEPGD = 0;           // Escritura a la EEPROM
    EECON1bits.WREN = 1;            // Habilitamos escritura en la EEPROM

    INTCONbits.GIE = 0;             // Deshabilitamos interrupciones para escribir
    EECON2 = 0x55;                  // Registro de control
    EECON2 = 0xAA;                  // Registro de control

    EECON1bits.WR = 1;              // Iniciamos escritura

    EECON1bits.WREN = 0;            // Deshabilitamos escritura en la EEPROM
    INTCONbits.RBIF = 0;            // Limpiar bandera de interrupcion
    INTCONbits.GIE = 1;             // Habilitamos interrupciones
}

void send_data(void){
    PORTAbits.RA6 = 1;                      // Deshabilitamos el ss del esclavo
    __delay_ms(10);                         // Esperamos un tiempo para que el PIC pueda detectar el cambio en el pin
    PORTAbits.RA6 = 0;                      // habilitamos nuevamente el escalvo
    __delay_ms(10);

    //Enviar el valor del potenciometro

    SSPBUF = POT3;                          // Cargamos valor del contador al buffer
    while(!SSPSTATbits.BF){}                // Esperamos a que termine el envio
    PORTAbits.RA6 = 1;                      // Deshabilitar el ss del PIC esclavo 2
    __delay_ms(10);

    // Preparar envio del POT4 de maestro a esclavo

    PORTAbits.RA6 = 1;                      // Deshabilitamos el ss del esclavo
    __delay_ms(10);                         // Esperamos un tiempo para que el PIC pueda detectar el cambio en el pin
    PORTAbits.RA6 = 0;                      // habilitamos nuevamente el escalvo
    __delay_ms(10);

    //Enviar el valor del potenciometro

    SSPBUF = POT4;                          // Cargamos valor del contador al buffer
    while(!SSPSTATbits.BF){}                // Esperamos a que termine el envio
    PORTAbits.RA6 = 1;                      // Deshabilitar el ss del PIC esclavo 2
    __delay_ms(10);
}

unsigned short map(uint8_t x, uint8_t x0, uint8_t x1, unsigned short y0, unsigned short y1){
 return 
         (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}