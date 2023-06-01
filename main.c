#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF         // Low Voltage Programming Enable bit (RB3/PGM pin has PGM function, low voltage programming enabled)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

#include <xc.h>
#include <stdint.h>
#include <stdlib.h>

#define _XTAL_FREQ 8000000 //__delay_ms(x)
 uint8_t cont;  
 uint8_t duty0 = 0; 
 uint8_t duty1 = 0; 
 uint8_t valservo1;
 uint8_t valservo2;
 uint8_t valservo3;
 uint8_t valservo4;
 
 
int modos; 
int map(int input, int in_min, int in_max, int out_min, int out_max);
int map(int input, int in_min, int in_max, int out_min, int out_max) {
    return ((input - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
} 

uint8_t ind = 0;
unsigned char  serialdata[2];
unsigned char  indice;
unsigned char value;
int bandera = 0b0000;
int bandera2 = 0;
uint8_t read_EEPROM(uint8_t ADRESS);
void write_EEPROM(uint8_t ADRESS, uint8_t data);


void __interrupt()isr(void){
    
   if(INTCONbits.RBIF){
        
       
        if (PORTBbits.RB7 == 0 && bandera == 0b0010){
            if(!PORTBbits.RB7);
            CCPR1L = read_EEPROM(0x00);
            CCPR2L = read_EEPROM(0x01);
           

        }
          else if(!PORTBbits.RB6){
            if(!PORTBbits.RB6);
              write_EEPROM(0x01, (valservo2));
              write_EEPROM(0x00, (valservo1));
             
              

         }
      
        INTCONbits.RBIF = 0;
    }
   
    if (PIR1bits.RCIF)
    {
        PIR1bits.RCIF = 0;
        if(ind == 0){
            serialdata[0] = RCREG;
            indice = serialdata[0];
        }
        else if(ind == 1){
            serialdata[1] = RCREG;
            value = serialdata[1];

        }
        ind ++;
        if(ind == 2){
             ind = 0;
        }
        if(bandera == 0b0011){
            if(indice == 1){
                CCPR1L = value;
             
            }
            else if(indice == 2){
                CCPR2L = value;
            }  
      
            else if(indice == 3){
                 duty0 = value;
            }  
            else if(indice == 4){ 
                 duty1 = value;
            } 
            }
        
    }
    
    //MODO MANUAL
    if (INTCONbits.TMR0IF)  // Verificar si la interrupción del TMR0 se ha activado
    {
        INTCONbits.TMR0IF = 0;  
        cont++;
        if (cont < duty0){
              PORTCbits.RC3 = 1;
        }
        if(cont >= duty0){
              PORTCbits.RC3 = 0;
            }
        if (cont < duty1){
              PORTCbits.RC4 = 1;
        }
        if (cont > duty1){
              PORTCbits.RC4 = 0;
        }
        if(cont >= 100){
            cont = 0; 
        }

        TMR0 = 206; 
    }   
    //MODO MANUAL
     
        if(PIR1bits.ADIF == 1){
            
            PIR1bits.ADIF = 0;
            if(bandera == 0b0001){
            if(ADCON0bits.CHS == 0b0000){
                CCPR1L = (ADRESH >> 1) + 124;
                valservo1 = (ADRESH >> 1) + 124;
            }
            else if(ADCON0bits.CHS == 0b0001){
                CCPR2L = (ADRESH >> 1) + 124;
                valservo2 = (ADRESH >> 1) + 124;
            }  
            
            else if(ADCON0bits.CHS == 0b0010){
                 duty0 = map(ADRESH,0,255,1,10);
                 valservo3 = duty0;
                 
            }  
            else if(ADCON0bits.CHS == 0b0011){ 
                 duty1 = map(ADRESH,0,255,1,10);
                 valservo4 = duty1;
            } 
            }
        }
   
   switch(indice){
 
            case(5):
                bandera = 0b0001;
                PORTD = 1;
                break;  
            case(6):
                bandera = 0b0010;
                PORTD = 2;
                break;   
            case(7):
                 bandera = 0b0011;
                 PORTD = 3;
                break;   
            case(8):
                bandera = 0b0100;
                PORTD = 4;
                break; 
        } 
}


void main(void){
   
    ANSEL = 0b00001111;
    ANSELH =0;
    TRISA = 255;
    
    TRISBbits.TRISB5 = 1;
    TRISBbits.TRISB6 = 1;
    TRISBbits.TRISB7 = 1;
    
    
    TRISCbits.TRISC1 = 0;
    TRISCbits.TRISC2 = 0;
    TRISCbits.TRISC3 = 0;
    TRISCbits.TRISC4 = 0;
    TRISCbits.TRISC6 = 1;
    TRISCbits.TRISC7 = 1;
    
    TRISD = 0;
    PORTD = 0;
   
//         //Confi. ADC
    ADCON1bits.ADFM = 0;    //Justificado a la izquierda
    ADCON1bits.VCFG0 = 0;   //voltaje de 0V-5V
    ADCON1bits.VCFG1 = 0;
    ADCON0bits.ADCS = 0b10; //Fosc/32
    ADCON0bits.CHS = 0b0000;    
    __delay_us(50);
    ADCON0bits.ADON = 1;    //activo el modulo
    
    
    PR2 = 255;              //Valor de pwm
    
    CCP1CONbits.P1M = 0;    //PWM mode
    CCP1CONbits.CCP1M = 0b1100; 
    CCPR1L = 0x0f;          //inicio de ciclo de trabajo
    //
    CCP2CONbits.CCP2M = 0;    //PWM mode
    CCP2CONbits.CCP2M = 0b1100; 
    CCPR1L = 0x0f;          //inicio de ciclo de trabajo
    CCPR2L = 0x0f;          //inicio de ciclo de trabajo
    
    CCP1CONbits.DC1B = 0;
    CCP2CONbits.DC2B0 = 0;
    CCP2CONbits.DC2B1 = 0;
    //
    PIR1bits.TMR2IF = 0;     //bajo la bandera
    T2CONbits.T2CKPS = 0b11; //pre-escaler 1:16
    T2CONbits.TMR2ON = 1;    //Enciendo el timmer 2
    while(PIR1bits.TMR2IF == 0);    //espero a completar el un ciclo tmr2
    PIR1bits.TMR2IF = 0;
    TRISCbits.TRISC2 = 0;           // Salida PWM
    TRISCbits.TRISC5 = 0;           // Salida PWM
//    
    OSCCONbits.IRCF = 0b111;  // config. de oscilador
    OSCCONbits.SCS = 1;         //reloj interno
    
    
                            //confi. interrupciones
    PIR1bits.ADIF = 0;
    PIE1bits.ADIE = 1; 
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;
    ADCON0bits.GO = 1;  
    INTCONbits.TMR0IE = 1;
    INTCONbits.TMR0IF = 0;
    PIR1bits.RCIF = 0;
    PIE1bits.RCIE = 1;
    IOCBbits.IOCB5 = 1;
    IOCBbits.IOCB6 = 1;
    IOCBbits.IOCB7 = 1;
        
    
   // CONFIGURACIÓN DE PULL UPS
    OPTION_REGbits.nRBPU = 0;
    WPUBbits.WPUB5 = 1;
    WPUBbits.WPUB6 = 1;
    WPUBbits.WPUB7 = 1;
    
  
    //config. TMR0
    //Timer0 Registers Prescaler= 4 - TMR0 Preset = 206 - Freq = 10000.00 Hz - Period = 0.000100 seconds
    OPTION_REGbits.T0CS = 0;  // bit 5  TMR0 Clock Source Select bit...0 = Internal Clock (CLKO) 1 = Transition on T0CKI pin
    OPTION_REGbits.T0SE = 0;  // bit 4 TMR0 Source Edge Select bit 0 = low/high 1 = high/low
    OPTION_REGbits.PSA = 0;   // bit 3  Prescaler Assignment bit...0 = Prescaler is assigned to the Timer0
    OPTION_REGbits.PS2 = 0;   // bits 2-0  PS2:PS0: Prescaler Rate Select bits
    OPTION_REGbits.PS1 = 1;
    OPTION_REGbits.PS0 = 0;
    TMR0 = 206;             // preset for timer register
    
    TXSTAbits.SYNC = 0;
    TXSTAbits.BRGH = 1;
    
    BAUDCTLbits.BRG16 = 1;
    
    SPBRG = 207;
    SPBRGH =0;
    
    RCSTAbits.SPEN = 1;
    RCSTAbits.RX9 = 0;
    RCSTAbits.CREN = 1;

    //------------------------------loop principal----------------------------------
    while (1) {

        if(ADCON0bits.GO == 0){
            
            if(ADCON0bits.CHS == 0b0000){
                ADCON0bits.CHS = 0b0001;
            }
            else if(ADCON0bits.CHS == 0b0001){
                ADCON0bits.CHS = 0b0010;
            }
            else if(ADCON0bits.CHS == 0b0010){
                ADCON0bits.CHS = 0b0011;
            }
            else if(ADCON0bits.CHS == 0b0011){
                ADCON0bits.CHS = 0b0000;
            }
            
            __delay_us(50);    
            ADCON0bits.GO = 1;
        }
    }
    return;     
}


uint8_t read_EEPROM(uint8_t ADRESS){
    EEADR = ADRESS;
    EECON1bits.EEPGD = 0;
    EECON1bits.RD = 1;
    return EEDAT;
}

void write_EEPROM(uint8_t ADRESS, uint8_t data){
    
    while(EECON1bits.WR);
    EEADR = ADRESS;
    EEDAT = data;
    EECON1bits.EEPGD = 0;
    EECON1bits.WREN = 1;
    
    INTCONbits.GIE = 0;
    EECON2 = 0x55;
    EECON2 = 0xAA;
    
    
    EECON1bits.WR = 1;
    INTCONbits.RBIF = 0;
    INTCONbits.GIE = 1;
    //SLEEP();
    EECON1bits.WREN = 0;
   
}
