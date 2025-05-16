// ================================================
// Proyecto: LM35 + LCD + PWM + Encoder + UART
// UART a 9600 con BRGH=0, SPBRG=38 (Fosc=24 MHz)
// ================================================

// ========== CONFIG BITS ==========
#pragma config PLLDIV   = 5        // 20 MHz?4 MHz PLL
#pragma config CPUDIV   = OSC1_PLL2// 48 MHz/2 = 24 MHz CPU
#pragma config FOSC     = HSPLL_HS // HS osc + PLL
#pragma config PWRT     = ON
#pragma config WDT      = OFF
#pragma config MCLRE    = ON
#pragma config PBADEN   = OFF
#pragma config LVP      = OFF
#pragma config DEBUG    = OFF
#pragma config XINST    = OFF

#include <xc.h>
#include <stdio.h>
#include <string.h>
#include "LCD.h"

#define _XTAL_FREQ 24000000UL  // ahora 24 MHz

// rango PWM
#define TEMP_MIN_C 21.0f
#define TEMP_MAX_C 40.0f
#define PWM_MIN    15
#define PWM_MAX    235

volatile unsigned int pulseCount = 0;

// prototipos
void ADC_Init(void);
unsigned int ADC_Read(unsigned char ch);
float ADC_to_Volts(unsigned int v);
float Volts_to_Temp(float v);
void PWM1_Init(void);
void PWM1_SetDuty(unsigned char d);
void UART_Init(void);
void UART_SendString(const char *s);
void Encoder_Init(void);

void __interrupt() isr(void) {
    if (INTCONbits.INT0IF) {
        pulseCount++;
        INTCONbits.INT0IF = 0;
    }
}

void main(void) {
    char buf[32];
    unsigned int raw, pulses;
    float volt, temp;
    unsigned char duty;
    int t10, ti, td;

    // LCD
    TRISE = 0; TRISD = 0;
    LCD_CONFIG(); BORRAR_LCD();

    // perifericos
    ADC_Init();
    PWM1_Init();
    UART_Init();
    Encoder_Init();

    while(1) {
        pulseCount = 0;
        __delay_ms(1000);
        pulses = pulseCount;

        raw  = ADC_Read(0);
        volt = ADC_to_Volts(raw);
        temp = Volts_to_Temp(volt);

        if (temp <= TEMP_MIN_C) duty = PWM_MIN;
        else if (temp >= TEMP_MAX_C) duty = PWM_MAX;
        else
            duty = PWM_MIN +
                   (unsigned char)((temp - TEMP_MIN_C)
                       * (PWM_MAX - PWM_MIN)
                       / (TEMP_MAX_C - TEMP_MIN_C) + .5f);
        PWM1_SetDuty(duty);

        t10 = (int)(temp*10 + .5f);
        ti  = t10/10; td = t10%10;

        BORRAR_LCD();
        POS_CURSOR(1,0);
        sprintf(buf,"T:%2d.%1dC",ti,td);
        ESCRIBE_MENSAJE(buf,strlen(buf));

        sprintf(buf,"Speed:%4u pps\r\n",pulses);
        UART_SendString(buf);
    }
}

void ADC_Init(void) {
    ADCON1 = 0b00001110; // AN0 analog, resto digital
    ADCON2 = 0b10111001; // ACQT=111, ADCS=100 (Fosc/4), ADFM=1
    ADCON0 = 0b00000001; // ADON
    __delay_ms(2);
}

unsigned int ADC_Read(unsigned char ch) {
    ADCON0 = (ch<<2) | 1;
    __delay_us(50);
    ADCON0bits.GO_DONE = 1;
    while(ADCON0bits.GO_DONE);
    return (ADRESH<<8) | ADRESL;
}

float ADC_to_Volts(unsigned int v) {
    return v * (5.0f/1023.0f);
}

float Volts_to_Temp(float v) {
    return v*75.47f - 0.93f;
}

void PWM1_Init(void) {
    PR2 = 255;
    CCP1CON = 0x0C;
    T2CON = 0b00000100; // presc=1:1, TMR2 off
    T2CONbits.TMR2ON=1;
    TRISC2 = 0;
}

void PWM1_SetDuty(unsigned char d) {
    CCPR1L = d;
    CCP1CONbits.DC1B=0;
}

void UART_Init(void) {
    TRISC6=0; TRISC7=1;
    SPBRG = 38;           // (24MHz/(64*9600))-1
    TXSTAbits.BRGH=0;     // low speed
    BAUDCONbits.BRG16=0;  // 8-bit
    TXSTAbits.SYNC=0;
    RCSTAbits.SPEN=1;
    TXSTAbits.TX9=0; RCSTAbits.RX9=0;
    TXSTAbits.TXEN=1; RCSTAbits.CREN=1;
    PIR1bits.RCIF=0; PIR1bits.TXIF=0;
}

void UART_SendString(const char *s) {
    while(*s) {
        while(!TXSTAbits.TRMT);
        TXREG=*s++;
    }
}

void Encoder_Init(void) {
    TRISB0=1;
    INTCON2bits.INTEDG0=1;
    INTCONbits.INT0IF=0;
    INTCONbits.INT0IE=1;
    INTCONbits.GIE=1;
}
