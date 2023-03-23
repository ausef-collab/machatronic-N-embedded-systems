/* 
 * File:   main.c
 * Author: amr usef
 * lab:11
 *date: 11/23/20
 * Discription: in the main file what we are dong is using UART communcation to 
 * write to a servomotor controller. Also, we are using the potentometer to adjust the 
 * dirction of the servomotor by either +45 or -45 degrees. 
 */
      

#include <stdio.h>
#include <stdlib.h>
#include <sys/attribs.h>
#include <xc.h>
#include <string.h>
#include "lcd_display_driver.h"
//Furn CPU at 80MHz
#pragma config POSCMOD = HS
#pragma config FNOSC = PRIPLL
#pragma config FPLLMUL = MUL_20
#pragma config FPLLIDIV = DIV_2
#pragma config FPLLODIV = DIV_1
#pragma config FPBDIV = DIV_1

#define SAMPLE_TIME 10 //250 ns
#define BAUD 9600
#define FREQ_SYSTEM 80000000

static volatile float pervious_vaule=0.0, current_vaule=0.0, first_time=1.0; //to allow the lcd to display with out flackring 

unsigned int ADC_CONVERTER(void){
    unsigned int elapsed =0, finish_time =0;//to be used in my sampling 
    AD1CHSbits.CH0SA = 2;//MUXA pin
    AD1CON1bits.SAMP = 1;// start sampling
    elapsed =_CP0_GET_COUNT();//to get current time
    finish_time = elapsed + SAMPLE_TIME;//to set to get finish time
    while(_CP0_GET_COUNT() < finish_time){
        ;
    }
    AD1CON1bits.SAMP = 0;// stop sampling
    while(!AD1CON1bits.DONE){
        ;
    }
    return ADC1BUF0; 
}
void write_message(char message){
        while(U2STAbits.UTXBF){
            ;//wait until TX buffer isn't full
        }
        U2TXREG=message;
}
void __ISR(_TIMER_2_VECTOR, IPL4SOFT) Timer2ISR(void) {
    char hex_val[100];//store the target angle to be used later on
    int val =0;
    uint8_t lower_bits=0;
    uint8_t upper_bits=0;
    current_vaule=ADC_CONVERTER();
    float div= (current_vaule/1023.0);
    val = (int)(4*1000)*(1+div);//operating in a quator of micro sec
    lower_bits = val & 0x7F;
    upper_bits =(val >> 7) & 0x7F;
    
    //display the target degree and angle 
    if(first_time=1.0 ||pervious_vaule!= current_vaule){
    sprintf(hex_val,"%x %x %x %x %x %x",0xAA,0xC,0x04,0x0,lower_bits,upper_bits);   
    lcd_display_driver_clear();//clear before writing 
    lcd_display_driver_write(hex_val, strlen(hex_val));//writing into the first line
    write_message(0xAA);//start byte 
    write_message(0xC);//device ID
    write_message(0x04);//Command byte
    write_message(0x0);//using channel 0
    write_message(lower_bits);
    write_message(upper_bits);
    pervious_vaule=current_vaule;//for conditon to be checked 
    first_time=first_time+1.0;//to be used to print on to the LCD screen on first run in timer
    }
    IFS0bits.T2IF = 0;//clear timer interrupt flag
    return;
}
int main() {
    TRISE = 0xFF00;//tri-state E for output config for LCD
    DDPCON =0x0;   //debugging tool used when ever calling Tri-state registers or using LATx
    lcd_display_driver_initialize();//calling to initialize the lcd display 
    
    INTCONbits.MVEC=1;
     __builtin_disable_interrupts();
    U2MODEbits.BRGH=0;
    U2BRG =((FREQ_SYSTEM /BAUD)/16)-1;//((80*10^6)/(9600)/(16))-1=520
    //8 bits, no parity, and 1 stop bit
    U2MODEbits.PDSEL=0; 
    U2MODEbits.STSEL=0;
    U2STAbits.UTXEN=1;//enable TX
    U2STAbits.URXEN=1;//enable RX
    U2MODEbits.UEN=0;//only U2TX and U2RX pins to be used
    U2MODEbits.ON=1; //turn on UART
    
    PR2=31249; //(80MHz*(1/10)*(1/256))-1=PR2 -> 31249
    TMR2 = 0;//initializes count to zero
    T2CONbits.TCKPS = 7;//prescaler 256
    T2CONbits.TGATE = 0;
    T2CONbits.TCS = 0;
    T2CONbits.ON = 1;//time on
    IPC2bits.T2IP = 4;//priority 4  
    IPC2bits.T2IS = 0;//sub-priority
    IFS0bits.T2IF = 0;//clears flag
    IEC0bits.T2IE = 1;//enables interrupt
    __builtin_enable_interrupts();
    
    while(1){
        ;
    }
    
    return (0);
}

