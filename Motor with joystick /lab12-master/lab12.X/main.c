/* 
 * File:   main.c
 * Author: amr usef
 * lab:12
 *date: 12/3/20
 * Discription: This is the main file of this lab, in our main file we want to be able to display on to the LCD screen, 
 * get messages from controller via raspary *pie, and to write to the motor to move forwards or backword. 
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
#pragma FWDTEN = OFF

static volatile int first_time_in=1;//for the condation to display 
static volatile float precent_gamepad_x =0.0, past_x=0.0;
static volatile float precent_gamepad_y =0.0, past_y=0.0;

void __ISR(_SPI_4_VECTOR, IPL1SOFT) SPI4Handler(void){
    //check to see if there was a receive
    if(IFS1bits.SPI4RXIF){
        int done =0;//incidates message is recived
        int i=0;
        char strread[20];// string to read the message sent by ras pi
        
        while(!done){
            while(!SPI4STATbits.SPIRBF){
             ;
             }
        strread[i] = SPI4BUF;// reading the buffer and saving it to strread
        if(strread[i]=='\n'){
            done=1;//have reached end of message
        }
        i++;
        }
        char *token = strtok(strread,":");//breaking up data by ":"
        int x_check=strcmp("GX", token);//extacting GX into cmp
        int y_check=strcmp("GY", token);//extacting GX into cmp
        char*value = strtok(NULL, ":");
        int sh = atoi(value);
        if(x_check==0){
            precent_gamepad_x =(float) (sh)/32768.0;
        }else if(y_check==0){
            precent_gamepad_y =(float) (sh)/32768.0;
        }
        
        //motion control
        if(precent_gamepad_y<-.1){//moving back
             LATFbits.LATF0=1;//F0=1
             LATFbits.LATF1=0;//F1=0
             int speed =(int) (0-precent_gamepad_y)*1023;
             OC4RS = speed;
        }else if(precent_gamepad_y>.1){//moving forwards
            LATFbits.LATF0=0;//F0=0
            LATFbits.LATF1=1;//F1=1
            int speed =(int) (0+precent_gamepad_y)*1023;
            OC4RS = speed;
        }else{//no movement
            LATFbits.LATF0=0;//F0=0
            LATFbits.LATF1=0;//F1=0
        }
         IFS1bits.SPI4RXIF=0;//clear RX flag    
    }else if(IFS1bits.SPI4TXIF){
        IFS1bits.SPI4TXIF =0; //clear TX flag
    }else{
        //do nothing 
    }
    }

int main(void) {
    TRISE = 0xFF00;//tri-state E for output config for LCD
    DDPCON =0x0;   //debugging tool used when ever calling Tri-state registers or using LATx
    LATA =0x0; //set the value of lATA to zero
    char disp[20];//to display the X and Y corrd on LCD screen 
    lcd_display_driver_initialize();//calling to initialize the lcd display 
    
    TRISFbits.TRISF0=0;//F0 as an ouput
    TRISFbits.TRISF1=0;//F0 as an ouput

    INTCONbits.MVEC = 1;//allow many interrputs 
    
    
    __builtin_disable_interrupts();
    PR3 = 1022;//sets period
    TMR3 = 0;//start at 0
    T3CONbits.TCKPS = 0b011;//prescaler-8
    OC4CONbits.OCM = 0b110;//no fault pins, PWM
    OC4CONbits.OCTSEL = 1;//to use timer3 for timing 
    OC4R= 0;//set to be used with potenitmeter 
    OC4RS = 0;//set to be used with potenitmeter 
    T3CONbits.ON = 1;//turn on timer
    OC4CONbits.ON = 1;//turn on timer
    
    //Slave - SPI4
    SPI4BUF;    //clear the rx buffer be reading from it
    SPI4STATbits.SPIROV=0; //clear the overflow bit
    SPI4CONbits.CKP=0;//clock is idle when low, active when high
    SPI4CONbits.DISSDO=1;//SDO4 is disabled 
    SPI4CONbits.CKE=0;//clock edge select bit
    SPI4CONbits.SRXISEL=1;
    //operating in 8 bits
    SPI4CONbits.MODE16 =0;
    SPI4CONbits.MODE32 =0;
    SPI4CONbits.MSTEN=0; //operating in slave mode
    SPI4CONbits.ON=1;
    
    //SPI 4 interrupt 
    IPC8bits.SPI4IP = 1; //for now
    IPC8bits.SPI4IS=1; 
    IFS1bits.SPI4RXIF=0; //clear flag
    IEC1bits.SPI4RXIE=1; //enable
    __builtin_enable_interrupts();
    
  while(1){
    sprintf(disp,"X:%5.2f Y:%5.2f",precent_gamepad_x, precent_gamepad_y);
    if(first_time_in=1 ||precent_gamepad_x!= past_x||precent_gamepad_y!=past_y){
             lcd_display_driver_clear();//clear before writing 
             lcd_display_driver_write(disp, strlen(disp));//writing into the first line
             past_x=precent_gamepad_x;//for conditon to be checked
             past_y=precent_gamepad_y;//for condation to be checked 
             first_time_in++;//to be used to print on to the LCD screen on first run in timer 2 ;
     }
  }
    return (0);
}
